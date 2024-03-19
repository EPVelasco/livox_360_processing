#include <ros/ros.h>
#include <limits>
#include <cv_bridge/cv_bridge.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <image_transport/image_transport.h>
#include <pcl/point_types.h>
#include <pcl/range_image/range_image.h>
#include <pcl/range_image/range_image_spherical.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/impl/point_types.hpp>
#include <opencv2/core/core.hpp>
#include <iostream>
#include <Eigen/Dense>
#include <math.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl/filters/statistical_outlier_removal.h>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <armadillo>

#include <chrono> 

// ransac
#include <pcl/console/parse.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/io/pcd_io.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/sample_consensus/sac_model_sphere.h>

// ceres interpolation
#include "ceres/ceres.h"
#include "ceres/cubic_interpolation.h"

using Grid = ceres::Grid2D<double>;
using Interpolator = ceres::BiCubicInterpolator<Grid>;


typedef std::chrono::high_resolution_clock Clock;

using namespace Eigen;
using namespace sensor_msgs;
using namespace std;

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

//Publisher
ros::Publisher pc_pub;
ros::Publisher imgD_pub;
ros::Publisher img_mask_pub;
ros::Publisher img_z_pub;

float maxlen =100.0;     //maxima distancia del lidar
float minlen = 0.01;     //minima distancia del lidar
float max_FOV = 3.0;     // en radianes angulo maximo de vista de la camara
float min_FOV = 0.4;     // en radianes angulo minimo de vista de la camara

/// parametros para convertir nube de puntos en imagen
float angular_resolution_x =0.5f;
float angular_resolution_y = 2.1f;
float max_angle_width= 360.0f;
float max_angle_height = 180.0f;

float interpol_value = 20.0;
float ang_x_lidar = 0.6*M_PI/180.0; 
double max_var = 50.0; 
bool f_pc = true; 
int S = 5;
int times_roi = 5;
cv::Mat img_height_bk, img_range_bk;
cv::Mat img_range;


// topics a suscribirse del nodo
std::string pcTopic = "/livox/lidar";

// range image parametros
boost::shared_ptr<pcl::RangeImageSpherical> rangeImage;
pcl::RangeImage::CoordinateFrame coordinate_frame = pcl::RangeImage::LASER_FRAME;

///////////////////////////////////////callback
void callback(const PointCloud::ConstPtr& msg_pointCloud)
{
  //Conversion from sensor_msgs::PointCloud2 to pcl::PointCloud<T>
  /*pcl::PCLPointCloud2 pcl_pc2;
  pcl_conversions::toPCL(*in_pc2,pcl_pc2);
  PointCloud::Ptr msg_pointCloud(new PointCloud);
  pcl::fromPCLPointCloud2(pcl_pc2,*msg_pointCloud);*/
  ///

  ////// filter point cloud 
  if (msg_pointCloud == NULL) return;

  PointCloud::Ptr cloud_in (new PointCloud);
  PointCloud::Ptr cloud_out (new PointCloud);

  std::vector<int> indices;
  pcl::removeNaNFromPointCloud(*msg_pointCloud, *cloud_in, indices);

  float max_z = 0, min_z = std::numeric_limits<float>::infinity();
  float max_dis = 0, min_dis = std::numeric_limits<float>::infinity();
  
  for (int i = 0; i < (int) cloud_in->points.size(); i++)
  {
      double distance = sqrt(cloud_in->points[i].x * cloud_in->points[i].x + cloud_in->points[i].y * cloud_in->points[i].y);     
      if(distance<minlen || distance>maxlen)
       continue;

      cloud_out->push_back(cloud_in->points[i]);     
      if(cloud_in->points[i].z >max_z)
        max_z = cloud_in->points[i].z;
      if(cloud_in->points[i].z <min_z)
        min_z = cloud_in->points[i].z;
      if(distance>max_dis)
        max_dis = distance;
      if(distance<min_dis)
        min_dis = distance;    
  }  

  /*filtrado de la nube
  pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor_stat;
  sor_stat.setInputCloud (cloud_out);
  sor_stat.setMeanK (10.0);
  sor_stat.setStddevMulThresh (1.0);
  sor_stat.filter (*cloud_out);*/

     //filtrado de la nube
  /*pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
  sor.setInputCloud (cloud_filter);
  sor.setMeanK (50.0);
  sor.setStddevMulThresh (0.1);
  sor.filter (*cloud_out);*/

  //                                                  point cloud to image 

  //============================================================================================================
  //============================================================================================================

  // range image    

  Eigen::Affine3f sensorPose = (Eigen::Affine3f)Eigen::Translation3f(0.0f, 0.0f, 0.0f);
  rangeImage->pcl::RangeImage::createFromPointCloud(*cloud_out, pcl::deg2rad(angular_resolution_x), pcl::deg2rad(angular_resolution_y),
                                       pcl::deg2rad(max_angle_width), pcl::deg2rad(max_angle_height),
                                       sensorPose, coordinate_frame, 0.0f, 0.0f, 0);

  int cols_img = rangeImage->width;
  int rows_img = rangeImage->height;


  arma::mat Z;  // interpolation de la imagen
  arma::mat Zz; // interpolation de las alturas de la imagen

  Z.zeros(rows_img,cols_img);         // rango
  Zz.zeros(rows_img,cols_img);        // altura

  Eigen::MatrixXf ZZei (rows_img,cols_img);

  float max_depth = 0.0;
  float min_depth = -999.0;
 
  for (int i=0; i< cols_img; ++i)
      for (int j=0; j<rows_img ; ++j)
      {
        float r =  rangeImage->getPoint(i, j).range;     
        float zz = rangeImage->getPoint(i, j).z; 

       
        Eigen::Vector3f tmp_point;
        rangeImage->calculate3DPoint (float(i), float(j), r, tmp_point);

        //float zz = tmp_point[2]; 

        if(std::isinf(r) || r<minlen || r>maxlen || std::isnan(zz)){
            continue;
        }             
        Z.at(j,i) = r;   
        Zz.at(j,i) = zz;
        ZZei(j,i)=zz;

      if(r>max_depth)
        max_depth = r;
      if(r<min_depth)
        min_depth = r;      

      }

    cv::Mat interdephtfilter = cv::Mat::zeros(Z.n_rows, Z.n_cols, cv_bridge::getCvType("mono16"));
    cv::Mat heightImage      = cv::Mat::zeros(Z.n_rows, Z.n_cols, cv_bridge::getCvType("mono16"));
    cv::Mat interdephtImage  = cv::Mat::zeros(Z.n_rows, Z.n_cols, cv_bridge::getCvType("mono16"));

  
  arma::mat img_range_bk = Z;
  arma::mat img_height_bk = Zz;

 
    for (int r = 0; r < times_roi; r++) {
        arma::mat Znew = Z;
        arma::mat Zznew = Zz;
        for (int j = S; j < Z.n_rows - S; j++) {
            for (int i = S; i < Z.n_cols - S; i++) {
                if (Znew(j, i) == 0) {
                    // Definir (ROI)
                    arma::mat cropped_ra =  Znew(arma::span(j - 1, j + S - 2), arma::span(i - 1, i + S - 2));
                    arma::mat cropped_re = Zznew(arma::span(j - 1, j + S - 2), arma::span(i - 1, i + S - 2));

                    double N_ra = 0.0;
                    double N_re = 0.0;
                    double val_ra = 0.0;
                    double val_re = 0.0;
                    // Calcular la media de los valores en el ROI
                    for (int v = 0; v < S; v++) {
                        for (int u = 0; u < S; u++) {
                            if (cropped_ra(v, u) != 0) {
                                N_ra += 1.0;
                                val_ra += (double)(cropped_ra(v, u));
                            }
                            if (cropped_re(v, u) != 0) {
                                N_re += 1.0;
                                val_re += (double)(cropped_re(v, u));
                            }
                        }
                    }
                    // Asignar el valor promedio de los datos diferentes a 0
                    if (N_ra > 0.0){
                        img_range_bk.at(j, i) = val_ra / N_ra;
                    }
                    if (N_re > 0.0){
                        img_height_bk.at(j, i) = val_re / N_re;
                    }
                }
            }
        }

    }


  for (int i=0; i< Z.n_cols; ++i)
    for (int j=0; j<Z.n_rows ; ++j){
      
        interdephtImage.at<float>(j, i)  = Z(j,i); 
        heightImage.at<float>(j, i)      = img_height_bk(j,i); 
        interdephtfilter.at<float>(j, i) = img_range_bk(j,i); 

    }

  Z = img_range_bk; 
  Zz = img_height_bk; 


  /////////////////////////////////////////////// resize oepncv

  int factor = 5;

  // Calcular las nuevas dimensiones de la imagen
  cv::Size nuevaDimension(interdephtfilter.cols, interdephtfilter.rows * factor);

  // Aplicar la interpolación bicúbica para aumentar la resolución
  cv::Mat imagenAumentada;
  cv::resize(interdephtfilter, imagenAumentada, nuevaDimension, 0, 0, cv::INTER_CUBIC);

  cv::Mat z_inter;
  cv::resize(heightImage, z_inter, nuevaDimension, 0, 0, cv::INTER_CUBIC);

  ////////////////////////////////////// Inerpolacion con opencv

  PointCloud::Ptr point_cloud (new PointCloud);
  PointCloud::Ptr P_out (new PointCloud);
  point_cloud->width = imagenAumentada.cols; 
  point_cloud->height = imagenAumentada.rows;
  point_cloud->is_dense = false;
  point_cloud->points.resize (point_cloud->width * point_cloud->height);


  std::cout<<"Tamanno img: "<<imagenAumentada.rows<<", "<<imagenAumentada.cols<<std::endl;
  std::cout<<"Tamanno z  : "<<z_inter.rows<<", "<<z_inter.cols<<std::endl;

  int num_pc = 0; // numero de elementos en pointcloud
  for (int i=0; i< imagenAumentada.rows - factor; i+=1)
   {       
      for (int j=0; j<imagenAumentada.cols ; j+=1)
      {

        float ang = M_PI-((2.0 * M_PI * j )/(imagenAumentada.cols));

        if(imagenAumentada.at<float>(i,j) > 1e-6 ){  
          std::cout<<"img: "<< imagenAumentada.at<float>(i,j)<<std::endl;
          float pc_modulo = imagenAumentada.at<float>(i,j);
          float pc_x = sqrt(pow(pc_modulo,2)- pow(z_inter.at<float>(i,j),2)) * cos(ang);
          float pc_y = sqrt(pow(pc_modulo,2)- pow(z_inter.at<float>(i,j),2)) * sin(ang);

          /// transformacion de resultado para correguir error de paralelismo con el suelo//  
//          Eigen::MatrixXf Lidar_matrix(3,3); //matrix de transforamcion para lidar a rango de imagen () se gira los angulos que tiene de error con respoecto al suelo
          Eigen::MatrixXf result(3,1);
          // Lidar_matrix <<   cos(ang_x_lidar) ,0                ,sin(ang_x_lidar),
          //                   0                ,1                ,0,
          //                   -sin(ang_x_lidar),0                ,cos(ang_x_lidar) ;
      
          result << pc_x,
                    pc_y,
                    z_inter.at<float>(i,j);
            
          //result = Lidar_matrix*result;  // rotacion en eje X para correccion

          point_cloud->points[num_pc].x = result(0);
          point_cloud->points[num_pc].y = result(1);
          point_cloud->points[num_pc].z = result(2);
          P_out->push_back(point_cloud->points[num_pc]); 

          num_pc++;
          std::cout<<"z: "<< z_inter.at<float>(i,j)<<std::endl;
        }
      }
   }  
   std::cout<<"Salio "<<std::endl;
  //////////////////////////////==================================================================================

  int size_inter_Lidar = (int) P_out->points.size(); 
  uint px_data = 0; uint py_data = 0;
  pcl::PointXYZ point;

  /*PointCloud::Ptr pc_interpoled (new PointCloud);
  


  for (int i = 0; i < size_inter_Lidar; i++)
  {
      point.x = P_out->points[i].x;
      point.y = P_out->points[i].y;
      point.z = P_out->points[i].z;      
      pc_interpoled->points.push_back(point);     
      
  }*/

     //filremove noise of point cloud
  
  // pcl::StatisticalOutlierRemoval<pcl::PointXYZ> filt;
  // filt.setInputCloud (P_out);
  // filt.setMeanK (interpol_value);
  // filt.setStddevMulThresh (50.0);
  // filt.filter (*P_out);

  // // dowsmapling
  // PointCloud::Ptr pc_filter (new PointCloud);
  // pcl::VoxelGrid<pcl::PointXYZ> sor;
  // sor.setInputCloud (P_out);
  // sor.setLeafSize (0.1f, 0.1f, 0.1f);
  // sor.filter (*P_out);

  P_out->is_dense = true;
  P_out->width = (int) P_out->points.size();
  P_out->height = 1;
  P_out->header.frame_id = "livox_frame";
  pc_pub.publish (P_out);


  // imagen interpolada
  sensor_msgs::ImagePtr image_msg;
  image_msg = cv_bridge::CvImage(std_msgs::Header(),"mono16", imagenAumentada).toImageMsg();
  imgD_pub.publish(image_msg);

  // mascara de la imagen donde se tiene 0:
  image_msg = cv_bridge::CvImage(std_msgs::Header(),"mono16", interdephtfilter).toImageMsg();
  img_mask_pub.publish(image_msg);


  // mascara de la imagen donde se tiene 0:
  image_msg = cv_bridge::CvImage(std_msgs::Header(),"mono16", heightImage).toImageMsg();
  img_z_pub.publish(image_msg);


  //auto t2= Clock::now();
 //std::cout<< std::chrono::duration_cast<std::chrono::nanoseconds>(t2-t1).count()/1000000.0<<std::endl;
 // rate.sleep();
}


int main(int argc, char** argv)
{

  ros::init(argc, argv, "InterpolatedPointCloud");
  ros::NodeHandle nh;  
  
  /// Load Parameters

  nh.getParam("/maxlen", maxlen);
  nh.getParam("/minlen", minlen);
  nh.getParam("/pcTopic", pcTopic);
  nh.getParam("/x_resolution", angular_resolution_x);
  nh.getParam("/y_interpolation", interpol_value);
  nh.getParam("/max_angle_width", max_angle_width);
  nh.getParam("/max_angle_height", max_angle_height);

  nh.getParam("/ang_Y_resolution", angular_resolution_y);
  nh.getParam("/ang_ground", ang_x_lidar);
  nh.getParam("/max_var", max_var);  
  nh.getParam("/filter_output_pc", f_pc);

  nh.getParam("/ROI", S);
  nh.getParam("/Times_ROI", times_roi);

  ros::Subscriber sub = nh.subscribe<PointCloud>(pcTopic, 10, callback);
  rangeImage = boost::shared_ptr<pcl::RangeImageSpherical>(new pcl::RangeImageSpherical);
  pc_pub = nh.advertise<PointCloud> ("/pc_interpoled", 10);  

  imgD_pub = nh.advertise<sensor_msgs::Image>("/pc2imageInterpol", 10);

  img_mask_pub = nh.advertise<sensor_msgs::Image>("/filter_image", 10);

  img_z_pub = nh.advertise<sensor_msgs::Image>("/z_image", 10);

  ros::spin();

}