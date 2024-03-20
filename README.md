# Description
This package developed in ROS is for point cloud processing of a livox360 sensor.

## Compile pacakage.
### Requisites:
- The [Livox_ros_driver](https://github.com/Livox-SDK/livox_ros_driver).

- [Armadillo](http://arma.sourceforge.net/download.html) (11.0.1 or higher) This package is used for data lineal inerpolation.
  ```
    tar -xvf armadillo-11.1.1.tar.xz
    cd armadillo-11.1.1
    mkdir build
    cd build
    cmake ..
    make
    sudo make install
  ```

After that you can clone this repository.

```
    cd ~/catkin_ws/src
    git clone https://github.com/EPVelasco/livox_360_processing.git
    cd ..
    catkin_make
    source ~/catkin_ws/devel/setup.bash
```

## Nodes
### Interpolated point cloud.

#### Suscribed Topics.
*~/livox/lidar* Input Point Cloud message. ([sensor_msgs/PointCloud2](http://docs.ros.org/en/lunar/api/sensor_msgs/html/msg/PointCloud2.html))

#### Published Topics.
*~/pc_interpoled* Output point cloud interpolated. ([sensor_msgs/PointCloud2](http://docs.ros.org/en/lunar/api/sensor_msgs/html/msg/PointCloud2.html))

#### ROS launch.
```
  roslaunch livox_360_processing interpolated_livox.launch
```

### Livox-to-pointcloud2

#### Suscribed Topics.
*~/livox/lidar* Input Point Cloud message. ([livox_ros_driver2/CustomMsg](https://github.com/Livox-SDK/livox_ros_driver2))

#### Published Topics.
*~/livox/pointcloud* Output point cloud converted. ([sensor_msgs/PointCloud2](http://docs.ros.org/en/lunar/api/sensor_msgs/html/msg/PointCloud2.html))

#### ROS Run node.
```
  rosrun livox_360_processing livox_repub
```
