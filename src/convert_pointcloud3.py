#!/usr/bin/env python
import rospy
from sensor_msgs.msg import PointCloud2, PointField
import struct
import numpy as np

def callback(data):
    # Convertimos el PointCloud2 ajustando los campos 'line' a 'ring' y 'timestamp' a 'time'
    new_fields = []
    # Inicializar el ajuste del offset
    offset_adjustment = 0
    for field in data.fields:
        # Ajustar el offset para cada campo despus de remover 'tag'
        if field.name == 'line':
            # Cambiamos el nombre del campo de 'line' a 'ring' y ajustamos su tipo y offset
            # Suponemos que el campo 'ring' debe ser UINT16 (2 bytes) (de uint8 a uint16)
            new_field = PointField(name='ring', offset=16,
                                   datatype=4, count=field.count)
            offset_adjustment += 1  # 'ring' ahora es de 2 bytes, 'line' era de 1 byte
        elif field.name == 'timestamp':
            # Cambiamos el nombre del campo de 'timestamp' a 'time', ajustamos su tipo y offset
            # Suponemos que el campo 'time' debe ser FLOAT32 (4 bytes) (float64 a float32)
            new_field = PointField(name='time', offset=18,
                                   datatype=7, count=field.count)
            # Ajustamos el offset de acuerdo al nuevo tamao de 'time' (FLOAT32)
            offset_adjustment += 4  # timestamp era FLOAT64 (8 bytes), ahora es FLOAT32 (4 bytes)
        elif field.name == 'tag':
            # Si el campo es 'tag', lo omitimos y ajustamos el offset en 1 byte
            offset_adjustment += 1
            continue
        else:
            new_field = field
            #new_field.offset -= offset_adjustment
        new_fields.append(new_field)

    # Crear un nuevo PointCloud2 con los campos actualizados
    transformed_cloud = PointCloud2()
    transformed_cloud.header = data.header
    transformed_cloud.height = data.height
    transformed_cloud.width = data.width
    transformed_cloud.fields = new_fields
    transformed_cloud.is_bigendian = data.is_bigendian
    transformed_cloud.point_step = data.point_step -4#- offset_adjustment
    transformed_cloud.row_step = transformed_cloud.point_step * data.width
    transformed_cloud.is_dense = data.is_dense
    
    # Reestructurar 'data' para eliminar el campo 'tag' y ajustar el campo 'timestamp'
    new_data = bytearray()
    for i in range(0, len(data.data), data.point_step):
        point_data = data.data[i:i+data.point_step]
        
        # x, y, z, intensity (cada uno FLOAT32, 4 bytes)
        new_data.extend(point_data[0:16])
        # ring (UINT16, 2 bytes), leemos los datos originales como UINT8 y los empacamos como UINT16
        line_value = struct.unpack_from('B', point_data, 17)[0]  # Leer 'line' como UINT8
        new_data.extend(struct.pack('H', line_value))  # Convertir y escribir 'ring' como UINT16
       
        # time (FLOAT32, 4 bytes), convertido de FLOAT64
        timestamp = struct.unpack_from('d', point_data, 18)[0] # Leer como FLOAT64
        #time= np.float32(np.float64(timestamp)-np.float64(timestamp) ) # no se xq pero funciona con este
        time= np.float32(np.float64(timestamp)-17e+18 ) # Con este tambien funciona
        new_data.extend(struct.pack('f', time))  # Escribir como FLOAT32

    transformed_cloud.data = bytes(new_data)

    # Publicamos el PointCloud2 transformado
    pub.publish(transformed_cloud)
    #rospy.loginfo("Published transformed PointCloud2")

def listener():
    rospy.init_node('lidar_data_transformer', anonymous=True)
    rospy.Subscriber('/livox/lidar', PointCloud2, callback)
    rospy.spin()

if __name__ == '__main__':
    pub = rospy.Publisher('/livox/lidar_modified', PointCloud2, queue_size=10)
    listener()
