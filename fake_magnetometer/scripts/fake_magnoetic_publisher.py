#!/usr/bin/env python  
import roslib
roslib.load_manifest('fake_magnetometer')
import rospy
import math
import tf
import csv
from sensor_msgs.msg import MagneticField
import numpy as np
import yaml

if __name__ == '__main__':
    rospy.init_node('fake_mag_publisher')

    listener = tf.TransformListener()
    pub = rospy.Publisher('m_mag', MagneticField, queue_size=1)
    
    # Load Mag Map
    reader = csv.reader(open('/home/lui/catkin_ws/src/FY109-FLP/magnetic_map_data/20191029_dist008/mag_pred_x.csv', "rb"), delimiter=",")
    x = list(reader)
    mag_map_x = np.array(x).astype("float")
    np.flip(mag_map_x, 1)
    
    reader = csv.reader(open('/home/lui/catkin_ws/src/FY109-FLP/magnetic_map_data/20191029_dist008/mag_pred_y.csv', "rb"), delimiter=",")
    x = list(reader)
    mag_map_y = np.array(x).astype("float")
    np.flip(mag_map_y, 1)
    
    reader = csv.reader(open('/home/lui/catkin_ws/src/FY109-FLP/magnetic_map_data/20191029_dist008/mag_pred_z.csv', "rb"), delimiter=",")
    x = list(reader)
    mag_map_z = np.array(x).astype("float")
    np.flip(mag_map_z, 1)
    
    # Load Real Map Infomation
    with open('/home/lui/map.yaml', 'r') as stream:
        yam_data = yaml.load(stream)
    # resolution = yam_data['resolution']
    resolution = 0.05
    origin = yam_data['origin']
    
    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        try:
            (trans,rot) = listener.lookupTransform('/map', '/base_link', rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue
        print('x: ', trans[0], 'y: ', trans[1])
        robot_on_magmap_x = int((trans[0]-origin[0])/resolution)
        robot_on_magmap_y = int((trans[1]-origin[1])/resolution)
        print('x_onMap: ', robot_on_magmap_x, 'y_onMap: ', robot_on_magmap_y)
        m_mag = MagneticField()
        m_mag.header.frame_id = 'map'
        m_mag.magnetic_field.x = mag_map_x[robot_on_magmap_y][robot_on_magmap_x]
        m_mag.magnetic_field.y = mag_map_y[robot_on_magmap_y][robot_on_magmap_x]
        m_mag.magnetic_field.z = mag_map_z[robot_on_magmap_y][robot_on_magmap_x]
        pub.publish(m_mag)
        rate.sleep()