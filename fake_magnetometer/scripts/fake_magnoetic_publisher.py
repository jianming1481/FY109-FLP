#!/usr/bin/env python  
import roslib
roslib.load_manifest('fake_magnetometer')
import rospy
import math
import tf
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Twist

import csv
from sensor_msgs.msg import MagneticField
import numpy as np
import yaml

if __name__ == '__main__':
    rospy.init_node('fake_mag_publisher')

    listener = tf.TransformListener()
    index_pub = rospy.Publisher('mag_index', Twist, queue_size=10)
    mag_pub_0 = rospy.Publisher('m_mag_0', MagneticField, queue_size=1)
    mag_pub_1 = rospy.Publisher('m_mag_1', MagneticField, queue_size=1)
    mag_pub_2 = rospy.Publisher('m_mag_2', MagneticField, queue_size=1)
    # Load Mag Map
    reader = csv.reader(open('/home/lui/catkin_ws/src/FY109-FLP/magnetic_map_data/20191029_dist008/mag_pred_x.csv', "rb"), delimiter=",")
    x = list(reader)
    mag_map_x = np.array(x).astype("float")
    # np.flip(mag_map_x, 1)
    # print("[0][0]: ", mag_map_x[143][100])
    
    reader = csv.reader(open('/home/lui/catkin_ws/src/FY109-FLP/magnetic_map_data/20191029_dist008/mag_pred_y.csv', "rb"), delimiter=",")
    x = list(reader)
    mag_map_y = np.array(x).astype("float")
    # np.flip(mag_map_y, 1)
    
    reader = csv.reader(open('/home/lui/catkin_ws/src/FY109-FLP/magnetic_map_data/20191029_dist008/mag_pred_z.csv', "rb"), delimiter=",")
    x = list(reader)
    mag_map_z = np.array(x).astype("float")
    # np.flip(mag_map_z, 1)
    
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
        
        theta = tf.transformations.euler_from_quaternion(rot)[2]        
        
        br = tf.TransformBroadcaster()
        br.sendTransform((trans[0], trans[1], 0),
                        tf.transformations.quaternion_from_euler(0, 0, 0),
                        rospy.Time.now(),
                        "mag_0_robot",
                        'map'
        )
        
        # print('x: ', trans[0], 'y: ', trans[1], 'yaw: ', theta)
        robot_on_magmap_x = int((trans[0]-origin[0])/resolution)
        robot_on_magmap_y = int((trans[1]-origin[1])/resolution)
        if robot_on_magmap_x < 0 or robot_on_magmap_x > 143 or robot_on_magmap_y < 0 or robot_on_magmap_y > 100:
            # print("######################### Index Out of Bound!!! #########################")
            robot_on_magmap_x = 72
            robot_on_magmap_y = 50
        # print('x_onMap: ', robot_on_magmap_x, 'y_onMap: ', robot_on_magmap_y)
        m_mag = MagneticField()
        m_mag.header.frame_id = 'mag_0_robot'
        m_mag.magnetic_field.x = mag_map_x[robot_on_magmap_x][robot_on_magmap_y]
        m_mag.magnetic_field.y = mag_map_y[robot_on_magmap_x][robot_on_magmap_y]
        m_mag.magnetic_field.z = mag_map_z[robot_on_magmap_x][robot_on_magmap_y]
        mag_pub_0.publish(m_mag)
        
        pose = Twist()
        pose.linear.x = robot_on_magmap_x
        pose.linear.y = robot_on_magmap_y
        index_pub.publish(pose)

        dist = 0.2

        tmp_x = dist*math.cos(theta)+trans[0]
        tmp_y = dist*math.sin(theta)+trans[1]
        mag_1_on_magmap_x = int((tmp_x-origin[0])/resolution)
        mag_1_on_magmap_y = int((tmp_y-origin[0])/resolution)
        m_mag = MagneticField()
        m_mag.header.frame_id = 'mag_1_robot'
        m_mag.magnetic_field.x = mag_map_x[mag_1_on_magmap_x][mag_1_on_magmap_y]
        m_mag.magnetic_field.y = mag_map_y[mag_1_on_magmap_x][mag_1_on_magmap_y]
        m_mag.magnetic_field.z = mag_map_z[mag_1_on_magmap_x][mag_1_on_magmap_y]
        mag_pub_1.publish(m_mag)
        br.sendTransform((tmp_x, tmp_y, 0),
                tf.transformations.quaternion_from_euler(0, 0, 0),
                rospy.Time.now(),
                "mag_1_robot",
                'map'   
        )
        
        tmp_x = -dist*math.cos(theta)+trans[0]
        tmp_y = -dist*math.sin(theta)+trans[1]
        mag_2_on_magmap_x = int((tmp_x-origin[0])/resolution)
        mag_2_on_magmap_y = int((tmp_y-origin[0])/resolution)
        m_mag = MagneticField()
        m_mag.header.frame_id = 'mag_2_robot'
        m_mag.magnetic_field.x = mag_map_x[mag_2_on_magmap_x][mag_2_on_magmap_y]
        m_mag.magnetic_field.y = mag_map_y[mag_2_on_magmap_x][mag_2_on_magmap_y]
        m_mag.magnetic_field.z = mag_map_z[mag_2_on_magmap_x][mag_2_on_magmap_y]
        mag_pub_2.publish(m_mag)
        br.sendTransform((tmp_x, tmp_y, 0),
                tf.transformations.quaternion_from_euler(0, 0, 0),
                rospy.Time.now(),
                "mag_2_robot",
                'map' 
        )
        rate.sleep()