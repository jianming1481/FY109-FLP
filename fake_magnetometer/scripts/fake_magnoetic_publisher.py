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
    index0_pub = rospy.Publisher('mag0_index', Twist, queue_size=10)
    index1_pub = rospy.Publisher('mag1_index', Twist, queue_size=10)
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
    with open('/home/lui/map_r1.yaml', 'r') as stream:
        yam_data = yaml.load(stream)
    # resolution = yam_data['resolution']
    resolution = 0.05
    origin = yam_data['origin']
    
    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        try:
            (trans,rot) = listener.lookupTransform('/map', '/base_link', rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            # print("No TF!")
            continue
        # print("working...")
        theta = tf.transformations.euler_from_quaternion(rot)[2]        
        
        br = tf.TransformBroadcaster()
        br.sendTransform((trans[0], trans[1], 0),
                        tf.transformations.quaternion_from_euler(0, 0, 0),
                        rospy.Time.now(),
                        "mag_0_robot",
                        'map'
        )
        
        robot_on_magmap_x = int((trans[0]-origin[0])/resolution)
        robot_on_magmap_y = int((trans[1]-origin[1])/resolution)
        # print(origin[0], origin[1])
        # print('mag0_real_x: ', trans[0], 'mag0_real_y: ', trans[1], 'mag0_map_x', robot_on_magmap_x, 'mag0_map_y', robot_on_magmap_y)
        # print(robot_on_magmap_y, robot_on_magmap_x)
        # if robot_on_magmap_x < 0 or robot_on_magmap_x > 143 or robot_on_magmap_y < 0 or robot_on_magmap_y > 100:
        #     # print("######################### Index Out of Bound!!! #########################")
        #     robot_on_magmap_x = 72
        #     robot_on_magmap_y = 50
        # print('x_onMap: ', robot_on_magmap_x, 'y_onMap: ', robot_on_magmap_y)
        m_mag0 = MagneticField()
        m_mag0.header.frame_id = 'mag_0_robot'
        m_mag0.magnetic_field.x = mag_map_x[robot_on_magmap_y][robot_on_magmap_x]
        m_mag0.magnetic_field.y = mag_map_y[robot_on_magmap_y][robot_on_magmap_x]
        m_mag0.magnetic_field.z = mag_map_z[robot_on_magmap_y][robot_on_magmap_x]
        mag_pub_0.publish(m_mag0)
        
        pose = Twist()
        pose.linear.x = robot_on_magmap_x
        pose.linear.y = robot_on_magmap_y
        index0_pub.publish(pose)

        dist = 0.2
        print('biasX: ', dist*math.cos(theta), 'biasY: ', dist*math.sin(theta))
        tmp_x = dist*math.cos(theta)+trans[0]
        tmp_y = dist*math.sin(theta)+trans[1]
        # print("dist: ", math.hypot(tmp_x-trans[0], tmp_y-trans[1]))
        # print('mag1_x: ', tmp_x, 'mag1_y: ', tmp_y)
        mag_1_on_magmap_x = int((tmp_x-origin[0])/resolution)
        mag_1_on_magmap_y = int((tmp_y-origin[1])/resolution)
        pose1 = Twist()
        pose1.linear.x = mag_1_on_magmap_x
        pose1.linear.y = mag_1_on_magmap_y
        index1_pub.publish(pose1)
        # print('mag1_real_x: ', tmp_x, 'mag1_real_y: ', tmp_y, 'mag1_map_x', mag_1_on_magmap_x, 'mag1_map_y', mag_1_on_magmap_y)

        m_mag1 = MagneticField()
        m_mag1.header.frame_id = 'mag_1_robot'
        m_mag1.magnetic_field.x = mag_map_x[mag_1_on_magmap_y][mag_1_on_magmap_x]
        m_mag1.magnetic_field.y = mag_map_y[mag_1_on_magmap_y][mag_1_on_magmap_x]
        m_mag1.magnetic_field.z = mag_map_z[mag_1_on_magmap_y][mag_1_on_magmap_x]
        mag_pub_1.publish(m_mag1)
        br.sendTransform((tmp_x, tmp_y, 0),
                tf.transformations.quaternion_from_euler(0, 0, 0),
                rospy.Time.now(),
                "mag_1_robot",
                'map'   
        )
        
        tmp_x = -dist*math.cos(theta)+trans[0]
        tmp_y = -dist*math.sin(theta)+trans[1]
        mag_2_on_magmap_x = int((tmp_x-origin[0])/resolution)
        mag_2_on_magmap_y = int((tmp_y-origin[1])/resolution)
        m_mag2 = MagneticField()
        m_mag2.header.frame_id = 'mag_2_robot'
        m_mag2.magnetic_field.x = mag_map_x[mag_2_on_magmap_y][mag_2_on_magmap_x]
        m_mag2.magnetic_field.y = mag_map_y[mag_2_on_magmap_y][mag_2_on_magmap_x]
        m_mag2.magnetic_field.z = mag_map_z[mag_2_on_magmap_y][mag_2_on_magmap_x]
        mag_pub_2.publish(m_mag2)
        br.sendTransform((tmp_x, tmp_y, 0),
                tf.transformations.quaternion_from_euler(0, 0, 0),
                rospy.Time.now(),
                "mag_2_robot",
                'map' 
        )
        print(robot_on_magmap_x, robot_on_magmap_y, mag_1_on_magmap_x, mag_1_on_magmap_y, mag_2_on_magmap_x, mag_2_on_magmap_y)
        rate.sleep()