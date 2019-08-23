#!/usr/bin/env python  
# ROS
import roslib
roslib.load_manifest('mag_recoder')
import rospy
import tf
from sensor_msgs.msg import MagneticField
from sensor_msgs.msg import Imu

# Utils
import math
import numpy as np

rotation_matrix = np.zeros([4,4])
mag_data = [0.0, 0.0, 0.0]
imu_orientation_data = [0.0, 0.0, 0.0, 0.0]
theta = 0.0

def mag_CB(data):
    global mag_data
    mag_data[0] = data.magnetic_field.x
    mag_data[1] = data.magnetic_field.y
    mag_data[2] = data.magnetic_field.z

def imu_CB(data):
    global imu_orientation_data
    global rotation_matrix
    imu_orientation_data[0] = data.orientation.x
    imu_orientation_data[1] = data.orientation.y
    imu_orientation_data[2] = data.orientation.z
    imu_orientation_data[3] = data.orientation.w
    rotation_matrix = tf.transformations.quaternion_matrix(imu_orientation_data)
    
    
if __name__ == '__main__':
    # Start ROS Node
    rospy.init_node('mag_recoder')
    listener = tf.TransformListener()
    rospy.Subscriber("imu/mag",MagneticField, mag_CB)
    rospy.Subscriber("imu/data",Imu, imu_CB)
    pub = rospy.Publisher('m_mag', MagneticField, queue_size=1)
    rate = rospy.Rate(10) # 10hz
    # global theta

    while not rospy.is_shutdown():
        cal_mag_data = [mag_data[0], mag_data[1], mag_data[2], 0.0]
        new_mag_vector = np.dot(rotation_matrix, cal_mag_data)
        m_mag = MagneticField()
        m_mag.header.frame_id = 'map'
        m_mag.magnetic_field.x = new_mag_vector[0]
        m_mag.magnetic_field.y = new_mag_vector[1]
        m_mag.magnetic_field.z = new_mag_vector[2]
        pub.publish(m_mag)
        rate.sleep()
