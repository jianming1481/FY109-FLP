#!/usr/bin/env python  

# ROS
import roslib
roslib.load_manifest('line_follower')
import rospy
import actionlib
import move_base_msgs.msg
import tf
from tf.transformations import quaternion_from_euler
from geometry_msgs.msg import PoseStamped
from math import *
import numpy as np

target_position = np.array([[0.165, -1.674, '0'], [0.165, 0.126, '0'], [0.165, 0.126, '1'], [0.665, 0.126, '1'], [0.665, 0.126, '2'], [0.665, -1.674, '2'], [0.665, -1.674, '3'], [0.165, -1.674, '3'], [0.165, -1.674, '0']])
quaternion_dict = {'0': np.array([0, 0, 0.707, 0.707]), '1': np.array([0, 0, 0, 1]), '2': np.array([0, 0, -0.707, 0.707]), '3': np.array([0, 0, 1, 0])}

def movebase_client(index):
    client = actionlib.SimpleActionClient('move_base', move_base_msgs.msg.MoveBaseAction)

    # Waits until the action server has started up and started
    # listening for goals.
    client.wait_for_server()

    goal = move_base_msgs.msg.MoveBaseActionGoal().goal
    
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose.position.x = float(target_position[index][0])
    goal.target_pose.pose.position.y = float(target_position[index][1])

    # print('Rotation in direction ',quaternion_dict[target_position[index][2]])

    q = quaternion_dict[target_position[index][2]]
    # print('q: ',q[0], ', ', q[1], ', ', q[2], ', ', q[3])
    goal.target_pose.pose.orientation.x = q[0]
    goal.target_pose.pose.orientation.y = q[1]
    goal.target_pose.pose.orientation.z = q[2]
    goal.target_pose.pose.orientation.w = q[3]
    print(goal)
    client.send_goal(goal)
    
    # goal.target_pose.header.frame_id = "map"
    # goal.target_pose.header.stamp = rospy.Time.now()
    # goal.target_pose.pose.position.x = -1.969
    # goal.target_pose.pose.position.y = -0.628

    # print('Rotation in direction ',quaternion_dict[target_position[index][2]])

    # q = quaternion_dict[target_position[index][2]]
    # goal.target_pose.pose.orientation.x = 0
    # goal.target_pose.pose.orientation.y = 0
    # goal.target_pose.pose.orientation.z = 0
    # goal.target_pose.pose.orientation.w = 1
    # client.send_goal(goal)    
    
    
    wait = client.wait_for_result()
    if not wait:
        rospy.logerr("Action server not available!")
        rospy.signal_shutdown("Action server not available!")
    else:
        return client.get_result()

if __name__ == '__main__':
    try:
        rospy.init_node('magic_conch_shell_node')
        target_position.astype(float)
        for i in range(len(target_position)):
        # for i in range(1):
            print("Current Round: ", i)
            result = movebase_client(i)
            if result:
                rospy.loginfo("Goal execution done!")
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation test finished.")
