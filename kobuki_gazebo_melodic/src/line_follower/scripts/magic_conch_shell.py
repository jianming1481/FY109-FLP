#!/usr/bin/env python  

# ROS
import roslib
roslib.load_manifest('mag_recoder')
import rospy
import actionlib
import move_base_msgs.msg
import tf
from tf.transformations import quaternion_from_euler
from geometry_msgs.msg import PoseStamped
from math import *

def calculate_path(index):
    listener = tf.TransformListener()
    try:
        # Listening TF may need some time 
        listener.waitForTransform("map", "base_footprint", rospy.Time(), rospy.Duration(1.5))
        (trans,rot) = listener.lookupTransform('map', 'base_footprint', rospy.Time(0))
    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        print("what happened?")
        return 0
    
    max_dist = 2
    current_dist = max_dist - 0.2*index
    
    target_pose = PoseStamped()
    # target_pose.header.seq = index
    target_pose.header.stamp = rospy.Time.now()
    target_pose.header.frame_id = 'map'
    target_pose.pose.position.x = current_dist*sin(1.5707*index) + trans[0]
    target_pose.pose.position.y = current_dist*cos(1.5707*index) + trans[1]
    q = quaternion_from_euler(0.0, 0.0, -1.5707*index)
    target_pose.pose.orientation.x = q[0]
    target_pose.pose.orientation.y = q[1]
    target_pose.pose.orientation.z = q[2]
    target_pose.pose.orientation.w = q[3]
    
    return target_pose

def movebase_client(index):
    client = actionlib.SimpleActionClient('move_base', move_base_msgs.msg.MoveBaseAction)

    # Waits until the action server has started up and started
    # listening for goals.
    client.wait_for_server()

    if index==0:
        # Creates a goal to send to the action server.
        goal = move_base_msgs.msg.MoveBaseActionGoal().goal
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.orientation.w = 1
        # Sends the goal to the action server.
        client.send_goal(goal)
    else:
        # Creates a goal to send to the action server.
        goal = move_base_msgs.msg.MoveBaseActionGoal().goal
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose = calculate_path(index)
        # Sends the goal to the action server.
        client.send_goal(goal)
    
    wait = client.wait_for_result()
    if not wait:
        rospy.logerr("Action server not available!")
        rospy.signal_shutdown("Action server not available!")
    else:
        return client.get_result()

if __name__ == '__main__':
    try:
        rospy.init_node('magic_conch_shell_node')
        for i in range(10):
            print("Current Round: ", i)
            result = movebase_client(i)
            if result:
                rospy.loginfo("Goal execution done!")
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation test finished.")