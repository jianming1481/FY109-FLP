#ifndef _ROBOT_POSE_PUBLISHER_H_
#define _ROBOT_POSE_PUBLISHER_H_

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <tf2/LinearMath/Quaternion.h>

#include "monte_carlo_localization/robot.h"

class RobotPosePublisher
{
public:
    RobotPosePublisher(){}
    ~RobotPosePublisher(){}
    void init()
    {
        robot_pose_publisher = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("robot_pose",1);
    }
    void generate_robot_pose_msgs(Robot robot)
    {
        pose_msgs.header.stamp = ros::Time::now();
        pose_msgs.header.frame_id = "map";
        pose_msgs.pose.pose.position.x = robot.pose.x;
        pose_msgs.pose.pose.position.y = robot.pose.y;

        tf::Quaternion myQuaternion;
        myQuaternion.setRPY( 0, 0, robot.pose.yaw);  // Create this quaternion from roll/pitch/yaw (in radians)
        pose_msgs.pose.pose.orientation.x = myQuaternion.getX();
        pose_msgs.pose.pose.orientation.y = myQuaternion.getY();
        pose_msgs.pose.pose.orientation.z = myQuaternion.getZ();
        pose_msgs.pose.pose.orientation.w = myQuaternion.getW();

        transform.setOrigin( tf::Vector3(robot.pose.x, robot.pose.y, 0.0) );
        transform.setRotation(myQuaternion);
    }
    void publish_msgs()
    {
        robot_pose_publisher.publish(pose_msgs);
        br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "map", "base_link"));
    }
    
private:
    ros::NodeHandle nh;
    ros::Publisher robot_pose_publisher;
    tf::TransformBroadcaster br;
    tf::Transform transform;
    geometry_msgs::PoseWithCovarianceStamped pose_msgs;
};

// tf::TransformBroadcaster RobotPosePublisher::br;

#endif