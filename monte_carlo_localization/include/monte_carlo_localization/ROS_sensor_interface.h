#ifndef _ROS_SENSOR_INTERFACE_H_
#define _ROS_SENSOR_INTERFACE_H_

#include <ros/ros.h>
#include "sensor_msgs/LaserScan.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "tf/transform_datatypes.h"

#include "monte_carlo_localization/laser_scan.h"
#include "monte_carlo_localization/robot.h"

using namespace std;

class ROSSensorInterfaces
{
public:
    ROSSensorInterfaces(){}
    ~ROSSensorInterfaces(){}

    void init()
    {
        odom_sub = nh.subscribe<nav_msgs::Odometry>("odom", 1, &ROSSensorInterfaces::odomCallback, this);
        scan_sub = nh.subscribe<sensor_msgs::LaserScan> ("scan", 1, &ROSSensorInterfaces::scanCallback, this);
        init_flag = true;
        // init_pose_sub = nh.subscribe<geometry_msgs::PoseWithCovarianceStamped> (
        //                     "initialpose", 1, &ROSSensorInterfaces::initPoseCallback, this);
    }
    // void initPoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &initpose_in)
    // {

    // }

    void odomCallback(const nav_msgs::Odometry::ConstPtr& odom_in)
    {
        robot.odom.frame_id = odom_in->header.frame_id;
        robot.odom.child_frame_id = odom_in->child_frame_id;

        robot.odom.pose.x = odom_in->pose.pose.position.x;
        robot.odom.pose.y = odom_in->pose.pose.position.y;
        tf::Quaternion quat;
        tf::quaternionMsgToTF(odom_in->pose.pose.orientation , quat);
        double roll, pitch, yaw;
        tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
        robot.odom.pose.yaw = yaw;

        robot.odom.twist.linear.x = odom_in->twist.twist.linear.x;
        robot.odom.twist.linear.y = odom_in->twist.twist.linear.y;
        robot.odom.twist.angular.yaw = odom_in->twist.twist.angular.z;
        calculate_displacement();
    }

    void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan_in)
    {
        scan.data.clear();
        scan.ranges = scan_in->angle_max - scan_in->angle_min;
        // std::cout << "Hokuyo Send: " << scan_in->ranges.size() << std::endl;
        int downsampling_param = 1;
        scan.sample_num = scan_in->ranges.size()/(double)downsampling_param;
        // std::cout << "After downsampling: " << samples << std::endl;

        for(int i=0; i<scan.sample_num; i++)
        {
            scan.data.push_back(scan_in->ranges[downsampling_param*i]);
        }
    }

    void calculate_displacement()
    {
        if(init_flag)
        {
            robot.last_odom.pose = robot.odom.pose;
            init_flag = false;
        }
        double tmp_x = robot.odom.pose.x- robot.last_odom.pose.x;
        double tmp_y = robot.odom.pose.y- robot.last_odom.pose.y;
        double direction = atan2(tmp_y,tmp_x);
        double dist = hypot(tmp_x,tmp_y);
        double robot_yaw = robot.odom.pose.yaw;
        if(abs(direction - robot_yaw)<1.57)
        {
            dist = dist*1;
        }else{
            dist = dist*-1;
        }
        displacement(0) = dist;
        displacement(1) = 0;
        displacement(2) = (robot_yaw - robot.last_odom.pose.yaw);
    }

    Vector3d get_displacement()
    {
        Vector3d tmp_displacement;
        if(init_flag)
        {
            tmp_displacement(0) = 0.0;
            tmp_displacement(1) = 0.0;
            tmp_displacement(2) = 0.0;
        }else{
            tmp_displacement = displacement;
        }
        return tmp_displacement;
    }
    void clean_displacement()
    {
        displacement(0) = 0.0;
        displacement(1) = 0.0;
        displacement(2) = 0.0;
        robot.last_odom = robot.odom;
    }
    LaserScan get_scan(){return scan;}
    Robot get_robot_odom(Robot robot_in)
    {
        robot_in.odom = robot.odom;
        robot_in.last_odom = robot.last_odom;
        return robot;
    }
private:
    ros::NodeHandle nh;
    ros::Subscriber scan_sub;
    ros::Subscriber odom_sub;
    ros::Subscriber init_pose_sub;

    Vector3d displacement;
    LaserScan scan;
    Robot robot;
    bool init_flag;
};

#endif