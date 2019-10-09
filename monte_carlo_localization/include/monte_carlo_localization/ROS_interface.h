#ifndef _ROS_INTERFACE_H_
#define _ROS_INTERFACE_H_

#include <ros/ros.h>
#include "sensor_msgs/LaserScan.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"

#include "monte_carlo_localization/laser_scan.h"
#include "monte_carlo_localization/odometry.h"

using namespace std;

class ROSInterfaces
{
public:
    ROSInterfaces()
    {
        odom_sub = nh.subscribe<nav_msgs::Odometry>("odom", 1, &ROSInterfaces::odomCallback, this);
        scan_sub = nh.subscribe<sensor_msgs::LaserScan> ("scan", 1, &ROSInterfaces::scanCallback, this);
        init_pose_sub = nh.subscribe<geometry_msgs::PoseWithCovarianceStamped> (
                            "initialpose", 1, &ROSInterfaces::initPoseCallback, this);
    }
    ~ROSInterfaces(){}

    void initPoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &initpose_in)
    {

    }

    void odomCallback(const nav_msgs::Odometry::ConstPtr& odom_in)
    {
        odom.frame_id = odom_in->header.frame_id;
        odom.child_frame_id = odom_in->child_frame_id;
        odom.pose.x = odom_in->pose.pose.position.x;
    }

    void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan_in)
    {
        scan.data.clear();
        scan.ranges = scan_in->angle_max - scan_in->angle_min;
        // std::cout << "Hokuyo Send: " << scan_in->ranges.size() << std::endl;
        int downsampling_param = 10;
        scan.sample_num = scan_in->ranges.size()/downsampling_param;
        // std::cout << "After downsampling: " << samples << std::endl;

        for(int i=0; i<scan.sample_num; i++)
        {
            scan.data.push_back(scan_in->ranges[downsampling_param*i]);
        }
    }
    LaserScan get_scan(){return scan;}
    Odometry get_odom(){return odom;}
private:
    ros::NodeHandle nh;
    ros::Subscriber scan_sub;
    ros::Subscriber odom_sub;
    ros::Subscriber init_pose_sub;
    
    LaserScan scan;
    Odometry odom;
};

#endif