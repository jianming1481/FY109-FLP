#ifndef _ROS_SENSOR_INTERFACE_H_
#define _ROS_SENSOR_INTERFACE_H_

#include <ros/ros.h>
#include "sensor_msgs/LaserScan.h"
#include "sensor_msgs/MagneticField.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "geometry_msgs/Twist.h"
#include "tf/transform_datatypes.h"

#include "monte_carlo_localization/laser_scan.h"
#include "monte_carlo_localization/robot.h"
#include "monte_carlo_localization/magnetic_field.h"

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
        mag_index_sub = nh.subscribe<geometry_msgs::Twist> ("mag_index", 1, &ROSSensorInterfaces::indexCallback, this);
        mag_0_sub = nh.subscribe<sensor_msgs::MagneticField> ("m_mag_0", 1, &ROSSensorInterfaces::mag0Callback, this);
        mag_1_sub = nh.subscribe<sensor_msgs::MagneticField> ("m_mag_1", 1, &ROSSensorInterfaces::mag1Callback, this);
        mag_2_sub = nh.subscribe<sensor_msgs::MagneticField> ("m_mag_2", 1, &ROSSensorInterfaces::mag2Callback, this);
        init_flag = true;
        pose_estimation_flag = false;
        init_pose_sub = nh.subscribe<geometry_msgs::PoseWithCovarianceStamped> (
                            "initialpose", 1, &ROSSensorInterfaces::initPoseCallback, this);
    }
    void initPoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &initpose_in)
    {
        cout << "Received initial signal!" << endl;
        pose_estimation_flag = true;
        init_flag = true;
        init_pose(0) = initpose_in->pose.pose.position.x;
        init_pose(1) = initpose_in->pose.pose.position.y;
        tf::Quaternion quat;
        tf::quaternionMsgToTF(initpose_in->pose.pose.orientation , quat);
        double roll, pitch, yaw;
        tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
        init_pose(2) = yaw;
    }

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
        if(init_flag)
        {
            robot.last_odom.pose = robot.odom.pose;
            init_flag = false;
        }
        calculate_displacement();
    }

    void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan_in)
    {
        scan.data.clear();
        scan.ranges = scan_in->angle_max - scan_in->angle_min;
        // std::cout << "Hokuyo Send: " << scan_in->ranges.size() << std::endl;
        int downsampling_param = 144;
        scan.sample_num = (int)(scan_in->ranges.size()/downsampling_param);
        // std::cout << "After downsampling: " << samples << std::endl;

        for(int i=0; i<scan.sample_num; i++)
        {
            scan.data.push_back(scan_in->ranges[downsampling_param*i]);
        }
    }
    void indexCallback(const geometry_msgs::Twist::ConstPtr& index_in)
    {
        mag_index.linear.x = index_in->linear.x;
        mag_index.linear.y = index_in->linear.y;
    }

    void mag0Callback(const sensor_msgs::MagneticField::ConstPtr& mag_in)
    {
        mag_0.x = mag_in->magnetic_field.x;
        mag_0.y = mag_in->magnetic_field.y;
        mag_0.z = mag_in->magnetic_field.z;
    }
    void mag1Callback(const sensor_msgs::MagneticField::ConstPtr& mag_in)
    {
        mag_1.x = mag_in->magnetic_field.x;
        mag_1.y = mag_in->magnetic_field.y;
        mag_1.z = mag_in->magnetic_field.z;
    }
    void mag2Callback(const sensor_msgs::MagneticField::ConstPtr& mag_in)
    {
        mag_2.x = mag_in->magnetic_field.x;
        mag_2.y = mag_in->magnetic_field.y;
        mag_2.z = mag_in->magnetic_field.z;
    }

    void calculate_displacement()
    {
        double tmp_x = robot.odom.pose.x- robot.last_odom.pose.x;
        double tmp_y = robot.odom.pose.y- robot.last_odom.pose.y;
        double tmp_yaw = robot.odom.pose.yaw - robot.last_odom.pose.yaw;

        double direction = atan2(tmp_y,tmp_x);
        // cout << "direction: " << direction << endl;
        // cout << "robot yaw: " <<robot.odom.pose.yaw << endl;
        // cout << direction-robot.odom.pose.yaw << endl;
        double dist = hypot(tmp_x,tmp_y);
        if(abs(direction - robot.odom.pose.yaw)<1.57)
        {
            dist = dist*1;
        }else{
            dist = dist*-1;
        }
 
        displacement(0) = dist;
        displacement(1) = 0;
        displacement(2) = tmp_yaw;
        // cout <<"Displacement: " << displacement <<endl;
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
    geometry_msgs::Twist get_index(){return mag_index;}
    Robot get_robot_odom(Robot robot_in)
    {
        robot_in.odom = robot.odom;
        robot_in.last_odom = robot.last_odom;
        return robot;
    }
    MagneticField get_mag_0(){return mag_0;}
    MagneticField get_mag_1(){return mag_1;}
    MagneticField get_mag_2(){return mag_2;}
    bool init_flag;
    bool pose_estimation_flag;
    Vector3d init_pose;
private:
    ros::NodeHandle nh;
    ros::Subscriber scan_sub;
    ros::Subscriber odom_sub;
    ros::Subscriber mag_index_sub;
    ros::Subscriber mag_0_sub;
    ros::Subscriber mag_1_sub;
    ros::Subscriber mag_2_sub;
    ros::Subscriber init_pose_sub;

    Vector3d displacement;
    LaserScan scan;
    geometry_msgs::Twist mag_index;
    MagneticField mag_0;
    MagneticField mag_1;
    MagneticField mag_2;
    Robot robot;
};

#endif