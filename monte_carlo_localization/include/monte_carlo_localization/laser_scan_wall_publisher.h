#ifndef _LASER_SCAN_WALL_PUBLISHER_H_
#define _LASER_SCAN_WALL_PUBLISHER_H_

#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include <vector>
#include "eigen3/Eigen/Dense"
#include "laser_scan.h"
#include "particles.h"

using namespace std;
using namespace Eigen;

class LaserScanWallPublisher
{
public:
    LaserScanWallPublisher(){}
    ~LaserScanWallPublisher(){}
    void init()
    {
        publisher = nh.advertise<sensor_msgs::LaserScan>("scan_wall", 1);
    }
    void generate_scanMsg(LaserScan scan, Particles particles)
    {
        scan_msg.ranges.clear();
        scan_msg.header.stamp = ros::Time::now();
        scan_msg.range_max = 30.0;
        scan_msg.range_min = 0.1;
        scan_msg.angle_max = scan.ranges/2;
        scan_msg.angle_min = -scan.ranges/2;
        scan_msg.angle_increment= scan.ranges/scan.sample_num;
        scan_msg.scan_time = 0;
        scan_msg.time_increment = 0;
        scan_msg.header.frame_id = "particle_0";
        // for(int j=0;j< particles.size();j++)
        for(int j=0;j< 1;j++)
        {
            for(int i=0;i < scan.sample_num;i++)
            {
                double dist = hypot(scan.scan_wall[i][0]-particles.pAry[j].x, scan.scan_wall[i][1]-particles.pAry[j].y);
                scan_msg.ranges.push_back(dist);
            }
        }
    }
    void generate_scanMsg(vector<Vector2d> scan_wall, double ranges)
    {
        scan_msg.ranges.clear();
        scan_msg.header.stamp = ros::Time::now();
        scan_msg.range_max = 30.0;
        scan_msg.range_min = 0.1;
        scan_msg.angle_max = ranges/2;
        scan_msg.angle_min = -ranges/2;
        scan_msg.angle_increment= ranges/scan_wall.size();
        scan_msg.scan_time = 0;
        scan_msg.time_increment = 0;
        scan_msg.header.frame_id = "map";
        // for(int j=0;j< particles.size();j++)
        for(int j=0;j< 1;j++)
        {
            for(int i=0;i < scan_wall.size();i++)
            {
                double dist = hypot(scan_wall[i][0], scan_wall[i][1]);
                scan_msg.ranges.push_back(dist);
            }
        }
    }
    void publish_wall()
    {
        publisher.publish(scan_msg);
    }
    // ROS
    ros::NodeHandle nh;
    ros::Publisher publisher;
    sensor_msgs::LaserScan scan_msg;
};

#endif