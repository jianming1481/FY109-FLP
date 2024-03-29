#ifndef _LASER_MAP_PUBLISHER_H_
#define _LASER_MAP_PUBLISHER_H_

#include <iostream>
#include "ros/ros.h"
#include "nav_msgs/OccupancyGrid.h"
#include "geometry_msgs/Pose.h"
#include "laser_map.h"

using namespace std;

class LaserMapPublisher
{
private:
    nav_msgs::OccupancyGrid laser_map_msg;

    // ROS
    ros::NodeHandle nh;
    ros::Publisher laser_map_publisher;

    // Function
public:
    LaserMapPublisher()
    {
        laser_map_publisher = nh.advertise<nav_msgs::OccupancyGrid>("laser_map", 1);
    }
    ~LaserMapPublisher(){}
    void publish_map()
    {
        laser_map_publisher.publish(laser_map_msg);
    }

    void find_unique_value_in_map(LaserMap map)
    {
        // 0:   obstacles
        // 205: unknown
        // 254: no obstacle
        vector<double> pixel_value_vec;  
        bool different = true;
        double pixel_value;
        for(int i=0;i<map.height;i++)
        {
            for(int j=0;j<map.width;j++)
            {
                pixel_value = map.data[i][j];
                for (int i=0;i<pixel_value_vec.size();i++)
                {
                    if (pixel_value == pixel_value_vec[i])
                    {
                        different = false;
                        break;
                    }else{
                        different = true;
                    }
                }
                if(different)
                {
                    pixel_value_vec.push_back(pixel_value);
                    different = false;
                }
            }
        }
        // cout << "Pixel Value Vector: ";
        // for(int i=0;i<pixel_value_vec.size();i++)
        // {
        //     cout << pixel_value_vec[i] << ", ";
        // }
        // cout << endl;
    }

    void generate_OccGridMapMsg(LaserMap map)
    {
        laser_map_msg.header.frame_id = "map";
        laser_map_msg.info.resolution = map.resolution;
        laser_map_msg.info.width = map.width;
        laser_map_msg.info.height = map.height;
        geometry_msgs::Pose tmp_origin;
        tmp_origin.position.x = map.origin[0];
        tmp_origin.position.y = map.origin[1];
        tmp_origin.position.z = map.origin[2];
        laser_map_msg.info.origin = tmp_origin;
    
        double max_value = 0; 
        double min_value = 99;
        double pixel_value;
        find_unique_value_in_map(map);
        bool different = true;
        for(int i=0;i<map.height;i++)
        {
            for(int j=0;j<map.width;j++)
            {
                pixel_value = map.data[i][j];
                if(pixel_value ==0)
                {
                    laser_map_msg.data.push_back(100);
                }else if(pixel_value <= 205 && pixel_value >0)
                {
                    laser_map_msg.data.push_back(-1);
                }else{
                    laser_map_msg.data.push_back(0);
                }

                // laser_map_msg_.data.push_back(map.data_[i][j]);
                if(max_value < map.data[i][j]) max_value = map.data[i][j];
                if(min_value > map.data[i][j]) min_value = map.data[i][j];
            }
        }
        cout << "ROS MAP Message Generate Done!" << endl;
        // cout << "Max_value: " << max_value << endl;
        // cout << "Min_valueL " << min_value << endl;
    }
};
#endif