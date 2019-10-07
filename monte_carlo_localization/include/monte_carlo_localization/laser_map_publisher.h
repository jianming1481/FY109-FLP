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
    nav_msgs::OccupancyGrid laser_map_msg_;

    // ROS
    ros::NodeHandle nh_;
    ros::Publisher laser_map_publisher_;

    // Function
public:
    LaserMapPublisher()
    {
        laser_map_publisher_ = nh_.advertise<nav_msgs::OccupancyGrid>("laser_map", 1);
    }
    ~LaserMapPublisher(){}
    void publish_laser_map()
    {
        laser_map_publisher_.publish(laser_map_msg_);
    }

    void find_unique_value_in_map(LaserMap map)
    {
        vector<double> pixel_value_vec;  // 0, 205, 254
        bool different = true;
        double pixel_value;
        for(int i=0;i<map.height_;i++)
        {
            for(int j=0;j<map.width_;j++)
            {
                pixel_value = map.data_[i][j];
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
        cout << "Pixel Value Vector: ";
        for(int i=0;i<pixel_value_vec.size();i++)
        {
            cout << pixel_value_vec[i] << ", ";
        }
        cout << endl;
    }

    void generate_OccGridMapMsg(LaserMap map)
    {
        laser_map_msg_.header.frame_id = "map";
        laser_map_msg_.info.resolution = map.resolution_;
        laser_map_msg_.info.width = map.width_;
        laser_map_msg_.info.height = map.height_;
        geometry_msgs::Pose tmp_origin;
        tmp_origin.position.x = map.origin_[0];
        tmp_origin.position.y = map.origin_[1];
        tmp_origin.position.z = map.origin_[2];
        laser_map_msg_.info.origin = tmp_origin;
    
        double max_value = 0; 
        double min_value = 99;
        double pixel_value;
        find_unique_value_in_map(map);
        bool different = true;
        for(int i=0;i<map.height_;i++)
        {
            for(int j=0;j<map.width_;j++)
            {
                pixel_value = map.data_[i][j];
                if(pixel_value ==0)
                {
                    laser_map_msg_.data.push_back(100);
                }else if(pixel_value <= 205 && pixel_value >0)
                {
                    laser_map_msg_.data.push_back(-1);
                }else{
                    laser_map_msg_.data.push_back(0);
                }

                // laser_map_msg_.data.push_back(map.data_[i][j]);
                if(max_value < map.data_[i][j]) max_value = map.data_[i][j];
                if(min_value > map.data_[i][j]) min_value = map.data_[i][j];
            }
        }
        cout << "Generate Map Message Done!" << endl;
        cout << "Max_value: " << max_value << endl;
        cout << "Min_valueL " << min_value << endl;
    }
};
#endif