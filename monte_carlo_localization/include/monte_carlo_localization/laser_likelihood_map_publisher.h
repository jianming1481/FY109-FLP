#ifndef _LASER_LIKELIHOOD_MAP_PUBLISHER_H_
#define _LASER_LIKELIHOOD_MAP_PUBLISHER_H_

#include <iostream>
#include "ros/ros.h"
#include "nav_msgs/OccupancyGrid.h"
#include "geometry_msgs/Pose.h"
#include "laser_likelihood_map.h"

using namespace std;

class LaserLikelihoodMapPublisher
{
private:
    nav_msgs::OccupancyGrid map_msg_;

    // ROS
    ros::NodeHandle nh_;
    ros::Publisher publisher_;

public:
    LaserLikelihoodMapPublisher()
    {
        publisher_ = nh_.advertise<nav_msgs::OccupancyGrid>("laser_likelihood_map", 1);
    }
    ~LaserLikelihoodMapPublisher(){}
    void publish_map()
    {
        publisher_.publish(map_msg_);
    }

    void generate_OccGridMapMsg(LaserLikelihoodMap map)
    {
        map_msg_.header.frame_id = "map";
        map_msg_.info.resolution = map.resolution_;
        map_msg_.info.width = map.width_;
        map_msg_.info.height = map.height_;
        geometry_msgs::Pose tmp_origin;
        tmp_origin.position.x = map.origin_[0];
        tmp_origin.position.y = map.origin_[1];
        tmp_origin.position.z = map.origin_[2];
        map_msg_.info.origin = tmp_origin;
        
        for(int i=0;i<map.height_;i++)
        {
            for(int j=0;j<map.width_;j++)
            {
                if(map.data_[i][j]==0)
                {
                    map_msg_.data.push_back(-1);
                }else{
                    map_msg_.data.push_back(map.data_[i][j]*100);
                }
            }
        }
        cout << "ROS MAP Message Generate Done!" << endl;
    }
};
#endif