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
    nav_msgs::OccupancyGrid map_msg;

    // ROS
    ros::NodeHandle nh;
    ros::Publisher publisher;

public:
    LaserLikelihoodMapPublisher()
    {
        publisher = nh.advertise<nav_msgs::OccupancyGrid>("laser_likelihood_map", 1);
    }
    ~LaserLikelihoodMapPublisher(){}
    void publish_map()
    {
        publisher.publish(map_msg);
    }

    void generate_OccGridMapMsg(LaserLikelihoodMap map)
    {
        map_msg.header.frame_id = "map";
        map_msg.info.resolution = map.resolution;
        map_msg.info.width = map.width;
        map_msg.info.height = map.height;
        geometry_msgs::Pose tmp_origin;
        tmp_origin.position.x = map.origin[0];
        tmp_origin.position.y = map.origin[1];
        tmp_origin.position.z = map.origin[2];
        map_msg.info.origin = tmp_origin;
        
        for(int i=0;i<map.height;i++)
        {
            for(int j=0;j<map.width;j++)
            {
                if(map.data[i][j]==0)
                {
                    map_msg.data.push_back(-1);
                }else{
                    map_msg.data.push_back(map.data[i][j]*100);
                }
            }
        }
        cout << "ROS MAP Message Generate Done!" << endl;
    }
};
#endif