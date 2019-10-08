#ifndef _MAG_MAP_PUBLISHER_H_
#define _MAG_MAP_PUBLISHER_H_

#include <iostream>
#include "monte_carlo_localization/csv_reader.h"
#include "monte_carlo_localization/magnetic_map.h"
// ROS Stuff
#include "ros/ros.h"
#include "nav_msgs/OccupancyGrid.h"

using namespace std;

class mag_map_publisher
{
private:
    /* data */
    CSVReader csv_reader_;
    // Magnetic Map
    MagneticMap mag_data_zx_;
    MagneticMap mag_data_zy_;
    MagneticMap mag_data_zz_;
    nav_msgs::OccupancyGrid mag_map_x_msg_;
    nav_msgs::OccupancyGrid mag_map_y_msg_;
    nav_msgs::OccupancyGrid mag_map_z_msg_;

    // ROS
    ros::NodeHandle nh_;
    ros::Publisher mag_map_x_publisher_;
    ros::Publisher mag_map_y_publisher_;
    ros::Publisher mag_map_z_publisher_;

    // Function
public:
    mag_map_publisher(/* args */);
    ~mag_map_publisher(){}
    void read_magnetic_map();
    void publish_mag_map();
    nav_msgs::OccupancyGrid generate_OccGridMapMsg(MagneticMap mag_map);
};
#endif