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
public:
    mag_map_publisher()
    {
        mag_map_x_publisher_ = nh_.advertise<nav_msgs::OccupancyGrid>("magnetic_likelihood_map_x", 1);
        mag_map_y_publisher_ = nh_.advertise<nav_msgs::OccupancyGrid>("magnetic_likelihood_map_y", 1);
        mag_map_z_publisher_ = nh_.advertise<nav_msgs::OccupancyGrid>("magnetic_likelihood_map_z", 1);
        read_magnetic_map();
        mag_map_x_msg_ = generate_OccGridMapMsg(mag_data_zx_);
        mag_map_y_msg_ = generate_OccGridMapMsg(mag_data_zy_);
        mag_map_z_msg_ = generate_OccGridMapMsg(mag_data_zz_);
    }
    ~mag_map_publisher(){}

    void read_magnetic_map()
    {
        csv_reader_.set_filename("/home/lui/catkin_ws/src/FY109-FLP/magnetic_map_data/ROS_msg/mag_pred_x.csv");
        mag_data_zx_ = csv_reader_.get_data();
        
        csv_reader_.set_filename("/home/lui/catkin_ws/src/FY109-FLP/magnetic_map_data/ROS_msg/mag_pred_y.csv");
        mag_data_zy_ = csv_reader_.get_data();

        csv_reader_.set_filename("/home/lui/catkin_ws/src/FY109-FLP/magnetic_map_data/ROS_msg/mag_pred_z.csv");
        mag_data_zz_ = csv_reader_.get_data();

        // std::cout << "Magnetic Data Size Width: " << mag_data_zx_.data.size() << " Height: " << mag_data_zx_.data[0].size() << std::endl; 
        cout << "Load map done!" << endl;
    }

    nav_msgs::OccupancyGrid generate_OccGridMapMsg(MagneticMap mag_map)
    {
        nav_msgs::OccupancyGrid mag_map_msg;
        mag_map_msg.header.frame_id = "map";
        mag_map_msg.info.resolution = 0.5;
        mag_map_msg.info.width = mag_map.width_;
        mag_map_msg.info.height = mag_map.height_;
        geometry_msgs::Pose origin;
        origin.position.x=-35.657444;
        origin.position.y=-20.981978;
        mag_map_msg.info.origin = origin;
        for(int i=0;i<mag_map.height_;i++)
        {
            for(int j=0;j<mag_map.width_;j++)
            {
                // mag_map_msg.data.push_back(mag_map.data[i][j]);
            }
        }
        cout << "Generate Map Message Done!" << endl;
        return mag_map_msg;
    }

    void publish_mag_map()
    {

        mag_map_x_publisher_.publish(mag_map_x_msg_);
        mag_map_y_publisher_.publish(mag_map_y_msg_);
        mag_map_z_publisher_.publish(mag_map_z_msg_);
    }
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
};
#endif