#include "monte_carlo_localization/mag_map_publisher.h"

mag_map_publisher::mag_map_publisher(/* args */)
{
    mag_map_x_publisher_ = nh_.advertise<nav_msgs::OccupancyGrid>("magnetic_likelihood_map_x", 1);
    mag_map_y_publisher_ = nh_.advertise<nav_msgs::OccupancyGrid>("magnetic_likelihood_map_y", 1);
    mag_map_z_publisher_ = nh_.advertise<nav_msgs::OccupancyGrid>("magnetic_likelihood_map_z", 1);
}

void mag_map_publisher::read_magnetic_map()
{
    csv_reader_.set_filename("/home/lui/catkin_ws/src/FY109-FLP/magnetic_map_data/predic/mag_pred_x.csv");
    mag_data_zx_.mag_map = csv_reader_.get_data();
    std::cout << "Magnetic Data Size Width: " << mag_data_zx.size() << " Height: " << mag_data_zx[0].size() << std::endl; 
    csv_reader_.set_filename("/home/lui/catkin_ws/src/FY109-FLP/magnetic_map_data/predic/mag_pred_y.csv");
    mag_data_zy_.mag_map = csv_reader_.get_data();
    csv_reader_.set_filename("/home/lui/catkin_ws/src/FY109-FLP/magnetic_map_data/predic/mag_pred_z.csv");
    mag_data_zz_.mag_map = csv_reader_.get_data();
}

nav_msgs::OccupancyGrid mag_map_publisher::generate_OccGridMapMsg(magnetic_map mag_map)
{
    nav_msgs::OccupancyGrid mag_map_msg;
    mag_map_msg.header.frame_id = "map";
    mag_map_msg.info.resolution = 0.5;
    mag_map_msg.info.width = mag_map.width;
    mag_map_msg.info.height = mag_map.height;
    geometry_msgs::Pose origin;
    origin.position.x=-mag_map.width/2*0.01;
    origin.position.y=-mag_map.height/2*0.01;
    mag_map_msg.info.origin = origin;
    for(int i=0;i<mag_map.height;i++)
    {
        for(int j=0;j<mag_map.width;j++)
        {
        }
    }
    return mag_map_msg;
}

void mag_map_publisher::publish_mag_map()
{
    mag_map_x_publisher_.publish(mag_map_x_msg_);
    mag_map_y_publisher_.publish(mag_map_y_msg_);
    mag_map_z_publisher_.publish(mag_map_z_msg_);
}