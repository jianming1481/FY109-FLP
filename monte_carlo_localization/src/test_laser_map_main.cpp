#include "monte_carlo_localization/laser_map_publisher.h"
#include "monte_carlo_localization/laser_map_reader.h"
#include "monte_carlo_localization/laser_map.h"
#include "ros/ros.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "laser_map_publisher");
    LaserMapPublisher laser_map_publisher;
    LaserMapReader laser_map_reader;
    LaserMap laser_map;

    laser_map_reader.read_yaml(argv[1]);
    laser_map_publisher.generate_OccGridMapMsg(laser_map_reader.get_map());

    ros::Rate loop_rate(1);
    while (ros::ok())
    {
        laser_map_publisher.publish_map();
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}