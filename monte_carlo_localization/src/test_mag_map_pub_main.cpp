#include "monte_carlo_localization/mag_map_publisher.h"
#include "ros/ros.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "mag_map_publisher");
    mag_map_publisher mag_pub;
    mag_pub.init();
    ros::Rate loop_rate(10);
    while (ros::ok())
    {
        mag_pub.publish_mag_map();
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}