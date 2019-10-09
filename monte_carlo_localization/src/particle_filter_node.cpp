#include "ros/ros.h"
#include "monte_carlo_localization/laser_scan.h"
#include "monte_carlo_localization/ROS_interface.h"
#include "monte_carlo_localization/particle_filter.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "my_mcl");
    ParticleFilter pf;

    ros::Rate loop_rate(10);
    while (ros::ok())
    {
        pf.rate_particles();
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}
