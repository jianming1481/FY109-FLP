#include "ros/ros.h"
#include "monte_carlo_localization/robot.h"
#include "monte_carlo_localization/particle_filter.h"
#include "monte_carlo_localization/ROS_sensor_interface.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "my_mcl");
    Robot robot;
    ParticleFilter pf;
    ROSSensorInterfaces ros_sensor;

    ros_sensor.init();
    if (argc < 2)
    {
        string default_map_path = "/home/lui/test.yaml";
        pf.init(default_map_path);
    }else{
        pf.init(argv[1], 500);
    }
    ros::Rate loop_rate(10);
    while (ros::ok())
    {
        robot = ros_sensor.get_robot_odom(robot);
        if(pf.rate_particles())
        {
            pf.roulette_wheel_selection();
        }
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}
