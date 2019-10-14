#include "ros/ros.h"
#include "monte_carlo_localization/particles.h"
#include "monte_carlo_localization/particles_publisher.h"
#include "monte_carlo_localization/robot.h"
#include "monte_carlo_localization/ROS_sensor_interface.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "particles_publisher");
    ROSSensorInterfaces ros_sensor;
    Particles particles(200);         // Input parameter is the amount of particles
    ParticlesPublisher pAry_pub;
    Robot robot;

    // particles.set_not_trust_imu();
    particles.set_trust_imu();
    ros::Rate loop_rate(100);
    while (ros::ok())
    {
        particles.move_particle(ros_sensor.get_displacement());
        pAry_pub.generate_particles_msgs(particles.get_particles());
        pAry_pub.publish_msg();
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}