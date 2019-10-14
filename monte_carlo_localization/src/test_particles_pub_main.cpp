#include "ros/ros.h"
#include "monte_carlo_localization/particles.h"
#include "monte_carlo_localization/particles_publisher.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "particles_publisher");
    Particles particles(200);
    ParticlesPublisher pAry_pub;

    // particles.set_not_trust_imu();
    particles.set_trust_imu();
    pAry_pub.generate_particles_msgs(particles.get_particles());
    
    ros::Rate loop_rate(1);
    while (ros::ok())
    {
        pAry_pub.publish_msg();
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}