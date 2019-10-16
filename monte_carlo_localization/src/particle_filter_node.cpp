#include "ros/ros.h"
#include "monte_carlo_localization/robot.h"
#include "monte_carlo_localization/particles.h"
#include "monte_carlo_localization/particle_filter.h"
#include "monte_carlo_localization/particles_publisher.h"
#include "monte_carlo_localization/ROS_sensor_interface.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "my_mcl");
    Robot robot;
    Vector3d displacement;
    Particles particles;
    ParticleFilter pf;
    ParticlesPublisher pAry_pub;
    ROSSensorInterfaces ros_sensor;

    // Initial ROS Sensor Communications
    ros_sensor.init();
    // Initial Particles
    particles.init(200);
    // Input map to initial ParticleFilter
    if (argc < 2)
    {
        string default_map_path = "/home/lui/test.yaml";
        pf.init(default_map_path);
    }else{
        pf.init(argv[1]);
    }
    pAry_pub.init();
    
    ros::Rate loop_rate(40);
    while (ros::ok())
    {
        robot = ros_sensor.get_robot_odom(robot);
        displacement = ros_sensor.get_displacement();
        if(abs(displacement(0))>0.05 || abs(displacement(2))>0.01745)
        {
            particles.move_particles(displacement);
            ros_sensor.clean_displacement();
            if(pf.input_scan(ros_sensor.get_scan())&&pf.input_particles(particles))
            {
                if(pf.rate_particles())
                {
                    pf.roulette_wheel_selection();
                    particles = pf.get_particles();
                }
            }
        }
        pAry_pub.generate_particles_msgs(particles());
        pAry_pub.publish_msg();
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}
