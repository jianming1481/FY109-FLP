#include "ros/ros.h"
#include "monte_carlo_localization/robot.h"
#include "monte_carlo_localization/robot_pose_publisher.h"
#include "monte_carlo_localization/particles.h"
#include "monte_carlo_localization/particle_filter.h"
#include "monte_carlo_localization/particles_publisher.h"
#include "monte_carlo_localization/ROS_sensor_interface.h"
#include "monte_carlo_localization/laser_likelihood_map_publisher.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "my_mcl");
    Robot robot;
    RobotPosePublisher robot_pose_pub;
    Vector3d displacement;
    Particles particles;
    ParticleFilter pf;
    ParticlesPublisher pAry_pub;
    ROSSensorInterfaces ros_sensor;
    LaserLikelihoodMapPublisher likelihood_map_publisher;
    // Initial ROS Sensor Communications
    ros_sensor.init();

    // Initial Particles
    int pNum = 1000;
    pf.set_pNum(pNum);
    particles.init(pNum, -1.0, -1.5);
    
    // Input map to initial ParticleFilter
    if (argc < 2)
    {
        string default_map_path = "/home/lui/map_r1.yaml";
        pf.init(default_map_path);
    }else{
        pf.init(argv[1]);
    }
    particles.set_map_boundary(pf.x_max, pf.y_max, pf.x_min, pf.y_min);
    pAry_pub.init();
    robot_pose_pub.init();

    likelihood_map_publisher.generate_OccGridMapMsg(pf.get_likelihood_map());

    ros::Rate loop_rate(40);
    while (ros::ok())
    {
        likelihood_map_publisher.publish_map();
        robot = ros_sensor.get_robot_odom(robot);
        displacement = ros_sensor.get_displacement();
        if(abs(displacement(0))>0.05 || abs(displacement(2))>0.01745)
        {
            particles.move_particles(displacement);
            ros_sensor.clean_displacement();

            // if(pf.input_scan(ros_sensor.get_scan())&&pf.input_particles(particles))
            // {
            //     // int iterator = 0;
            //     // while(iterator<5)
            //     // {
            //         pf.input_mag_0(ros_sensor.get_mag_0());
            //         pf.input_mag_1(ros_sensor.get_mag_1());
            //         pf.input_mag0_index(ros_sensor.get_mag0_index());
            //         pf.input_mag1_index(ros_sensor.get_mag1_index());
            //         if(pf.rate_particles())
            //         {
            //             pf.roulette_wheel_selection();
            //             // pf.tournment_selection();
            //             particles = pf.get_particles();
            //             pAry_pub.generate_particles_msgs(particles());
            //             pAry_pub.publish_msg();
            //             robot_pose_pub.generate_robot_pose_msgs(pf.get_robot());
            //             robot_pose_pub.publish_msgs();
            //         }
            //         // iterator++;
            //     // }
            // }


            if(        pf.input_mag_0(ros_sensor.get_mag_0()) 
                    && pf.input_mag_1(ros_sensor.get_mag_1())
                    && pf.input_mag_2(ros_sensor.get_mag_2())
                    && pf.input_particles(particles)
            )
            {
                pf.input_mag0_index(ros_sensor.get_mag0_index());
                if(pf.mag_rate_particles())
                {
                    pf.roulette_wheel_selection();
                    particles = pf.get_particles();
                    pAry_pub.generate_particles_msgs(particles());
                    pAry_pub.publish_msg();
                    robot_pose_pub.generate_robot_pose_msgs(pf.get_robot());
                    robot_pose_pub.publish_msgs();
                }
            }
        }
        if(ros_sensor.pose_estimation_flag)
        {
            particles.init(pNum, ros_sensor.init_pose(0), ros_sensor.init_pose(1), ros_sensor.init_pose(2));
            ros_sensor.pose_estimation_flag = false;
            ros_sensor.init_flag = false;
        }
        pAry_pub.generate_particles_msgs(particles());
        pAry_pub.publish_msg();
        robot_pose_pub.generate_robot_pose_msgs(pf.get_robot());
        robot_pose_pub.publish_msgs();
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}
