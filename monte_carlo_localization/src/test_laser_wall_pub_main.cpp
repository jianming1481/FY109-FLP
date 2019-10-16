#include "ros/ros.h"
#include <tf/transform_broadcaster.h>

#include "monte_carlo_localization/particles.h"
#include "monte_carlo_localization/particle_filter.h"
#include "monte_carlo_localization/particles_publisher.h"
#include "monte_carlo_localization/ROS_sensor_interface.h"
#include "monte_carlo_localization/laser_scan.h"
#include "monte_carlo_localization/laser_scan_wall_publisher.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "laser_likelihood_map_publisher");
    static tf::TransformBroadcaster br;
    tf::Transform transform;
    tf::Quaternion q;

    ParticleFilter pf;
    ROSSensorInterfaces ros_sensor;
    LaserScanWallPublisher scan_pub;
    ParticlesPublisher pAry_pub;

    pf.init("/home/lui/map.yaml", 1);
    ros_sensor.init();
    scan_pub.init();
    pAry_pub.init();

    ros::Rate loop_rate(100);
    while (ros::ok())
    {
        // broadcast tf to connect map and particle_0
        transform.setOrigin( tf::Vector3(pf.get_particles().pAry[0].x, 
                                         pf.get_particles().pAry[0].y, 
                                         0.0) );
        q.setRPY(0, 0, pf.get_particles().pAry[0].yaw);
        transform.setRotation(q);
        br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "map", "particle_0"));
        
        // publish scan wall after transfer to map
        if(pf.input_scan(ros_sensor.get_scan()))
        {
            pf.scan2wall_onMap(pf.get_particles().pAry[0].x,pf.get_particles().pAry[0].y,pf.get_particles().pAry[0].yaw);
            // scan_pub.generate_scanMsg(pf.get_scan(), pf.get_particles());            // Generate scan wall on particle_0
            scan_pub.generate_scanMsg(pf.get_scan_wall_on_map(), pf.get_scan_ranges());                              // Generate scan wall mapping on map

            scan_pub.publish_wall();
        }else{
            cout << "Scan Error!!!" << endl;
        }

        // Publish particles
        pAry_pub.generate_particles_msgs(pf.get_particles());
        pAry_pub.publish_msg();

        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}