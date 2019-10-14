#ifndef _PARTICLES_PUBLISHER_H_
#define _PARTICLES_PUBLISHER_H_

#include "ros/ros.h"
#include "geometry_msgs/PoseArray.h"
#include "tf/transform_datatypes.h"
#include <tf2/LinearMath/Quaternion.h>

#include "particles.h"

class ParticlesPublisher
{
public:
    ParticlesPublisher()
    {
        particles_publisher = nh.advertise<geometry_msgs::PoseArray>("particles",1);
    }
    ~ParticlesPublisher(){}
    void generate_particles_msgs(vector<Pose> pAry)
    {
        particles_msgs.poses.clear();
        particles_msgs.header.frame_id = "map";
        int p_num = pAry.size();
        for(int i=0;i<p_num;i++)
        {
            geometry_msgs::Pose tmp_particle_pose;
            tmp_particle_pose.position.x = pAry[i].x;
            tmp_particle_pose.position.y = pAry[i].y;
            tf2::Quaternion myQuaternion;
            myQuaternion.setRPY( 0, 0, pAry[i].yaw );
            tmp_particle_pose.orientation.x = myQuaternion.getX();
            tmp_particle_pose.orientation.y = myQuaternion.getY();
            tmp_particle_pose.orientation.z = myQuaternion.getZ();
            tmp_particle_pose.orientation.w = myQuaternion.getW();
            particles_msgs.poses.push_back(tmp_particle_pose);
        }
        // cout << "Particles numbers: " << particles_msgs.poses.size() << endl;

    }
    void publish_msg(){particles_publisher.publish(particles_msgs);}
private:
    ros::NodeHandle nh;
    ros::Publisher particles_publisher;
    geometry_msgs::PoseArray particles_msgs;
};

#endif