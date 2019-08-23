#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/PoseArray.h"
#include "nav_msgs/OccupancyGrid.h"
#include <tf/transform_listener.h>
#include "monte_carlo_localization/particle_filter.h"
#include <sstream>
#include <tf2/LinearMath/Quaternion.h>
#include <tf/transform_broadcaster.h>
#include <math.h>       /* hypot */

#define PARTICLES_NUMS 200


std::vector<Vector2d> laser_wall;
std::vector<double> laser_data;
double scan_range;
int samples;
sensor_msgs::LaserScan laser_msg;


nav_msgs::OccupancyGrid generate_OccGridMapMsg(double *likelihood_map, int width, int height)
{
  nav_msgs::OccupancyGrid likelihood_map_msg;
  likelihood_map_msg.header.frame_id = "map";
  likelihood_map_msg.info.resolution = 0.01;
  likelihood_map_msg.info.width = width;
  likelihood_map_msg.info.height = height;
  geometry_msgs::Pose origin;
  origin.position.x=-width/2*0.01;
  origin.position.y=-height/2*0.01;
  likelihood_map_msg.info.origin = origin;
  for(int i=0;i<height;i++)
  {
    for(int j=0;j<width;j++)
    {
      int index = i*width+j;
      double value = likelihood_map[index]*100;
      likelihood_map_msg.data.push_back(value);
    }
  }
  return likelihood_map_msg;
}

geometry_msgs::PoseArray generate_ParticlesMsg(std::vector<Vector3d> particles)
{
  geometry_msgs::PoseArray particles_msg;
  particles_msg.header.frame_id = "map";

  for(int i=0;i<PARTICLES_NUMS;i++)
  {
    geometry_msgs::Pose tmp_particle_pose;
    tmp_particle_pose.position.x = particles[i][0]/100;
    tmp_particle_pose.position.y = particles[i][1]/100;
    tf2::Quaternion myQuaternion;
    myQuaternion.setRPY( 0, 0, particles[i][2]);
    tmp_particle_pose.orientation.x = myQuaternion.getX();
    tmp_particle_pose.orientation.y = myQuaternion.getY();
    tmp_particle_pose.orientation.z = myQuaternion.getZ();
    tmp_particle_pose.orientation.w = myQuaternion.getW();
    particles_msg.poses.push_back(tmp_particle_pose);
  }
  return particles_msg;
}

geometry_msgs::Twist calculate_displacement(tf::StampedTransform current_odom, geometry_msgs::Twist last_odom)
{
  geometry_msgs::Twist displacement;
  // system("clear");
  // std::cout << "Current odom: " << current_odom.getOrigin().x() << ", " << current_odom.getOrigin().y() << std::endl;
  // std::cout << "Last odom: " << last_odom.linear.x << ", " << last_odom.linear.y << std::endl;
  // std::cout << "=============================================" << std::endl;
  double tmp_x = current_odom.getOrigin().x() - last_odom.linear.x;
  double tmp_y = current_odom.getOrigin().y() - last_odom.linear.y;
  double direction = atan2(tmp_y,tmp_x);
  double dist = hypot(tmp_x,tmp_y);
  double robot_yaw = tf::getYaw(current_odom.getRotation());
  if(abs(direction - robot_yaw)<1.57)
  {
    dist = dist*1;
  }else{
    dist = dist*-1;
  }
  displacement.linear.x = dist;
  displacement.linear.y = 0;
  displacement.angular.z = robot_yaw-last_odom.angular.z;
  // std::cout << "=============================================" << std::endl;
  // std::cout << "Yaw: " << displacement.angular.z << std::endl;
  return displacement;
}

sensor_msgs::LaserScan generate_scanMsg_for_debug(std::vector<Vector3d> particles,std::vector<Vector2i> lasers_map_on_particle)
{
  laser_msg.ranges.clear();
  laser_msg.header.stamp = ros::Time::now();
  laser_msg.range_max = 30.0;
  laser_msg.range_min = 0.1;
  laser_msg.angle_max = 2.35599994659;
  laser_msg.angle_min = -2.35599994659;
  laser_msg.angle_increment= (laser_msg.angle_max-laser_msg.angle_min)/lasers_map_on_particle.size();
  laser_msg.scan_time = 0;
  laser_msg.time_increment = 0;
  laser_msg.header.frame_id = "pf_outPutPose";
  // for(int j=0;j< particles.size();j++)
  for(int j=0;j< 1;j++)
  {
    for(int i=0;i < lasers_map_on_particle.size();i++)
    {
      double dist = hypot(lasers_map_on_particle[i][0]-particles[j][0],lasers_map_on_particle[i][1]-particles[j][1])/100;
      laser_msg.ranges.push_back(dist);
    }
  }
  
  return laser_msg;
}

void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan_in)
{
  laser_data.clear();
  scan_range = scan_in->angle_max - scan_in->angle_min;
  // std::cout << "Hokuyo Send: " << scan_in->ranges.size() << std::endl;
  int downsampling_param = 10;
  samples = scan_in->ranges.size()/downsampling_param;
  // std::cout << "After downsampling: " << samples << std::endl;

  for(int i=0;i<samples;i++)
  {
    laser_data.push_back(scan_in->ranges[downsampling_param*i]);
  }
}

void publish_pf_pose_tf(Vector3d predictionPose)
{
  static tf::TransformBroadcaster br;
  tf::Transform transform;
  transform.setOrigin( tf::Vector3(predictionPose(0), predictionPose(1), 0.0) );
  tf::Quaternion q;
  q.setRPY(0, 0, predictionPose(2));
  transform.setRotation(q);
  br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "map", "pf_outPutPose"));
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "my_mcl");
  ros::NodeHandle nh;
  ros::Subscriber scan_sub_ = nh.subscribe<sensor_msgs::LaserScan> ("scan", 1, scanCallback);
  ros::Publisher particles_pub_ = nh.advertise<geometry_msgs::PoseArray>("my_amcl/particles", 1);
  ros::Publisher likelihood_map_pub_ = nh.advertise<nav_msgs::OccupancyGrid>("likelihood_map", 1);
  ros::Publisher particle_scan_pub = nh.advertise<sensor_msgs::LaserScan> ("particle_scan", 1);
  tf::TransformListener listener;
  bool init_flag1 = true;
  bool init_flag2 = true;

  ParticleFilter pf(PARTICLES_NUMS);
  pf.build_likelihoodMap();
  pf.initParticle_Filter();
  double *likelihood_map = pf.get_Likelihood_map();
  int map_size[2];
  pf.get_likelihoodMapSize(&map_size[0]);
  std::cout << "map_w: " << map_size[0] << ", map_h: " << map_size[1] << std::endl;
  nav_msgs::OccupancyGrid likelihood_map_msg = generate_OccGridMapMsg(likelihood_map, map_size[0], map_size[1]);

  geometry_msgs::Twist tmp_displacement;
  geometry_msgs::Twist last_displacement;

  ros::Rate loop_rate(10);
  while (ros::ok())
  {
    std::vector<Vector3d> particles = pf.get_Particle();
    geometry_msgs::PoseArray particles_msg = generate_ParticlesMsg(particles);
    particles_pub_.publish(particles_msg);
    likelihood_map_pub_.publish(likelihood_map_msg);
    
    tf::StampedTransform transform;
    try{
      listener.lookupTransform("odom", "base_link", ros::Time(0), transform);
    }catch (tf::TransformException ex){
      ROS_ERROR("%s",ex.what());
      ros::Duration(1.0).sleep();
    }

    if(init_flag1)
    {
      if(init_flag2)
        init_flag1 = true;
      else
        init_flag1 = false;
      init_flag2 = false;
      last_displacement.linear.x = transform.getOrigin().x();
      last_displacement.linear.y = transform.getOrigin().y();
      last_displacement.angular.z = tf::getYaw(transform.getRotation());
    }

    tmp_displacement = calculate_displacement(transform, last_displacement);

    if(hypot(tmp_displacement.linear.x,tmp_displacement.linear.y)>0.05 || abs(tmp_displacement.angular.z) > 0.01)
    {
      pf.moveParticle(tmp_displacement);
      last_displacement.linear.x = transform.getOrigin().x();
      last_displacement.linear.y = transform.getOrigin().y();
      last_displacement.angular.z = tf::getYaw(transform.getRotation());
      // std::cout << "Move 5cm or Rotate 5 degree!!!" << std::endl;

      int counter = 0;
      // while(counter<1 || pf.ISConvergence())
      // {
        pf.rateGrade(scan_range, laser_data);
        /********* For Rate Particle Debug *********/
        // std::vector<Vector3d> particles = pf.get_Particle();
        std::vector<Vector2i> lasers_map_on_particle = pf.get_tpwall();
        sensor_msgs::LaserScan laser_msg;
        laser_msg = generate_scanMsg_for_debug(particles, lasers_map_on_particle);
        particle_scan_pub.publish(laser_msg);
        
        // std::vector<Vector3d> particles = pf.get_Particle();
        // std::cout << particles[0](0) << particles[0](1) << std::endl;
        // pf.Sim_LaserWall(particles[0]);
        // std::vector<Vector2i> lasers_sim_on_particle = pf.get_simSensorWall();
        // sensor_msgs::LaserScan sim_laser_msg;
        // sim_laser_msg = generate_scanMsg_for_debug(particles, lasers_sim_on_particle);
        // particle_scan_pub.publish(sim_laser_msg);
        /*******************************************/
        // pf.tournament_selection();
        pf.roulette_wheel_selection();
        /********* Publish Particles for Debug *********/
        particles.clear();
        particles = pf.get_Particle();
        geometry_msgs::PoseArray particles_msg = generate_ParticlesMsg(particles);
        particles_pub_.publish(particles_msg);
        /***********************************************/
        publish_pf_pose_tf(pf.get_Estimate_pose());
        // counter++;
      // }

    }

    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}
