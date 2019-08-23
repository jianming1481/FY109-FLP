#ifndef LINE_FOLLOWER_H_
#define LINE_FOLLOWER_H_
#include <tf/tf.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <string>

#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <base_local_planner/trajectory_planner_ros.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <nav_core/base_local_planner.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

namespace line_follower {
  class LineFollower : public nav_core::BaseLocalPlanner {
    public:
      LineFollower(){};
      // LineFollower(std::string name){};
      ~LineFollower(){}
      void initialize(std::string name, tf2_ros::Buffer* tf, 
                      costmap_2d::Costmap2DROS* costmap_ros);
      bool isGoalReached();
      bool setPlan(const std::vector<geometry_msgs::PoseStamped>& global_plan);
      bool computeVelocityCommands(geometry_msgs::Twist& cmd_vel);

    private:
      inline double sign(double n){
        return n < 0.0 ? -1.0 : 1.0;
      }
      inline void set_vec3(tf::Vector3& vec, double x, double y, double z){
        vec.setX(x);
        vec.setY(y);
        vec.setZ(z);
      }

      double cal_dis(double x1, double x2, double x3, double x4);
      bool pure_pursuit_controller(geometry_msgs::Twist& cmd_vel);
      bool checkReachedCriteria(geometry_msgs::Twist& cmd_vel);
      bool transformGlobalPlan(
          const tf2_ros::Buffer& tf_buffer, 
          const std::vector<geometry_msgs::PoseStamped>& global_plan, 
          const costmap_2d::Costmap2DROS& costmap,
          const std::string& global_frame,
          std::vector<geometry_msgs::PoseStamped>& transformed_plan);

      //
      std::vector<double> path_line_;
      bool start_;
      bool reached_;
      bool rotate_to_goal_;
      std::string robot_name_prefix;
      std::string base_tf_name;

      //tf::TransformListener* tf_;
      tf2_ros::Buffer* tf_buffer_;
      tf2_ros::TransformListener* tf_listener_;
      costmap_2d::Costmap2DROS* costmap_ros_;
      std::vector<geometry_msgs::PoseStamped> global_plan_;
      geometry_msgs::PoseStamped goal_pt_;
      base_local_planner::TrajectoryPlannerROS collision_planner_;
  };
};
#endif
