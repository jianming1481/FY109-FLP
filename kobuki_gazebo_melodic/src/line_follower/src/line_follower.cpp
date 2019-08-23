#include <line_follower/line_follower.h>
#include <pluginlib/class_list_macros.h>
#include <cmath>

//PLUGINLIB_DECLARE_CLASS(pose_follower, PoseFollower, pose_follower::PoseFollower, nav_core::BaseLocalPlanner)
PLUGINLIB_EXPORT_CLASS(line_follower::LineFollower, nav_core::BaseLocalPlanner)

namespace line_follower {
  
  bool LineFollower::setPlan(const std::vector<geometry_msgs::PoseStamped>& global_plan)
  {
    reached_ = false;
    //start_ = true;
    path_line_.clear();
    std::cout<<"line follower setPlan"<<std::endl;
    goal_pt_ = global_plan.back();
    if(!transformGlobalPlan(*tf_buffer_, 
                            global_plan,
                            *costmap_ros_, 
                            costmap_ros_->getGlobalFrameID(),
                            global_plan_))
    {
      ROS_ERROR("Could not transform the global plan to the frame of the controller");
      return false;
    }
    geometry_msgs::PoseStamped start_pt = global_plan_.front();
    geometry_msgs::PoseStamped end_pt = global_plan_.back();

    //cal the line coef of form y = a*x + b
    double slope = (end_pt.pose.position.y-start_pt.pose.position.y)/\
                   (end_pt.pose.position.x-start_pt.pose.position.x);
    path_line_.push_back(slope);
    path_line_.push_back(0.5*(end_pt.pose.position.y-slope*end_pt.pose.position.x)+\
                         0.5*(start_pt.pose.position.y-slope*start_pt.pose.position.x));
    std::cout<<"line: "<<slope<<", "<<path_line_.back()<<std::endl;
    return true;
  }
  
  double LineFollower::cal_dis(double x1, double y1, double x2, double y2)
  {
    return sqrt((x1-x2)*(x1-x2) + (y1-y2)*(y1-y2));
  }

  bool LineFollower::pure_pursuit_controller(geometry_msgs::Twist& cmd_vel)
  {
    double look_ahead_dis = 1;
    
    //1.Determine the current location of the vehicle
    //tf::Stamped<tf::Pose> robot_pose;
    geometry_msgs::PoseStamped robot_pose;
    costmap_ros_->getRobotPose(robot_pose);
    
    //2.Find the path point closest to the vehicle
    //3.Find the goal point
    double rbt_x = robot_pose.pose.position.x;
    double rbt_y = robot_pose.pose.position.y;
    std::cout<<"rbt_pos: ("<<rbt_x<<", "<<rbt_y<<")"<<std::endl;
    double ln_coef_x = path_line_.front();
    double ln_intrcpt = path_line_.back();
    
    double coef_a = ln_coef_x*ln_coef_x+1;
    double coef_b = 2*(ln_coef_x*ln_intrcpt - rbt_x - rbt_y*ln_coef_x);
    double coef_c = ln_intrcpt*ln_intrcpt + rbt_x*rbt_x + rbt_y*rbt_y - \
                    look_ahead_dis*look_ahead_dis -2*rbt_y*ln_intrcpt;
  
    double sol_x1 = (-coef_b+sqrt(coef_b*coef_b-4*coef_a*coef_c))/(2*coef_a);  
    double sol_x2 = (-coef_b-sqrt(coef_b*coef_b-4*coef_a*coef_c))/(2*coef_a); 
   
    std::cout<<"solx1: "<<sol_x1<<std::endl;
    std::cout<<"solx2: "<<sol_x2<<std::endl;
    double goal_x = global_plan_.back().pose.position.x;
    double goal_y = global_plan_.back().pose.position.y;
    double dis1 = cal_dis(sol_x1, ln_coef_x*sol_x1+ln_intrcpt, goal_x, goal_y);
    double dis2 = cal_dis(sol_x2, ln_coef_x*sol_x2+ln_intrcpt, goal_x, goal_y);
    geometry_msgs::PoseStamped goal_pt;
    if(dis1>=dis2) goal_pt.pose.position.x = sol_x2;
    else goal_pt.pose.position.x = sol_x1;

    if(goal_pt.pose.position.x>=global_plan_.back().pose.position.x)
      goal_pt.pose.position.x = global_plan_.back().pose.position.x;
    goal_pt.pose.position.y = goal_pt.pose.position.x*ln_coef_x + ln_intrcpt;
      
    //4.Transform the goal point to vehicle coordinates
    const geometry_msgs::PoseStamped& plan_pose = global_plan_[0];
    geometry_msgs::TransformStamped transformStamped;
    transformStamped = tf_buffer_->lookupTransform(
      base_tf_name, 
      ros::Time(0), 
      plan_pose.header.frame_id, 
      plan_pose.header.stamp, 
      plan_pose.header.frame_id,
      ros::Duration(0.5)
    );

    const geometry_msgs::PoseStamped& pose = goal_pt;
    //std::cout<<"origin goal_pt: ("<<goal_pt.pose.position.x<<", "<<
    //  goal_pt.pose.position.y<<")"<<std::endl;
    geometry_msgs::PoseStamped tf_pose;
    tf2::doTransform(goal_pt, tf_pose, transformStamped);

    /*
    std::cout<<"tf_pose: "<<"("<<tf_pose.getOrigin().x()<<", "<<
      tf_pose.getOrigin().y()<<")"<<std::endl;
    
    tf::Stamped<tf::Pose> rbt_poses;
    rbt_poses.setData(transform * robot_pose);
    std::cout<<"robot_pose: ("<<rbt_poses.getOrigin().x()<<", "<<
      rbt_poses.getOrigin().y()<<")"<<std::endl;
    */
    //5.Calculate the curvature
    //6.Update the robot's position
    double dis = cal_dis(robot_pose.pose.position.x, robot_pose.pose.position.y,
               global_plan_.back().pose.position.x,global_plan_.back().pose.position.y);
    // double dtheta = 
    std::cout<<"dis (rbt to goal): "<<dis<<std::endl;
    cmd_vel.linear.x = dis*0.6;
    if(cmd_vel.linear.x>0.3) cmd_vel.linear.x = 0.3;
    if(cmd_vel.linear.x<0.05) cmd_vel.linear.x = 0.05;
    cmd_vel.angular.z = cmd_vel.linear.x*2*tf_pose.pose.position.y/\
                        (look_ahead_dis*look_ahead_dis);
    if(cmd_vel.angular.z > 0.7) cmd_vel.angular.z=0.7;
    if(cmd_vel.angular.z < -0.7) cmd_vel.angular.z=-0.7;
    // double dtheta = (global_plan_.back().pose.position.y - robot_pose.pose.position.y)/\
    //                   (global_plan_.back().pose.position.x - robot_pose.pose.position.x);
    //cmd_vel.angular.z = dtheta*0.5;
    return true;
  }
  
  bool LineFollower::computeVelocityCommands(geometry_msgs::Twist& cmd_vel)
  {
    //get the current pose of the robot in the fixed frame
    std::cout<<"pose follower computeVelocityCommands"<<std::endl;
    geometry_msgs::PoseStamped robot_pose;
    //tf::Stamped<tf::Pose> robot_pose;
    if(!costmap_ros_->getRobotPose(robot_pose)){
      ROS_ERROR("Can't get robot pose");
      geometry_msgs::Twist empty_twist;
      cmd_vel = empty_twist;
      return false;
    }else{
      std::cout<<"PASS!"<<std::endl;
    }
    
    const geometry_msgs::PoseStamped& plan_pose = global_plan_[0];
    geometry_msgs::TransformStamped transformStamped;
        transformStamped = tf_buffer_->lookupTransform(
          base_tf_name, 
          ros::Time(0), 
          plan_pose.header.frame_id, 
          plan_pose.header.stamp, 
          plan_pose.header.frame_id,
          ros::Duration(0.5)
        );

    geometry_msgs::PoseStamped tf_pose1, tf_pose2;
    tf2::doTransform(global_plan_.front(), tf_pose1, transformStamped);
    tf2::doTransform(global_plan_.back(), tf_pose2, transformStamped);
    
    tf::Vector3 path_head, path_end, path_vec, rbt_vec;
    set_vec3(rbt_vec, 1, 0, 0);
    set_vec3(path_head, tf_pose1.pose.position.x, tf_pose1.pose.position.y, 0);
    set_vec3(path_end, tf_pose2.pose.position.x, tf_pose2.pose.position.y, 0);
    set_vec3(path_vec, path_end.x()-path_head.x(), path_end.y()-path_head.y(), 0);
    
    double dtheta = atan2((rbt_vec.cross(path_vec)).z(), rbt_vec.dot(path_vec));
    std::cout<<"dtheta at the beginning: "<<dtheta<<std::endl;
    
    ///////////////////////////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////////
    
    if(fabs(dtheta) >= 0.04 && start_){
      std::cout<<"in start rotation"<<std::endl;
      cmd_vel.linear.x = 0;
      if(fabs(dtheta)>0.4) cmd_vel.angular.z = sign(dtheta)*0.4;
      else if(fabs(dtheta)<0.15) cmd_vel.angular.z = sign(dtheta)*0.15;
      else cmd_vel.angular.z = dtheta;
      //return true;
    }
    else{
      start_ = false;
      pure_pursuit_controller(cmd_vel);
      reached_ = checkReachedCriteria(cmd_vel);
      std::cout<<"cmd_vel: "<<"("<<cmd_vel.linear.x<<", "<<cmd_vel.linear.y<<", "<<
        cmd_vel.angular.z<<")\n"<<std::endl;
      //return true;
    } 
    bool legal_traj = collision_planner_.checkTrajectory(cmd_vel.linear.x, 
                              cmd_vel.linear.y, cmd_vel.angular.z, true);
    if(!legal_traj){
      cmd_vel.linear.x = 0;
      cmd_vel.angular.z = 0;
    }
    return true;
  }
 
  bool LineFollower::isGoalReached(){
    if(reached_){
      std::cout<<"reached goal!!!"<<std::endl;
      reached_ = false;
      start_ = true;
      rotate_to_goal_ = false;
      return true;
    }
    return reached_;
  }

  bool LineFollower::checkReachedCriteria(geometry_msgs::Twist& cmd_vel){
    reached_ = false;
    //tf::Stamped<tf::Pose> robot_pose;
    geometry_msgs::PoseStamped robot_pose;
    costmap_ros_->getRobotPose(robot_pose);

    double dis = cal_dis(robot_pose.pose.position.x, robot_pose.pose.position.y,
               global_plan_.back().pose.position.x,global_plan_.back().pose.position.y);
    
    //view the goal point in base_link frame
    const geometry_msgs::PoseStamped& plan_pose = global_plan_.back();
    geometry_msgs::TransformStamped transformStamped;
    transformStamped = tf_buffer_->lookupTransform(
          base_tf_name, 
          ros::Time(0), 
          plan_pose.header.frame_id, 
          plan_pose.header.stamp, 
          plan_pose.header.frame_id,
          ros::Duration(0.5)
        );
    geometry_msgs::PoseStamped tf_pose;
    tf2::doTransform(plan_pose, tf_pose, transformStamped);
    
    // if(dis < 0.03 || tf_pose.getOrigin().x()<=0 || rotate_to_goal_){
    // Release threshold
    if(dis < 0.05){
      //check robot's direction
      if(!start_){
        rotate_to_goal_ = true;
        double dtheta, _goal_pitch, _goal_roll;
        tf2::Matrix3x3(tf2::Quaternion(tf_pose.pose.orientation.x, tf_pose.pose.orientation.y,
                                       tf_pose.pose.orientation.z, tf_pose.pose.orientation.w)).getEulerYPR(dtheta,
                                     _goal_pitch, _goal_roll);

        if(fabs(dtheta)<=0.0175){
          cmd_vel.linear.x = 0;
          cmd_vel.angular.z = 0;
          std::cout<<"goal reach criteria: "<<std::endl;
          std::cout<<"dis? --> "<<dis<<std::endl;
          std::cout<<"dtheta? --> "<<dtheta<<std::endl;
          std::cout<<"reache goal!!!"<<std::endl;
          return true;
        }
        else{
          cmd_vel.linear.x = 0;
          cmd_vel.angular.z = dtheta*0.5;
          if(fabs(dtheta)>0.4) cmd_vel.angular.z = sign(dtheta)*0.4;
          else if(fabs(dtheta)<0.05) cmd_vel.angular.z = sign(dtheta)*0.05;
        }
      }
    }
    return false;
  }
  
  bool LineFollower::transformGlobalPlan(
                        const tf2_ros::Buffer& tf_buffer,
                        const std::vector<geometry_msgs::PoseStamped>& global_plan, 
                        const costmap_2d::Costmap2DROS& costmap, 
                        const std::string& global_frame,
                        std::vector<geometry_msgs::PoseStamped>& transformed_plan)
    {
    const geometry_msgs::PoseStamped& plan_pose = global_plan[0];

    transformed_plan.clear();
    geometry_msgs::TransformStamped transformStamped;
    try{
      if (!global_plan.size() > 0)
      {
        ROS_ERROR("Recieved plan with zero length");
        return false;
      }
      
      try{
        transformStamped = tf_buffer.lookupTransform(
          global_frame, 
          ros::Time(0), 
          plan_pose.header.frame_id, 
          plan_pose.header.stamp, 
          plan_pose.header.frame_id,
          ros::Duration(0.5)
        );
      } catch (tf2::TransformException &ex) {
        ROS_WARN("Could NOT transform turtle2 to turtle1: %s", ex.what());
      }

      //now we'll transform until points are outside of our distance threshold
      geometry_msgs::PoseStamped newer_pose;
      for(unsigned int i = 0; i < global_plan.size(); ++i)
      {
        const geometry_msgs::PoseStamped& pose = global_plan[i];
        tf2::doTransform(pose, newer_pose, transformStamped);
        std::cout<<"new x: "<<newer_pose.pose.position.x<<" new y "<<newer_pose.pose.position.y<<std::endl;
        transformed_plan.push_back(newer_pose);
      }

    }catch(tf2::LookupException& ex){
      ROS_ERROR("No Transform available Error: %s\n", ex.what());
      return false;
    }
    catch(tf::ConnectivityException& ex) {
      ROS_ERROR("Connectivity Error: %s\n", ex.what());
      return false;
    }
    catch(tf::ExtrapolationException& ex) {
      ROS_ERROR("Extrapolation Error: %s\n", ex.what());
      if (global_plan.size() > 0)
        ROS_ERROR("Global Frame: %s Plan Frame size %d: %s\n", global_frame.c_str(), (unsigned int)global_plan.size(), global_plan[0].header.frame_id.c_str());

      return false;
    }

    return true;
  }

  void LineFollower::initialize(std::string name, tf2_ros::Buffer* tf_buffer, costmap_2d::Costmap2DROS* costmap_ros){
    start_ = true;
    rotate_to_goal_ = false;
    tf_buffer_ = tf_buffer;
    costmap_ros_ = costmap_ros;
    ros::NodeHandle node;
    /*** For Multi-Robot ***/
    // node.getParam("/Libra/tf_prefix",robot_name_prefix);
    // base_tf_name = robot_name_prefix;
    // base_tf_name.append("/base_footprint");
    
    /*** For Only One Robot ***/
    base_tf_name = "base_footprint";
    // base_tf_name = "m_base_link";

    collision_planner_.initialize(name, tf_buffer_, costmap_ros_);
    tf_listener_ = new tf2_ros::TransformListener(*tf_buffer_);

    ROS_INFO("================================== INSIDE LINE FOLLOWER PLUGIN ==================================");
    ROS_INFO("%s initial finished!",robot_name_prefix.c_str());
    // std::cout<<robot_name_prefix<<std::endl;
    ROS_INFO("================================== INSIDE LINE FOLLOWER PLUGIN ==================================");
    
  }
}
