#ifndef _RISING_MANIPULATION__RISING_MANIPULATION_H_
#define _RISING_MANIPULATION__RISING_MANIPULATION_H_

// ROS
#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <tf2/convert.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/convert.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

// CPP
#include <std_srvs/Empty.h>
#include <std_srvs/Trigger.h>
#include <std_msgs/Byte.h>
#include <string>
#include <iostream>
#include <map>
#include <algorithm>
#include <vector>
#include <thread>

// CUSTOM ACTIONS
#include <rising_manipulation_msgs/MoveToAction.h>

// UTILS
#include <rcomponent/rcomponent.h>


// MOVEIT
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <warehouse_ros/database_connection.h>
#include <moveit/warehouse/constraints_storage.h>
#include <moveit_msgs/Constraints.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>


// SCENE MANAGER
#include "scene_manager/scene_manager.h"





class ManipulationApp : public rcomponent::RComponent
{
public:
  ManipulationApp(ros::NodeHandle h);
  ~ManipulationApp();

protected:

  // RComponent stuff

  //! Setups all the ROS' stuff
  virtual int rosSetup();
  //! Shutdowns all the ROS' stuff
  virtual int rosShutdown();
  //! Reads params from params server
  virtual void rosReadParams();

  virtual int setup();

  // States
  virtual void standbyState();
  virtual void readyState();
  

  // ROS stuff
  std::shared_ptr<tf2_ros::Buffer> tf2_buffer_;
  
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
  std::unique_ptr<tf2_ros::Buffer> tfBuffer_;
  geometry_msgs::TransformStamped transform_stamped;

  // Scene Manager
  std::unique_ptr<SceneManager> scene_manager_;

  // MoveIt stuff

  std::string group_name_;
  ros::WallDuration move_group_timeout_;
  moveit::planning_interface::MoveGroupInterfacePtr move_group_;
  moveit::planning_interface::MoveGroupInterface::Plan plan;
  moveit::planning_interface::PlanningSceneInterfacePtr planning_scene_interface_;
  planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor_;
  collision_detection::AllowedCollisionMatrix acm_;
  moveit_msgs::PlanningScene planning_scene_msg;

  warehouse_ros::DatabaseConnection::Ptr conn_;
  std::string host;
  int port;
  float connection_timeout;
  int connection_retries;

  double success_cartesian_plan;
  double allowed_fraction_success = 0.80;
  bool success_plan;
  bool success_move;
  bool success_execute;

  double scale_vel_;
  double scale_acc_;
  std::string end_effector_link_, robot_base_link_;

  std::string moveit_constraint;
  moveit_msgs::Constraints current_constraint;

  // Action servers
  virtual void goalCB(const std::string& action);
  virtual void preemptCB();
  std::string action_;

  actionlib::SimpleActionServer<rising_manipulation_msgs::MoveToAction>::GoalConstPtr move_to_goal_;
  std::shared_ptr<actionlib::SimpleActionServer<rising_manipulation_msgs::MoveToAction>> move_to_as_;

  // Action msgs
  rising_manipulation_msgs::MoveToFeedback move_to_feedback_;
  rising_manipulation_msgs::MoveToResult move_to_result_;

  // Action functions
  void move_to(std::string move_to_position);

  // Services
  ros::ServiceServer emergency_stop_trigger_;
  bool emergency_stop_cb(std_srvs::Trigger::Request &req, std_srvs::TriggerResponse &res);

  // Service Clients


  // Layout
  double crate_length_, crate_width_, crate_height_;

  // Threads
  boost::thread move_to_thread_;
  bool thread_active_flag_;
  bool action_finished_flag_;

};

#endif  // _RISING_MANIPULATION__RISING_MANIPULATION_H_
