#include <rising_manipulation/manipulation_app.h>

ManipulationApp::ManipulationApp(ros::NodeHandle h) : RComponent(h)
{
  init(h);
}
ManipulationApp::~ManipulationApp()
{
}

void ManipulationApp::rosReadParams()
{
  // Load required parameters from ros parameter server
  bool required = true;
  bool not_required = false;

  group_name_ = "arm";
  readParam(pnh_, "group_name", group_name_, group_name_, required);

  host = "localhost";
  readParam(pnh_, "host", host, host, required);  

  port = 33829;
  readParam(pnh_, "port", port, port, required);

  double timeout = 20;
  readParam(pnh_, "move_group_timeout", timeout, timeout, not_required);
  move_group_timeout_ = ros::WallDuration(timeout);

  moveit_constraint = "";
  readParam(pnh_, "moveit_constraint", moveit_constraint, moveit_constraint, required); 

  scale_vel_ = 1;
  readParam(pnh_, "scale_vel", scale_vel_, scale_vel_, required);

  scale_acc_ = 1;
  readParam(pnh_, "scale_acc", scale_acc_, scale_acc_, required);

  end_effector_link_ = "robot_vgc10_vgc10_link";
  readParam(pnh_, "end_effector_link", end_effector_link_, end_effector_link_, required);

  robot_base_link_ = "robot_base_footprint";
  readParam(pnh_, "robot_base_link", robot_base_link_, robot_base_link_, required);

  crate_length_ = 0.6;
  readParam(pnh_, "crate/length", crate_length_, crate_length_, required);

  crate_width_ = 0.4;
  readParam(pnh_, "crate/width", crate_width_, crate_width_, required);
  
  crate_height_ = 0.14;
  readParam(pnh_, "crate/height", crate_height_, crate_height_, required);

}

int ManipulationApp::rosSetup()
{
  if (ros_initialized)
  {
    RCOMPONENT_INFO("Already initialized");

    return rcomponent::INITIALIZED;
  }


  tf2_buffer_.reset(new tf2_ros::Buffer);

  // Reset move group interface
  try
  {
    move_group_.reset(
        new moveit::planning_interface::MoveGroupInterface(group_name_, tf2_buffer_, move_group_timeout_));
  }
  catch (const std::runtime_error& e)
  {
    RCOMPONENT_ERROR("Cannot create move group with group name: %s. Is MoveIt running? Group name is correct?",
                     group_name_.c_str());
    RCOMPONENT_ERROR_STREAM("Exception: " << e.what());
    return rcomponent::ERROR;
  }

  // Reset planning scene interface
  bool wait = true;
  std::string name_space = "";
  planning_scene_interface_.reset(
        new moveit::planning_interface::PlanningSceneInterface(name_space, wait));

  // Reset planning scene monitor
  planning_scene_monitor_.reset(new planning_scene_monitor::PlanningSceneMonitor("robot_description"));      

  // Update the planning scene monitor with the current state
  bool success = planning_scene_monitor_->requestPlanningSceneState("/get_planning_scene");
  ROS_INFO_STREAM("Request planning scene " << (success ? "succeeded." : "failed."));

  // Keep up to date with new changes
  planning_scene_monitor_->startSceneMonitor("/move_group/monitored_planning_scene");

  // Reset manipulation application action servers
  bool autostart = false;
  move_to_as_.reset(
      new actionlib::SimpleActionServer<rising_manipulation_msgs::MoveToAction>(pnh_, "move_to", autostart));
  move_to_as_->registerGoalCallback(boost::bind(&ManipulationApp::goalCB, this, std::string("move_to")));
  move_to_as_->registerPreemptCallback(boost::bind(&ManipulationApp::preemptCB, this));
  
  // Service
  emergency_stop_trigger_ = pnh_.advertiseService("emergency_stop_trigger", &ManipulationApp::emergency_stop_cb,this);

  // Connect to moveit's warehouse mongo db database
  conn_ = moveit_warehouse::loadDatabase();
  conn_->setParams(host, port);

  ROS_INFO("Connecting to warehouse on %s:%d", host.c_str(), port);

  while (!conn_->connect())
  {
    ROS_ERROR("Failed to connect to DB on %s:%d ", host.c_str(), port);
    ros::Duration(2).sleep();
    conn_->setParams(host, port);
  }
  
  // Retrieve stored moveit motion constrains in database
  move_group_->setConstraintsDatabase(host,port);
  std::vector< std::string > stored_constraints = move_group_->getKnownConstraints();
  if (stored_constraints.empty())
    ROS_WARN("There are no constraints stored in database");
  else
  {
    ROS_INFO("Constraints currently stored in database:");
    for (const std::string& name : stored_constraints)
      ROS_INFO(" * %s", name.c_str());
  }

  
  // TF listener
  tfBuffer_ = std::make_unique<tf2_ros::Buffer>();
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tfBuffer_);


  // Scene manager
  bool wait_ = true;
  scene_manager_ = std::make_unique<SceneManager>(nh_ , wait_);

  return RComponent::rosSetup();
}

int ManipulationApp::rosShutdown()
{
  return RComponent::rosShutdown();
}

int ManipulationApp::setup()
{

  // Set move_group's velocity scaling factor 
  move_group_->setMaxVelocityScalingFactor(scale_vel_);
  move_group_->setMaxAccelerationScalingFactor(scale_acc_);

  // Checks if rcompnent has been correctly initialized
  int setup_result;

  setup_result = rcomponent::RComponent::setup();
  if (setup_result != rcomponent::OK)
  {
    return setup_result;
  }

  // Create planning scene
  scene_manager_->initScene(); // CAMBIAR (COMPROBAR SI ES TRUE)

  return rcomponent::OK;

}

void ManipulationApp::standbyState()
{
  thread_active_flag_ = false;
  action_finished_flag_ = false;

  move_group_->clearPathConstraints();

  success_move=true;

  // Select constraint and check whether it exists
  move_group_->setPathConstraints(moveit_constraint);

  current_constraint = move_group_->getPathConstraints();
  if(moveit_constraint != current_constraint.name){
    ROS_ERROR("Desired moveit_constraint: %s is not available in database, please modify or run generate_path_constraints.cpp", moveit_constraint.c_str());
    switchToState(robotnik_msgs::State::FAILURE_STATE);
    return;
  }

  ROS_INFO("Planning with constraint: %s",moveit_constraint.c_str());

  RCOMPONENT_INFO_STREAM("Started server: move to");
  move_to_as_->start();

  switchToState(robotnik_msgs::State::READY_STATE);
}

void ManipulationApp::readyState()
{ 

  if (move_to_as_->isActive() == false)
  {
    ROS_INFO_THROTTLE(3, "I do not have a goal");
    return;
  }


  ROS_INFO_THROTTLE(3, "I have a new goal!");

  if(move_to_as_->isActive() == true && thread_active_flag_ == false){
    thread_active_flag_ = true;
    move_to_thread_= boost::thread(&ManipulationApp::move_to,this,move_to_goal_->to);
  }   


  if(thread_active_flag_ == true && action_finished_flag_ == true)
  {
    move_to_thread_.join();
    thread_active_flag_ = false;
    action_finished_flag_ = false;
  }
  
}

void ManipulationApp::goalCB(const std::string& action)
{
  RCOMPONENT_INFO_STREAM("I have received an action to: " << action);
  action_ = action;
  if(move_to_as_->isActive()){
    ROS_INFO("Cannot process %s action, another action is active", action.c_str());
    return;
  }
  if (action_ == "move_to"){
    move_to_goal_ = move_to_as_->acceptNewGoal();
  }
}


void ManipulationApp::preemptCB()
{
  RCOMPONENT_INFO_STREAM("ACTION: Preempted");
  move_group_->stop();

  if(move_to_as_->isActive()){
    /* destack_crate_thread_.interrupt(); */
    move_to_result_.success = true;
    move_to_result_.message = "Move to action preempted";
    move_to_as_->setPreempted(move_to_result_);
    move_to_thread_.join();
    thread_active_flag_ = false;
    action_finished_flag_ = false;
  }

}

bool ManipulationApp::emergency_stop_cb(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res)
{
  move_group_->stop();
  res.success = true;
  res.message = "Sending stop trigger to robot";
}


void ManipulationApp::move_to(std::string move_to_position)
{ 

  // Clear pose targets
  move_group_->clearPoseTargets();

  // Look if position is defined in srdf
  if(!move_group_->setNamedTarget(move_to_position)){
    move_to_result_.success = false;
    move_to_result_.message = "Position does not exist, it is not defined in srdf.";
    move_to_as_->setAborted(move_to_result_);
    thread_active_flag_ = false;
    ROS_ERROR(move_to_result_.message.c_str());
    return;
  }

  // Plan to pre-position goal
  for (int i = 0; i < 5; i++)
  { 
	  ROS_INFO("Try to plan %d", i);
    success_plan = (move_group_->plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    if (success_plan == true) break;
  }

  // If plan is successful execute trajectory
  if(success_plan){
    move_to_feedback_.state.clear();
    move_to_feedback_.state = "Plan to desired position computed";
    move_to_as_->publishFeedback(move_to_feedback_);

    while(move_to_as_->isActive())
    {
      //Check if goal is active and move to goal
      if (!move_to_as_->isActive()) return;
      success_execute = (move_group_->execute(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

      //Check if execute is successful
      if(success_execute){
        move_to_feedback_.state.clear();
        move_to_feedback_.state = "Moved end-effector to desired position";
        move_to_as_->publishFeedback(move_to_feedback_);

        move_to_result_.success = true;
        move_to_result_.message = "Move end-effector to desired position action: SUCCESSFUL";
        move_to_as_->setSucceeded(move_to_result_);
        action_finished_flag_ = true;
        return;
      }else{
        move_to_result_.success = false;
        move_to_result_.message = "Could not move end-effector to desired position";
        move_to_as_->setAborted(move_to_result_);
        thread_active_flag_ = false;
        return;
      }
    }    

    if (!move_to_as_->isActive()) return;
          
  }else{
    move_to_result_.success = false;
    move_to_result_.message = "Could not plan to desired position";
    move_to_as_->setAborted(move_to_result_);
    thread_active_flag_ = false;
    return;
  }

 
}