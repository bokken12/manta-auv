/*   Written by Kristoffer Rakstad Solberg, Student
     and Morten Fyhn Amundsen, Student
     Copyright (c) 2019 Manta AUV, Vortex NTNU.
     All rights reserved. */


#include "dp_controller/controller_ros.h"

#include "vortex/eigen_helper.h"
#include <tf/transform_datatypes.h>
#include <eigen_conversions/eigen_msg.h>

#include "std_msgs/String.h"

#include <math.h>
#include <map>
#include <string>
#include <vector>

Controller::Controller(ros::NodeHandle nh) : m_nh(nh), m_frequency(10)
{
  // Subscribers
  //m_command_sub = m_nh.subscribe("/manta/waypoints", 300, &Controller::waypointCallback, this);
  m_state_sub = m_nh.subscribe("/odometry/filtered", 1, &Controller::stateCallback, this);
  m_mode_sub = m_nh.subscribe("/manta/mode", 1, &Controller::controlModeCallback, this);

  // Publishers
  m_wrench_pub  = m_nh.advertise<geometry_msgs::Wrench>("dp_input", 1);
  m_mode_pub    = m_nh.advertise<std_msgs::String>("controller/mode", 10);
  m_debug_pub   = m_nh.advertise<vortex_msgs::Debug>("debug/controlstates", 10);

  // Initial control mode
  m_control_mode = ControlModes::OPEN_LOOP;

  // Launch file specifies manta.yaml as directory
  if (!m_nh.getParam("/controller/frequency", m_frequency))
    ROS_WARN("Failed to read parameter controller frequency, defaulting to %i Hz.", m_frequency);
  std::string s;
  if (!m_nh.getParam("/computer", s))
  {
    s = "pc-debug";
    ROS_WARN("Failed to read parameter computer");
  }
  if (s == "pc-debug")
    m_debug_mode = true;

  if(!m_nh.getParam("/controller/circleOfAcceptance", R)){
    ROS_WARN("Failed to read parameter circleOfAcceptance");
  }  

  m_state.reset(new State());
  initSetpoints();
  initPositionHoldController();

  // Set up a dynamic reconfigure server
  dynamic_reconfigure::Server<dp_controller::VortexControllerConfig>::CallbackType dr_cb;
  dr_cb = boost::bind(&Controller::configCallback, this, _1, _2);
  m_dr_srv.setCallback(dr_cb);

  ROS_INFO("Initialized at %i Hz.", m_frequency);

  /* Action server */
  //ros::NodeHandle nodeHandle("move_base");
  mActionServer = new MoveBaseActionServer(m_nh, "move_base", /*autostart*/false);

  //register the goal and feeback callbacks
  mActionServer->registerGoalCallback(boost::bind(&Controller::actionGoalCallBack, this));
  mActionServer->registerPreemptCallback(boost::bind(&Controller::preemptCallBack, this));
  mActionServer->start();
  ROS_INFO("Started action server.");
}

void Controller::controlModeCallback(const vortex_msgs::PropulsionCommand& msg){
  
  if (!healthyMessage(msg))
    return;

  ControlMode new_control_mode = getControlMode(msg);
  if (new_control_mode != m_control_mode)
  {
    m_control_mode = new_control_mode;
    //resetSetpoints();
    ROS_INFO_STREAM("Changing mode to " << controlModeString(m_control_mode) << ".");
  }
  publishControlMode();
}

/* ACTION SERVER */

// This action is event driven, the action code only runs when the callbacks occur therefore
// a preempt callback is created to ensure that the action responds promptly to a cancel request.
// The callback function takes no arguments and sets preempted on the action server.
// I wonder whether this needs to be mutexed.

void Controller::preemptCallBack()
{

 	//notify the ActionServer that we've successfully preempted
    ROS_DEBUG_NAMED("move_base","Move base preempting the current goal");	

    // set the action state to preempted
    mActionServer->setPreempted();
}

void Controller::actionGoalCallBack()
{

  // accept the new goal - do I have to cancel a pre-existing one first?
  mGoal = mActionServer->acceptNewGoal()->target_pose;

  // print the current goal
  ROS_INFO("Controller::actionGoalCallBack(): driving to %2.2f/%2.2f/%2.2f", mGoal.pose.position.x, mGoal.pose.position.y, mGoal.pose.position.z);

  // Transform from Msg to Eigen
  tf::pointMsgToEigen(mGoal.pose.position, setpoint_position);
  tf::quaternionMsgToEigen(mGoal.pose.orientation, setpoint_orientation);

  // setpoint declared as private variable
  m_setpoints->set(setpoint_position, setpoint_orientation);

}


ControlMode Controller::getControlMode(const vortex_msgs::PropulsionCommand& msg) const
{
  ControlMode new_control_mode = m_control_mode;
  for (unsigned i = 0; i < msg.control_mode.size(); ++i)
  {
    if (msg.control_mode[i])
    {
      new_control_mode = static_cast<ControlMode>(i);
      break;
    }
  }
  return new_control_mode;
}

//void Controller::stateCallback(const vortex_msgs::RovState &msg)
void Controller::stateCallback(const nav_msgs::Odometry &msg)
{

  // Convert to eigen for computation
  nav_msgs::Odometry flip_x_msg = msg;
  flip_x_msg.pose.pose.position.x = -1*flip_x_msg.pose.pose.position.x;
  tf::pointMsgToEigen(flip_x_msg.pose.pose.position, position);
  tf::quaternionMsgToEigen(flip_x_msg.pose.pose.orientation, orientation);
  tf::twistMsgToEigen(flip_x_msg.twist.twist, velocity);

  bool orientation_invalid = (abs(orientation.norm() - 1) > c_max_quat_norm_deviation);
  if (isFucked(position) || isFucked(velocity) || orientation_invalid)
  {
    ROS_WARN_THROTTLE(1, "Invalid state estimate received, ignoring...");
    return;
  }

  // takes a odometry message and transforms to Eigen message
  m_state->set(position, orientation, velocity);


  // ACTION SERVER

  // save current state to private variable
  // geometry_msgs/PoseStamped Pose
  if (!mActionServer->isActive())
      return;

  // return a feedback message to the client
  feedback_.base_position.header.stamp = ros::Time::now();
  feedback_.base_position.pose = msg.pose.pose;
  mActionServer->publishFeedback(feedback_);


  // if within circle of acceptance, return result succeeded
  if (m_controller->circleOfAcceptance(position,setpoint_position,R)){
  	mActionServer->setSucceeded(move_base_msgs::MoveBaseResult(), "Goal reached.");
  }

}

void Controller::configCallback(const dp_controller::VortexControllerConfig &config, uint32_t level)
{
  ROS_INFO_STREAM("Setting gains: [vel = " << config.velocity_gain << ", pos = " << config.position_gain
    << ", rot = " << config.attitude_gain << "]");
  m_controller->setGains(config.velocity_gain, config.position_gain, config.attitude_gain);
}

void Controller::spin()
{
  // Declaration of general forces
  Eigen::Vector6d    tau_command          = Eigen::VectorXd::Zero(6);
  Eigen::Vector6d    tau_openloop         = Eigen::VectorXd::Zero(6);
  Eigen::Vector6d    tau_restoring        = Eigen::VectorXd::Zero(6);
  Eigen::Vector6d    tau_staylevel        = Eigen::VectorXd::Zero(6);
  Eigen::Vector6d    tau_depthhold        = Eigen::VectorXd::Zero(6);
  Eigen::Vector6d    tau_headinghold      = Eigen::VectorXd::Zero(6);
  Eigen::Vector6d    tau_surgehold        = Eigen::VectorXd::Zero(6);
  Eigen::Vector6d    tau_posehold         = Eigen::VectorXd::Zero(6);  

  Eigen::Vector3d    position_state       = Eigen::Vector3d::Zero();
  Eigen::Quaterniond orientation_state    = Eigen::Quaterniond::Identity();
  Eigen::Vector6d    velocity_state       = Eigen::VectorXd::Zero(6);

  Eigen::Vector3d    position_setpoint    = Eigen::Vector3d::Zero();
  Eigen::Quaterniond orientation_setpoint = Eigen::Quaterniond::Identity();

  // Message declaration
  geometry_msgs::Wrench msg;
  vortex_msgs::Debug    dbg_msg;

  ros::Rate rate(m_frequency);
  while (ros::ok())
  {
    
    // gets the newest state and newest setpoints as Eigen
    m_state->get(&position_state, &orientation_state, &velocity_state);
    m_setpoints->get(&position_setpoint, &orientation_setpoint);
    m_setpoints->get(&tau_openloop);

    if (m_debug_mode)
    publishDebugMsg(position_state, orientation_state, velocity_state,
                    position_setpoint, orientation_setpoint);

    tau_command.setZero();

    switch (m_control_mode)
    {

      // idle
      case ControlModes::OPEN_LOOP:
      tau_command = tau_openloop;
      break;

      // 3D coordinates
      case ControlModes::POSE_HOLD:
  
      tau_posehold = poseHold(tau_openloop,
                                position_state,
                                orientation_state,
                                velocity_state,
                                position_setpoint,
                                orientation_setpoint);
      tau_command = tau_posehold;
      break;

      // 3D coordinates with heading
      case ControlModes::POSE_HEADING_HOLD:
      tau_surgehold = surgeHold(tau_openloop,
                                position_state,
                                orientation_state,
                                velocity_state,
                                position_setpoint);
      tau_headinghold = headingHold(tau_openloop,
                                    position_state,
                                    orientation_state,
                                    velocity_state,
                                    orientation_setpoint);
      tau_command = tau_surgehold + tau_headinghold;
      break;

      // adjust roll and pitch
      case ControlModes::STAY_LEVEL:
      tau_staylevel = stayLevel(orientation_state, velocity_state);
      tau_command = tau_openloop + tau_staylevel;
      break;

      // only heading
      case ControlModes::HEADING_HOLD:
      tau_headinghold = headingHold(tau_openloop,
                                    position_state,
                                    orientation_state,
                                    velocity_state,
                                    orientation_setpoint);
      tau_command = tau_headinghold;
      break;

      // only depth and heading
      case ControlModes::DEPTH_HEADING_HOLD:
      tau_depthhold = depthHold(tau_openloop,
                                position_state,
                                orientation_state,
                                velocity_state,
                                position_setpoint);
      tau_headinghold = headingHold(tau_openloop,
                                    position_state,
                                    orientation_state,
                                    velocity_state,
                                    orientation_setpoint);
      tau_command = tau_openloop + tau_depthhold + tau_headinghold;
      break;

      default:
      ROS_ERROR("Default control mode reached.");
      break;
    }

    tf::wrenchEigenToMsg(tau_command, msg);
    m_wrench_pub.publish(msg);

    ros::spinOnce();
    rate.sleep();
  }
}

void Controller::initSetpoints()
{
  std::vector<double> v;

  if (!m_nh.getParam("/propulsion/command/wrench/max", v))
    ROS_FATAL("Failed to read parameter max wrench command.");
  const Eigen::Vector6d wrench_command_max = Eigen::Vector6d::Map(v.data(), v.size());

  if (!m_nh.getParam("/propulsion/command/wrench/scaling", v))
    ROS_FATAL("Failed to read parameter scaling wrench command.");
  const Eigen::Vector6d wrench_command_scaling = Eigen::Vector6d::Map(v.data(), v.size());

  m_setpoints.reset(new Setpoints(wrench_command_scaling, wrench_command_max));
}

void Controller::resetSetpoints()
{
  // Reset setpoints to be equal to state
  Eigen::Vector3d position;
  Eigen::Quaterniond orientation;
  m_state->get(&position, &orientation);
  m_setpoints->set(position, orientation);
}

void Controller::updateSetpoint(PoseIndex axis)
{
  Eigen::Vector3d state;
  Eigen::Vector3d setpoint;

  switch (axis)
  {
    case SURGE:
    case SWAY:
    case HEAVE:

      m_state->get(&state);
      m_setpoints->get(&setpoint);

      setpoint[axis] = state[axis];
      m_setpoints->set(setpoint);
      break;

    case ROLL:
    case PITCH:
    case YAW:

      m_state->getEuler(&state);
      m_setpoints->getEuler(&setpoint);

      setpoint[axis-3] = state[axis-3];
      Eigen::Matrix3d R;
      R = Eigen::AngleAxisd(setpoint(2), Eigen::Vector3d::UnitZ())
        * Eigen::AngleAxisd(setpoint(1), Eigen::Vector3d::UnitY())
        * Eigen::AngleAxisd(setpoint(0), Eigen::Vector3d::UnitX());
      Eigen::Quaterniond quaternion_setpoint(R);
      m_setpoints->set(quaternion_setpoint);
      break;
  }
}

void Controller::initPositionHoldController()
{
  // Read controller gains from parameter server
  double a, b, c;
  if (!m_nh.getParam("/controller/velocity_gain", a))
    ROS_ERROR("Failed to read parameter velocity_gain.");
  if (!m_nh.getParam("/controller/position_gain", b))
    ROS_ERROR("Failed to read parameter position_gain.");
  if (!m_nh.getParam("/controller/attitude_gain", c))
    ROS_ERROR("Failed to read parameter attitude_gain.");

  // Read center of gravity and buoyancy vectors
  // from manta.yaml
  std::vector<double> r_G_vec, r_B_vec;
  if (!m_nh.getParam("/physical/center_of_mass", r_G_vec))
    ROS_FATAL("Failed to read robot center of mass parameter.");
  if (!m_nh.getParam("/physical/center_of_buoyancy", r_B_vec))
    ROS_FATAL("Failed to read robot center of buoyancy parameter.");
  Eigen::Vector3d r_G(r_G_vec.data());
  Eigen::Vector3d r_B(r_B_vec.data());

  // Read and calculate ROV weight and buoyancy
  // from manta.yaml
  double mass, displacement, acceleration_of_gravity, density_of_water;
  if (!m_nh.getParam("/physical/mass_kg", mass))
    ROS_FATAL("Failed to read parameter mass.");
  if (!m_nh.getParam("/physical/displacement_m3", displacement))
    ROS_FATAL("Failed to read parameter displacement.");
  if (!m_nh.getParam("/gravity/acceleration", acceleration_of_gravity))
    ROS_FATAL("Failed to read parameter acceleration of gravity");
  if (!m_nh.getParam("/water/density", density_of_water))
    ROS_FATAL("Failed to read parameter density of water");
  double W = mass * acceleration_of_gravity;
  double B = density_of_water * displacement * acceleration_of_gravity;

  m_controller.reset(new QuaternionPdController(a, b, c, W, B, r_G, r_B));
}

bool Controller::healthyMessage(const vortex_msgs::PropulsionCommand& msg)
{
  // Check that motion commands are in range
  for (int i = 0; i < msg.motion.size(); ++i)
  {
    if (msg.motion[i] > 1 || msg.motion[i] < -1)
    {
      ROS_WARN("Motion command out of range, ignoring message...");
      return false;
    }
  }

  // Check correct length of control mode vector
  if (msg.control_mode.size() != ControlModes::CONTROL_MODE_END)
  {
    ROS_WARN_STREAM_THROTTLE(1, "Control mode vector has " << msg.control_mode.size()
      << " element(s), should have " << ControlModes::CONTROL_MODE_END);
    return false;
  }

  // Check that exactly zero or one control mode is requested
  int num_requested_modes = 0;
  for (int i = 0; i < msg.control_mode.size(); ++i)
    if (msg.control_mode[i])
      num_requested_modes++;
  if (num_requested_modes > 1)
  {
    ROS_WARN_STREAM("Attempt to set " << num_requested_modes << " control modes at once, ignoring message...");
    return false;
  }

  return true;
}

void Controller::publishControlMode()
{
  std::string s = controlModeString(m_control_mode);
  std_msgs::String msg;
  msg.data = s;
  m_mode_pub.publish(msg);
}

void Controller::publishDebugMsg(const Eigen::Vector3d    &position_state,
                                 const Eigen::Quaterniond &orientation_state,
                                 const Eigen::Vector6d    &velocity_state,
                                 const Eigen::Vector3d    &position_setpoint,
                                 const Eigen::Quaterniond &orientation_setpoint)
{
  vortex_msgs::Debug dbg_msg;

  // Estimated position
  dbg_msg.state_position.x = position_state[0];
  dbg_msg.state_position.y = position_state[1];
  dbg_msg.state_position.z = position_state[2];

  // Estimated linear velocity
  dbg_msg.state_velocity.linear.x = velocity_state[0];
  dbg_msg.state_velocity.linear.y = velocity_state[1];
  dbg_msg.state_velocity.linear.z = velocity_state[2];

  // Estimated angular velocity
  dbg_msg.state_velocity.angular.x = velocity_state[3];
  dbg_msg.state_velocity.angular.y = velocity_state[4];
  dbg_msg.state_velocity.angular.z = velocity_state[5];

  // Setpoint position
  dbg_msg.setpoint_position.x = position_setpoint[0];
  dbg_msg.setpoint_position.y = position_setpoint[1];
  dbg_msg.setpoint_position.z = position_setpoint[2];

  // Debub state euler orientation
  Eigen::Vector3d dbg_state_orientation =
      orientation_state.toRotationMatrix().eulerAngles(2, 1, 0);
  Eigen::Vector3d dbg_setpoint_orientation =
      orientation_setpoint.toRotationMatrix().eulerAngles(2, 1, 0);

  // Debug state orientation  
  dbg_msg.state_yaw = dbg_state_orientation[0];
  dbg_msg.state_pitch = dbg_state_orientation[1];
  dbg_msg.state_roll = dbg_state_orientation[2];

  // Debug setpoint euler orientation
  dbg_msg.setpoint_yaw = dbg_setpoint_orientation[0];
  dbg_msg.setpoint_pitch = dbg_setpoint_orientation[1];
  dbg_msg.setpoint_roll = dbg_setpoint_orientation[2];

  /*
  // tau body
  Eigen::Vector6d tau_body = m_controller->getFeedback(position_state, Eigen::Quaterniond::Identity(), velocity_state,
                                    position_setpoint, Eigen::Quaterniond::Identity());  

  dbg_msg.tau_xb = tau_body[0];
  dbg_msg.tau_yb = tau_body[1];
  dbg_msg.tau_zb = tau_body[2]; */


  // publish
  m_debug_pub.publish(dbg_msg);
}

Eigen::Vector6d Controller::stayLevel(const Eigen::Quaterniond &orientation_state,
                                      const Eigen::Vector6d &velocity_state)
{
  // Convert quaternion setpoint to euler angles (ZYX convention)
  Eigen::Vector3d euler;
  euler = orientation_state.toRotationMatrix().eulerAngles(2, 1, 0);

  // Set pitch and roll setpoints to zero
  euler(EULER_PITCH) = 0;
  euler(EULER_ROLL)  = 0;

  // Convert euler setpoint back to quaternions
  Eigen::Matrix3d R;
  R = Eigen::AngleAxisd(euler(EULER_YAW),   Eigen::Vector3d::UnitZ())
    * Eigen::AngleAxisd(euler(EULER_PITCH), Eigen::Vector3d::UnitY())
    * Eigen::AngleAxisd(euler(EULER_ROLL),  Eigen::Vector3d::UnitX());
  Eigen::Quaterniond orientation_staylevel(R);

  Eigen::Vector6d tau = m_controller->getFeedback(Eigen::Vector3d::Zero(), orientation_state, velocity_state,
                                                Eigen::Vector3d::Zero(), orientation_staylevel);

  tau(ROLL)  = 0;
  tau(PITCH) = 0;

  return tau;
}

Eigen::Vector6d Controller::depthHold(const Eigen::Vector6d &tau_openloop,
                                      const Eigen::Vector3d &position_state,
                                      const Eigen::Quaterniond &orientation_state,
                                      const Eigen::Vector6d &velocity_state,
                                      const Eigen::Vector3d &position_setpoint)
{
  Eigen::Vector6d tau;

  bool activate_depthhold = fabs(tau_openloop(PoseIndex::HEAVE)) < c_normalized_force_deadzone;
  if (activate_depthhold)
  {
    tau = m_controller->getFeedback(position_state, Eigen::Quaterniond::Identity(), velocity_state,
                                    position_setpoint, Eigen::Quaterniond::Identity());

    // Allow only heave feedback command
    tau(SURGE) = 0;
    tau(SWAY)  = 0;
    tau(ROLL)  = 0;
    tau(PITCH) = 0;
    tau(YAW)   = 0;
  }
  else
  {
    updateSetpoint(HEAVE);
    tau.setZero();
  }

  return tau;
}

Eigen::Vector6d Controller::headingHold(const Eigen::Vector6d &tau_openloop,
                                        const Eigen::Vector3d &position_state,
                                        const Eigen::Quaterniond &orientation_state,
                                        const Eigen::Vector6d &velocity_state,
                                        const Eigen::Quaterniond &orientation_setpoint)
{
  Eigen::Vector6d tau;

  bool activate_headinghold = fabs(tau_openloop(PoseIndex::YAW)) < c_normalized_force_deadzone;
  if (activate_headinghold)
  {
    tau = m_controller->getFeedback(Eigen::Vector3d::Zero(), orientation_state, velocity_state,
                                  Eigen::Vector3d::Zero(), orientation_setpoint);

    // Allow only yaw feedback command
    tau(SURGE) = 0;
    tau(SWAY)  = 0;
    tau(HEAVE) = 0;
    tau(ROLL)  = 0;
    tau(PITCH) = 0;
  }
  else
  {
    updateSetpoint(YAW);
    tau.setZero();
  }

  return tau;
}

Eigen::Vector6d Controller::poseHold(const Eigen::Vector6d &tau_openloop,
                                      const Eigen::Vector3d &position_state,
                                      const Eigen::Quaterniond &orientation_state,
                                      const Eigen::Vector6d &velocity_state,
                                      const Eigen::Vector3d &position_setpoint,
                                      const Eigen::Quaterniond &orientation_setpoint)
{
  Eigen::Vector6d tau;

  bool activate_depthhold = fabs(tau_openloop(PoseIndex::HEAVE)) < c_normalized_force_deadzone;
  if (activate_depthhold)
  {
    tau = m_controller->getFeedback(position_state, orientation_state, velocity_state,
                                    position_setpoint, orientation_setpoint);

    // Allow only surge,sway,heave feedback command
    tau(ROLL)  = 0;
    tau(PITCH) = 0;
    tau(YAW)   = 0;
  }
  else
  {
    updateSetpoint(SURGE);
    updateSetpoint(SWAY);
    updateSetpoint(HEAVE);
    tau.setZero();
  }

  return tau;
}

Eigen::Vector6d Controller::surgeHold(const Eigen::Vector6d &tau_openloop,
                                      const Eigen::Vector3d &position_state,
                                      const Eigen::Quaterniond &orientation_state,
                                      const Eigen::Vector6d &velocity_state,
                                      const Eigen::Vector3d &position_setpoint)
{
  Eigen::Vector6d tau;

  bool activate_depthhold = fabs(tau_openloop(PoseIndex::HEAVE)) < c_normalized_force_deadzone;
  if (activate_depthhold)
  {
    tau = m_controller->getFeedback(position_state, Eigen::Quaterniond::Identity(), velocity_state,
                                    position_setpoint, Eigen::Quaterniond::Identity());

    // Allow only surge,sway,heave feedback command
    tau(ROLL)  = 0;
    tau(PITCH) = 0;
    tau(YAW)   = 0;
  }
  else
  {
    updateSetpoint(SURGE);
    updateSetpoint(SWAY);
    updateSetpoint(HEAVE);
    tau.setZero();
  }

  return tau;
}
