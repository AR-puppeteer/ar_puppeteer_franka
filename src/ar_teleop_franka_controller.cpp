// AR teleop controller for joint position using touqe for usage in KTH RPL lab

#include <ar_puppeteer_franka/ar_teleop_franka_controller.h>
#include <cmath>
#include <functional>
#include <memory>
#include <controller_interface/controller_base.h>
#include <eigen_conversions/eigen_msg.h>
#include <franka/robot_state.h>
#include <franka_hw/trigger_rate.h>
#include <geometry_msgs/PoseStamped.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>
#include <ros/transport_hints.h>



namespace ar_puppeteer_franka {

//------------------- CHECK FUNCTIONS 

inline bool ARTeleopFrankaController::checkTorqueLimits(const Eigen::Matrix<double, 7, 1>& tau, const string& arm_id){
  for(int i = 0; i < 7; i++){
      if (abs(tau[i]) > perc_allowed_torque_limit_*torque_limits_[i]) {
          ROS_ERROR("%s: Close to torque %d limit: torque value=%f, torque limit=%f", arm_id.c_str(), i, tau[i],torque_limits_[i]);
          return false; 
      } 
  }  
  return true; 
}

inline  bool ARTeleopFrankaController::checkDerivativeTorqueLimits(const Eigen::Matrix<double, 7, 1>& dtau, const string& arm_id){
  for(int i = 0; i < 7; i++){
      if (abs(dtau[i]) > perc_allowed_dtorque_limit_*dtorque_limits_[i]) {
          ROS_ERROR("%s: Close to derivative torque %d limit: dtorque value=%f, dtorque limit=%f", arm_id.c_str(),  i, dtau[i],dtorque_limits_[i]);
          return false; 
      } 
  }  
  return true; 
}

inline bool ARTeleopFrankaController::checkJointLimits(const Eigen::Matrix<double, 7, 1>& q, const string& arm_id){   
  for(int i = 0; i < 7; i++){
      if ( q[i] < perc_allowed_q_limit_*q_min_limits_[i] || q[i] > perc_allowed_q_limit_*q_max_limits_[i] ) {
          ROS_ERROR("%s: Close to joint %d limit: joint value=%f, joint range=[%f, %f]", arm_id.c_str(), i, q[i],q_min_limits_[i], q_max_limits_[i] );
          return false; 
      } 
  }  
  return true; 
}

inline bool ARTeleopFrankaController::checkVelocityJointLimits(const Eigen::Matrix<double, 7, 1>& dq, const string& arm_id){
  for(int i = 0; i < 7; i++){
      if (abs(dq[i]) > perc_allowed_dq_limit_*dq_limits_[i]) {
          ROS_ERROR("%s: Close to joint velocity %d limit: dq value=%f, dq limit=%f", arm_id.c_str(), i, dq[i],dq_limits_[i]);
          return false; 
      } 
  }  
  return true; 

}

bool ARTeleopFrankaController::initArm(
    hardware_interface::RobotHW* robot_hw, const std::string& arm_id, const std::vector<std::string>& joint_names, ros::NodeHandle& node_handle, int ind_robot) {
  CustomFrankaDataContainerKthJointAR arm_data;
  auto* model_interface = robot_hw->get<franka_hw::FrankaModelInterface>();
  if (model_interface == nullptr) {
    ROS_ERROR_STREAM(
        "ARTeleopFrankaController: Error getting model interface from hardware");
    return false;
  }
  try {
    arm_data.model_handle_ = std::make_unique<franka_hw::FrankaModelHandle>(
        model_interface->getHandle(arm_id + "_model"));
  } catch (hardware_interface::HardwareInterfaceException& ex) {
    ROS_ERROR_STREAM(
        "ARTeleopFrankaController: Exception getting model handle from "
        "interface: "
        << ex.what());
    return false;
  }

  auto* state_interface = robot_hw->get<franka_hw::FrankaStateInterface>();
  if (state_interface == nullptr) {
    ROS_ERROR_STREAM(
        "ARTeleopFrankaController: Error getting state interface from hardware");
    return false;
  }
  try {
    arm_data.state_handle_ = std::make_unique<franka_hw::FrankaStateHandle>(
        state_interface->getHandle(arm_id + "_robot"));
  } catch (hardware_interface::HardwareInterfaceException& ex) {
    ROS_ERROR_STREAM(
        "ARTeleopFrankaController: Exception getting state handle from "
        "interface: "
        << ex.what());
    return false;
  }

  auto* effort_joint_interface = robot_hw->get<hardware_interface::EffortJointInterface>();
  if (effort_joint_interface == nullptr) {
    ROS_ERROR_STREAM(
        "ARTeleopFrankaController: Error getting effort joint interface from "
        "hardware");
    return false;
  }
  for (size_t i = 0; i < 7; ++i) {
    try {
      arm_data.joint_handles_.push_back(effort_joint_interface->getHandle(joint_names[i]));
    } catch (const hardware_interface::HardwareInterfaceException& ex) {
      ROS_ERROR_STREAM(
          "ARTeleopFrankaController: Exception getting joint handles: "
          << ex.what());
      return false;
    }
  }
   
  arm_data.Kd_q_.setIdentity(); 
  arm_data.Kd_q_.topLeftCorner(4,4) << 600* Eigen::MatrixXd::Identity(4,4);
  arm_data.Kd_q_(4,4) = 250.0; 
  arm_data.Kd_q_(5,5) = 150.0; 
  arm_data.Kd_q_(6,6) = 50.0; 

  arm_data.Dd_q_.setIdentity(); 
  arm_data.Dd_q_.topLeftCorner(3,3) << 50* Eigen::MatrixXd::Identity(3,3);
  arm_data.Dd_q_(3,3) = 20.0; 
  arm_data.Dd_q_(4,4) = 20.0; 
  arm_data.Dd_q_(5,5) = 20.0; 
  arm_data.Dd_q_(6,6) = 10.0; 

  cout <<  arm_id << " Kd_q_ "<<endl << arm_data.Kd_q_<<endl; 
  cout <<  arm_id << " Dd_q_ "<<endl << arm_data.Dd_q_<<endl; 

  cout << "arm_id: " << arm_id_ << endl; 

  try {
    arms_data_.emplace(std::make_pair(arm_id, std::move(arm_data)));
    cout << "Emplace done" << endl;
  } catch (const std::exception& e) {
    ROS_ERROR_STREAM("Exception during emplace: " << e.what());
    return false;
  }

  return true;
}


bool ARTeleopFrankaController::init(hardware_interface::RobotHW* robot_hw,
                                                      ros::NodeHandle& node_handle) {  
  // Get arm id
  if (!node_handle.getParam("arm_id", arm_id_)) {
    ROS_ERROR_STREAM(
        "ARTeleopFrankaController: Could not read parameter arm_id");
    return false;
  }
  
  // Get joint names
  std::vector<std::string> joint_names;
  if (!node_handle.getParam("joint_names", joint_names) || joint_names.size() != 7) {
    ROS_ERROR(
        "ARTeleopFrankaController: Invalid or no left_joint_names parameters provided, aborting controller init!");
    return false;
  }
 

  // ------------ Subscribers
  // Desired poses for the robots
  boost::function<void(const geometry_msgs::PoseStamped::ConstPtr&)> callbackTarget =
      boost::bind(&ARTeleopFrankaController::targetJointCallback, this, _1, arm_id_);

  ros::SubscribeOptions subscribe_options;
  subscribe_options.init("panda/target_joint_positions", 1, callbackTarget);
  subscribe_options.transport_hints = ros::TransportHints().reliable().tcpNoDelay();
  sub_target_pose_ = node_handle.subscribe(subscribe_options);

  int ind_robot = 0; 
  bool success = initArm(robot_hw, arm_id_, joint_names, node_handle, ind_robot++ );

  rate_trigger_ = franka_hw::TriggerRate(DEFAULT_PUBLISH_RATE); 

  
  // Setup publishers
  tau_d_pub_.init(node_handle, arm_id_+"_tau", 1, true);
  gravity_pub_.init(node_handle, arm_id_+"_gravity", 1, true); 
  q_error_pub_.init(node_handle, arm_id_+"_q_error", 1, true);  
  time_pub_.init(node_handle, "current_time", 1, true); 
  // Torque limits
  torque_limits_ << 87, 87, 87, 87, 12, 12, 12; 
  dtorque_limits_ << 1000, 1000, 1000, 1000, 1000, 1000, 1000; 
  perc_allowed_torque_limit_ = 0.8; 
  perc_allowed_dtorque_limit_ = 0.7; 
  // Joint limits 
  perc_allowed_q_limit_ = 0.9; 
  perc_allowed_dq_limit_ = 0.95; 
  q_min_limits_ << -2.8973, -1.7628, -2.8973, -3.0718, -2.8973, -0.0175, -2.8973; 
  q_max_limits_ << 2.8973, 1.7628, 2.8973, -0.0698, 2.8973, 3.7525, 2.8973; 
  dq_limits_<< 2.1750, 2.1750, 2.1750, 2.1750, 2.61, 2.61, 2.61; 


  return success;
}

void ARTeleopFrankaController::startingArm(CustomFrankaDataContainerKthJointAR& arm_data){

 // compute initial velocity with jacobian and set x_attractor and q_d_nullspace
  // to initial configuration
  franka::RobotState initial_state = arm_data.state_handle_->getRobotState();
  Eigen::Map<Eigen::Matrix<double, 7, 1>> dq_initial(initial_state.dq.data());
  Eigen::Map<Eigen::Matrix<double, 7, 1>> q_initial(initial_state.q.data());
  Eigen::Affine3d initial_transform(Eigen::Matrix4d::Map(initial_state.O_T_EE.data()));


  // set nullspace target configuration to initial q
  arm_data.q_d_nullspace_ = q_initial;

  // Init pose    
  arm_data.initial_q_ = q_initial; 
  // init 
  pose_des_q_=arm_data.initial_q_;
  arm_data.q_filtered_=arm_data.initial_q_;
}

void ARTeleopFrankaController::starting(const ros::Time& /*time*/) {

  for (auto& arm_data : arms_data_) {
    startingArm(arm_data.second);
   }
 
  current_time_ = 0;
  stop_command_ = false;   
}

void ARTeleopFrankaController::update(const ros::Time& /*time*/,
                                                        const ros::Duration& period) {

  ros::Time starttime_upd = ros::Time::now(); 
  // Update arms
  for (auto& arm_data : arms_data_) {
    updateArm(arm_data.second, arm_data.first, period);
  }

  auto& arm_data = arms_data_.at(arm_id_);

  // Publishers
  ros::Time timestamp = ros::Time::now(); 
  if (rate_trigger_()){    

    // Publish desired tau
    publishTau(tau_d_pub_, arm_data.tau_d_,timestamp);
    publishTau(gravity_pub_, arm_data.gravity_,timestamp);

    // Publish q error
    publishTau(q_error_pub_, arm_data.q_error_,timestamp); 


    publishTime(time_pub_, current_time_); 
  }
  ros::Time endtime_upd = ros::Time::now(); 
  ros::Duration difftime = endtime_upd-starttime_upd; 
  current_time_ = current_time_ + period.toSec(); 
}

void ARTeleopFrankaController::updateArm(CustomFrankaDataContainerKthJointAR& arm_data, const string& arm_id, const ros::Duration& period){

  // get state variables
  franka::RobotState robot_state = arm_data.state_handle_->getRobotState();
  std::array<double, 49> inertia_array = arm_data.model_handle_->getMass();
  std::array<double, 7> coriolis_array = arm_data.model_handle_->getCoriolis();
  std::array<double, 7> gravity_array = arm_data.model_handle_->getGravity();
  
  // convert to Eigen
  Eigen::Map<Eigen::Matrix<double, 7, 1>> coriolis(coriolis_array.data());
  Eigen::Map<Eigen::Matrix<double, 7, 1>> gravity(gravity_array.data());
  Eigen::Map<Eigen::Matrix<double, 7, 7>> inertia(inertia_array.data());
  Eigen::Map<Eigen::Matrix<double, 7, 1>> q(robot_state.q.data());
  Eigen::Map<Eigen::Matrix<double, 7, 1>> dq(robot_state.dq.data());
  Eigen::Map<Eigen::Matrix<double, 7, 1>> tau_J_d( robot_state.tau_J_d.data());

  // Check position and velocity limits 
  bool check_joint = checkJointLimits(q, arm_id); 
  bool check_velocity = checkVelocityJointLimits(dq, arm_id);
  if (!check_joint || !check_velocity) stop_command_ = true; 


  // joint space motion
 
  Eigen::VectorXd tau_task(7); 

  double alpha = 0.02;
  for (size_t i = 0; i < 7; i++) {
    arm_data.q_filtered_[i] = (1 - alpha) * arm_data.q_filtered_[i] + alpha * pose_des_q_[i];
  }

  // simple PD control

  tau_task << arm_data.Kd_q_ *(arm_data.q_filtered_ - q) - arm_data.Dd_q_ *dq;

  tau_task << saturateTorqueRate(arm_data, tau_task, tau_J_d);
  arm_data.q_error_ = arm_data.q_filtered_- q;  

 
  // Check torque commands 
  bool check_torque = checkTorqueLimits(tau_task, arm_id); 
  if (!check_torque) stop_command_ = true;


  arm_data.tau_d_ = tau_task; 
  if (!stop_command_ )
   for (size_t i = 0; i < 7; ++i) {
    arm_data.joint_handles_[i].setCommand(tau_task(i));
  }else{
    for (size_t i = 0; i < 7; ++i) {
    arm_data.joint_handles_[i].setCommand(0);
    }
  }

}

Eigen::Matrix<double, 7, 1> ARTeleopFrankaController::saturateTorqueRate(
    const CustomFrankaDataContainerKthJointAR& arm_data,
    const Eigen::Matrix<double, 7, 1>& tau_d_calculated,
    const Eigen::Matrix<double, 7, 1>& tau_J_d) {  // NOLINT (readability-identifier-naming)
  Eigen::Matrix<double, 7, 1> tau_d_saturated{};
  for (size_t i = 0; i < 7; i++) {
    double difference = tau_d_calculated[i] - tau_J_d[i];
    tau_d_saturated[i] = tau_J_d[i] + std::max(std::min(difference, arm_data.delta_tau_max_),
                                               -arm_data.delta_tau_max_);
  }
  return tau_d_saturated;
}


void ARTeleopFrankaController::targetJointCallback(const geometry_msgs::PoseStamped::ConstPtr& msg, const string& robot_id){
  
  auto& arm_data = arms_data_.at(robot_id);
  franka::RobotState initial_state = arm_data.state_handle_->getRobotState();
    // convert to eigen
  Eigen::Map<Eigen::Matrix<double, 7, 1>> q_initial(initial_state.q.data());

  pose_des_q_ <<msg->pose.position.x, msg->pose.position.y, msg->pose.position.z, msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z, msg->pose.orientation.w;
}


void ARTeleopFrankaController::publishError(realtime_tools::RealtimePublisher<geometry_msgs::PoseStamped>& arm_pub, const Eigen::VectorXd& error,const ros::Time& timestamp){
  
  if (arm_pub.trylock()) {
    arm_pub.msg_.header.stamp = ros::Time::now(); 
    arm_pub.msg_.pose.position.x = error[0]; 
    arm_pub.msg_.pose.position.y = error[1]; 
    arm_pub.msg_.pose.position.z = error[2]; 
    arm_pub.msg_.pose.orientation.x = error[3]; 
    arm_pub.msg_.pose.orientation.y = error[4]; 
    arm_pub.msg_.pose.orientation.z = error[5]; 
    arm_pub.unlockAndPublish();
  }
}


void ARTeleopFrankaController:: publishTime(realtime_tools::RealtimePublisher<std_msgs::Float64>& time_pub, double time){
  if (time_pub.trylock()) {
      time_pub.msg_.data = time; 
      
      time_pub.unlockAndPublish();
    }
}


  void ARTeleopFrankaController::publishTau(realtime_tools::RealtimePublisher<geometry_msgs::PoseStamped>& arm_pub, const Eigen::VectorXd& tau, const ros::Time& timestamp){
    if (arm_pub.trylock()) {
      arm_pub.msg_.header.stamp = timestamp; 
      arm_pub.msg_.pose.position.x = tau[0]; 
      arm_pub.msg_.pose.position.y = tau[1]; 
      arm_pub.msg_.pose.position.z = tau[2]; 
      arm_pub.msg_.pose.orientation.x = tau[3]; 
      arm_pub.msg_.pose.orientation.y = tau[4]; 
      arm_pub.msg_.pose.orientation.z = tau[5]; 
      arm_pub.msg_.pose.orientation.w = tau[6]; 
      arm_pub.unlockAndPublish();
    }

  }

}  // namespace ar_puppeteer_franka

PLUGINLIB_EXPORT_CLASS(
    ar_puppeteer_franka::ARTeleopFrankaController,
    controller_interface::ControllerBase)