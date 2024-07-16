// Copyright (c) 2019 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#pragma once

#include <memory>
#include <string>
#include <vector>

#include <controller_interface/multi_interface_controller.h>
#include <dynamic_reconfigure/server.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/robot_hw.h>
#include <realtime_tools/realtime_publisher.h>
#include <ros/node_handle.h>
#include <ros/time.h>
#include <Eigen/Dense>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>

#include <franka_hw/franka_model_interface.h>
#include <franka_hw/franka_state_interface.h>
#include <franka_hw/trigger_rate.h>


#define n_arm 7
#define p_arm 6
#define DEFAULT_PUBLISH_RATE 500
using namespace std; 

namespace ar_puppeteer_franka {


struct CustomFrankaDataContainerKthJointAR {
  std::unique_ptr<franka_hw::FrankaStateHandle> state_handle_;  ///< To read to complete robot state.
  std::unique_ptr<franka_hw::FrankaModelHandle> model_handle_;  ///< To have access to e.g. jacobians.
  std::vector<hardware_interface::JointHandle> joint_handles_;  ///< To command joint torques.
  double filter_params_{0.005};       ///< [-] PT1-Filter constant to smooth target values set
                                      ///< by dynamic reconfigure servers (stiffness/damping)
                                      ///< or interactive markers for the target poses.
  double nullspace_stiffness_{20.0};  ///< [Nm/rad] To track the initial joint configuration in
                                      ///< the nullspace of the Cartesian motion.
  double nullspace_stiffness_target_{20.0};  ///< [Nm/rad] Unfiltered raw value.
  const double delta_tau_max_{1.0};          ///< [Nm/ms] Maximum difference in joint-torque per
                                             ///< timestep. Used to saturated torque rates to ensure
                                             ///< feasible commands.
  Eigen::Matrix<double, 7, 1> q_d_nullspace_;               ///< Target joint pose for nullspace
                                                            ///< motion. 
  
  Eigen::Matrix<double, 7, 7>  Kd_q_, Dd_q_; //desired inertia at the EE 
  Eigen::Matrix<double, 7, 1> q_filtered_;   

  Eigen::Matrix<double, 7, 1> tau_d_; // desired torques
  Eigen::Matrix<double, 7, 1> gravity_; // gravity vector
  Eigen::Matrix<double, 7, 1> initial_q_; // initial joint configurationpos  

  Eigen::Matrix<double, 7, 1>  q_error_; // error on joint configuration 
  
  int ind_robot_; 
};


class ARTeleopFrankaController: public controller_interface::MultiInterfaceController<
          franka_hw::FrankaModelInterface,
          hardware_interface::EffortJointInterface,
          franka_hw::FrankaStateInterface> {
 public:
  /**
   * Initializes the controller class to be ready to run.
   *
   * @param[in] robot_hw Pointer to a RobotHW class to get interfaces and resource handles.
   * @param[in] node_handle Nodehanlde that allows getting parameterizations from the server and
   * starting subscribers.
   * @return True if the controller was initialized successfully, false otherwise.
   */
  bool init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& node_handle) override;

  /**
   * Prepares the controller for the real-time execution. This method is executed once everytime the
   * controller is started and runs in real-time.
   */
  void starting(const ros::Time&) override;

  /**
   * Computes the control-law and commands the resulting joint torques to the robot.
   *
   * @param[in] period The control period (here 0.001s).
   */
  void update(const ros::Time&, const ros::Duration& period) override;

 private:
  std::map<std::string, CustomFrankaDataContainerKthJointAR>  arms_data_;  ///< Holds all relevant data for both arms.
  std::string arm_id_;   ///< Name of the left arm, retreived from the parameter server.
  double current_time_; // elapsed time from the beginning
  bool stop_command_; // if 1, zero torques are commanded
  franka_hw::TriggerRate rate_trigger_;

  Eigen::Matrix<double, 7, 1> pose_des_q_; 

  // Variables for limits
  Eigen::Matrix<double,7,1> torque_limits_, dtorque_limits_;
  Eigen::Matrix<double,7,1> q_min_limits_, q_max_limits_, dq_limits_; 
  double perc_allowed_torque_limit_, perc_allowed_dtorque_limit_, perc_allowed_q_limit_, perc_allowed_dq_limit_; 

  //Publishers
  realtime_tools::RealtimePublisher<geometry_msgs::PoseStamped> tau_d_pub_;
  realtime_tools::RealtimePublisher<geometry_msgs::PoseStamped> q_error_pub_;

  realtime_tools::RealtimePublisher<geometry_msgs::PoseStamped> gravity_pub_; 
  realtime_tools::RealtimePublisher<std_msgs::Float64> time_pub_;

  ///< Target pose subscriber
  ros::Subscriber sub_target_pose_;

  /**
   * Saturates torque commands to ensure feasibility.
   *
   * @param[in] arm_data The data container of the arm.
   * @param[in] tau_d_calculated The raw command according to the control law.
   * @param[in] tau_J_d The current desired torque, read from the robot state.
   * @return The saturated torque commmand for the 7 joints of one arm.
   */
  Eigen::Matrix<double, 7, 1> saturateTorqueRate( const CustomFrankaDataContainerKthJointAR& arm_data, const Eigen::Matrix<double, 7, 1>& tau_d_calculated,
       const Eigen::Matrix<double, 7, 1>& tau_J_d);  

  /**
   * Initializes a single Panda robot arm.
   *
   * @param[in] robot_hw A pointer the RobotHW class for getting interfaces and resource handles.
   * @param[in] arm_id The name of the panda arm.
   * @param[in] joint_names The names of all joints of the panda.
   * @return True if successfull, false otherwise.
   */
  bool initArm(hardware_interface::RobotHW* robot_hw, const std::string& arm_id, const std::vector<std::string>& joint_names,ros::NodeHandle& node_handle, int ind_robot);

  /**
   * Computes the decoupled controller update for a single arm.
   *
   * @param[in] arm_data The data container of the arm to control.
   */
  void updateArm(CustomFrankaDataContainerKthJointAR& arm_data, const string& arm_id, const ros::Duration& period);

  /**
   * Prepares all internal states to be ready to run the real-time control for one arm.
   *
   * @param[in] arm_data The data container of the arm to prepare for the control loop.
   */
  void startingArm(CustomFrankaDataContainerKthJointAR& arm_data);


  // Functions for robot limits
  bool checkTorqueLimits(const Eigen::Matrix<double, 7, 1>& tau,  const string& arm_id); 
  bool checkDerivativeTorqueLimits(const Eigen::Matrix<double, 7, 1>& dtau, const string& arm_id); 
  bool checkJointLimits(const Eigen::Matrix<double, 7, 1>& q,  const string& arm_id); 
  bool checkVelocityJointLimits(const Eigen::Matrix<double, 7, 1>& dq, const string& arm_id);


  // Functions for publishing
  void publishError(realtime_tools::RealtimePublisher<geometry_msgs::PoseStamped>& arm_error_pub, const Eigen::VectorXd& error, const ros::Time& timestamp); 
  void publishTau(realtime_tools::RealtimePublisher<geometry_msgs::PoseStamped>& arm_tau_pub, const Eigen::VectorXd& tau, const ros::Time& timestamp);   
  void publishTime(realtime_tools::RealtimePublisher<std_msgs::Float64>& time_pub, double time); 

  // Functions for callbacks
  void targetJointCallback(const geometry_msgs::PoseStamped::ConstPtr& msg, const string& robot_id); 

};

}  // namespace ar_puppeteer_franka
