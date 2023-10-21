// Copyright 2022 Pixel Robotics.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

/*
 * Author: Tony Najjar
 */

#ifndef SYNCHRO_DRIVE_CONTROLLER__SYNCHRO_DRIVE_CONTROLLER_HPP_
#define SYNCHRO_DRIVE_CONTROLLER__SYNCHRO_DRIVE_CONTROLLER_HPP_

#include <chrono>
#include <cmath>
#include <memory>
#include <queue>
#include <string>
#include <tuple>
#include <vector>

#include "ackermann_msgs/msg/ackermann_drive.hpp"
#include "controller_interface/controller_interface.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "hardware_interface/handle.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "realtime_tools/realtime_box.h"
#include "realtime_tools/realtime_buffer.h"
#include "realtime_tools/realtime_publisher.h"
#include "std_srvs/srv/empty.hpp"
#include "tf2_msgs/msg/tf_message.hpp"
#include "synchro_drive_controller/odometry.hpp"
#include "synchro_drive_controller/steering_limiter.hpp"
#include "synchro_drive_controller/traction_limiter.hpp"
#include "synchro_drive_controller/visibility_control.h"

namespace synchro_drive_controller
{
using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

class SychroController : public controller_interface::ControllerInterface
{
  using Twist = geometry_msgs::msg::Twist;
  using TwistStamped = geometry_msgs::msg::TwistStamped;
  using AckermannDrive = ackermann_msgs::msg::AckermannDrive;

public:
  SYNCHRO_DRIVE_CONTROLLER_PUBLIC
  SychroController();

  SYNCHRO_DRIVE_CONTROLLER_PUBLIC
  controller_interface::InterfaceConfiguration command_interface_configuration() const override;

  SYNCHRO_DRIVE_CONTROLLER_PUBLIC
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  SYNCHRO_DRIVE_CONTROLLER_PUBLIC
  controller_interface::return_type update(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  SYNCHRO_DRIVE_CONTROLLER_PUBLIC
  CallbackReturn on_init() override;

  SYNCHRO_DRIVE_CONTROLLER_PUBLIC
  CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;

  SYNCHRO_DRIVE_CONTROLLER_PUBLIC
  CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;

  SYNCHRO_DRIVE_CONTROLLER_PUBLIC
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

  SYNCHRO_DRIVE_CONTROLLER_PUBLIC
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State & previous_state) override;

  SYNCHRO_DRIVE_CONTROLLER_PUBLIC
  CallbackReturn on_error(const rclcpp_lifecycle::State & previous_state) override;

  SYNCHRO_DRIVE_CONTROLLER_PUBLIC
  CallbackReturn on_shutdown(const rclcpp_lifecycle::State & previous_state) override;

protected:
  struct JointHandle
  {
    std::reference_wrapper<const hardware_interface::LoanedStateInterface> state;
    std::reference_wrapper<hardware_interface::LoanedCommandInterface> command;
  };

  struct OdometryParams
  {
    bool open_loop = false;
    bool enable_odom_tf = false;
    bool odom_only_twist = false;  // for doing the pose integration in separate node
    std::string base_frame_id = "base_link";
    std::string odom_frame_id = "odom";
    std::array<double, 6> pose_covariance_diagonal;
    std::array<double, 6> twist_covariance_diagonal;
  } odom_params_;


  CallbackReturn get_traction(
    const std::string & traction_joint_name, std::vector<JointHandle> & joint);

  CallbackReturn get_steering(
    const std::string & steering_joint_name, std::vector<JointHandle> & joint);

  void reset_odometry(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<std_srvs::srv::Empty::Request> req,
    std::shared_ptr<std_srvs::srv::Empty::Response> res);

  bool reset();


  double wheel_radius_ = 0.0;
  std::chrono::milliseconds cmd_vel_timeout_{500}; // Timeout to consider cmd_vel commands old
  
  /* speed limiters */
  TractionLimiter limiter_traction_;
  SteeringLimiter limiter_steering_;
  /* Joints */
  std::string traction_joint_name_;
  std::string steering_joint_name_;
  std::vector<JointHandle> traction_joint_; // HACK: put into vector to avoid initializing structs because they have no default constructors
  std::vector<JointHandle> steering_joint_;
  /* Storage */
  std::queue<AckermannDrive::SharedPtr> 
    previous_commands_;
  realtime_tools::RealtimeBox<std::shared_ptr<TwistStamped>> 
    received_velocity_msg_ptr_;
  /* Odom */
  Odometry odometry_;

  bool publish_ackermann_command_ = false;
  bool subscriber_is_active_ = false;
  bool use_stamped_vel_ = true;
  
  /* Pub, subs and srv*/
  rclcpp::Publisher<AckermannDrive>::SharedPtr 
    ackermann_command_publisher_ = nullptr;
  std::shared_ptr<realtime_tools::RealtimePublisher<AckermannDrive>>
    realtime_ackermann_command_publisher_ = nullptr;

  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr 
    odometry_publisher_ = nullptr;
  std::shared_ptr<realtime_tools::RealtimePublisher<nav_msgs::msg::Odometry>>
    realtime_odometry_publisher_ = nullptr;

  rclcpp::Publisher<tf2_msgs::msg::TFMessage>::SharedPtr 
    odometry_transform_publisher_ = nullptr;
  std::shared_ptr<realtime_tools::RealtimePublisher<tf2_msgs::msg::TFMessage>>
    realtime_odometry_transform_publisher_ = nullptr;

  rclcpp::Subscription<TwistStamped>::SharedPtr 
    velocity_command_subscriber_ = nullptr;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr
    velocity_command_unstamped_subscriber_ = nullptr;

  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr 
    reset_odom_service_;
};
}  // namespace synchro_drive_controller
#endif  // SYNCHRO_DRIVE_CONTROLLER__SYNCHRO_DRIVE_CONTROLLER_HPP_
