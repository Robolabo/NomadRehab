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

#include <memory>
#include <queue>
#include <string>
#include <utility>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "rclcpp/logging.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "synchro_drive_controller/synchro_drive_controller.hpp"

namespace
{
constexpr auto DEFAULT_COMMAND_TOPIC = "~/cmd_vel";
constexpr auto DEFAULT_ACKERMANN_OUT_TOPIC = "~/cmd_ackermann";
constexpr auto DEFAULT_ODOMETRY_TOPIC = "~/odom";
constexpr auto DEFAULT_TRANSFORM_TOPIC = "/tf";
constexpr auto DEFAULT_RESET_ODOM_SERVICE = "~/reset_odometry";
}  // namespace

namespace synchro_drive_controller
{
using namespace std::chrono_literals;
using controller_interface::interface_configuration_type;
using controller_interface::InterfaceConfiguration;
using hardware_interface::HW_IF_POSITION;
using hardware_interface::HW_IF_VELOCITY;
using lifecycle_msgs::msg::State;

SychroController::SychroController() : controller_interface::ControllerInterface() {}

CallbackReturn SychroController::on_init()
{
  try {
    /* with the lifecycle node being initialized, we can declare parameters */
    auto_declare<std::string>("traction_joint_name", std::string());
    auto_declare<std::string>("steering_joint_name", std::string());

    auto_declare<double>("wheel_radius", wheel_radius_);

    auto_declare<std::string>("odom_frame_id", odom_params_.odom_frame_id);
    auto_declare<std::string>("base_frame_id", odom_params_.base_frame_id);

    auto_declare<std::vector<double>>("pose_covariance_diagonal", std::vector<double>());
    auto_declare<std::vector<double>>("twist_covariance_diagonal", std::vector<double>());
    auto_declare<bool>("open_loop", odom_params_.open_loop);
    auto_declare<bool>("enable_odom_tf", odom_params_.enable_odom_tf);
    auto_declare<bool>("odom_only_twist", odom_params_.odom_only_twist);

    auto_declare<int>("cmd_vel_timeout", static_cast<int>(cmd_vel_timeout_.count()));
    auto_declare<bool>("publish_ackermann_command", publish_ackermann_command_);
    auto_declare<int>("velocity_rolling_window_size", 10);
    auto_declare<bool>("use_stamped_vel", use_stamped_vel_);

    auto_declare<double>("traction.max_velocity", NAN);
    auto_declare<double>("traction.min_velocity", NAN);
    auto_declare<double>("traction.max_acceleration", NAN);
    auto_declare<double>("traction.min_acceleration", NAN);
    auto_declare<double>("traction.max_deceleration", NAN);
    auto_declare<double>("traction.min_deceleration", NAN);
    auto_declare<double>("traction.max_jerk", NAN);
    auto_declare<double>("traction.min_jerk", NAN);

    auto_declare<double>("steering.max_position", NAN);
    auto_declare<double>("steering.min_position", NAN);
    auto_declare<double>("steering.max_velocity", NAN);
    auto_declare<double>("steering.min_velocity", NAN);
    auto_declare<double>("steering.max_acceleration", NAN);
    auto_declare<double>("steering.min_acceleration", NAN);
  }
  catch (const std::exception & e) {
    RCLCPP_ERROR(get_node()->get_logger(), "Exception thrown during init stage with message: %s \n", e.what());
    return CallbackReturn::ERROR;
  }
  return CallbackReturn::SUCCESS;
}

InterfaceConfiguration SychroController::command_interface_configuration() const
{
  InterfaceConfiguration command_interfaces_config;
  command_interfaces_config.type = interface_configuration_type::INDIVIDUAL;

  /* Configure front wheel commands */
  command_interfaces_config.names.push_back(traction_joint_name_ + "/" + HW_IF_VELOCITY);
  command_interfaces_config.names.push_back(steering_joint_name_ + "/" + HW_IF_POSITION);

  return command_interfaces_config;
}

InterfaceConfiguration SychroController::state_interface_configuration() const
{
  InterfaceConfiguration state_interfaces_config;
  state_interfaces_config.type = interface_configuration_type::INDIVIDUAL;
  /* Define only two state interface for all */
  /* The three wheels are supposed to move synchronuslly*/
  state_interfaces_config.names.push_back(traction_joint_name_ + "/" + HW_IF_VELOCITY);
  state_interfaces_config.names.push_back(steering_joint_name_ + "/" + HW_IF_POSITION);
  return state_interfaces_config;
}

controller_interface::return_type SychroController::update(
  const rclcpp::Time & time, const rclcpp::Duration & period)
{

  if (get_state().id() == State::PRIMARY_STATE_INACTIVE) {
    /* Halt if the controller is not active */
    traction_joint_[0].command.get().set_value(0.0);
    steering_joint_[0].command.get().set_value(0.0);
    return controller_interface::return_type::OK;
  }

  std::shared_ptr<TwistStamped> last_command_msg;
  received_velocity_msg_ptr_.get(last_command_msg);

  if (last_command_msg == nullptr) {
    RCLCPP_WARN(get_node()->get_logger(), "Velocity message received was a nullptr.");
    return controller_interface::return_type::ERROR;
  }

  const auto age_of_last_command = time - last_command_msg->header.stamp;
  /* Brake if cmd_vel has timeout, override the stored command */
  if (age_of_last_command > cmd_vel_timeout_) {
    last_command_msg->twist.linear.x = 0.0;
    last_command_msg->twist.angular.z = 0.0;
  }

  /* Get linear velocity and angular position commands */
  double linear_command = last_command_msg->twist.linear.x;
  double alpha_command = last_command_msg->twist.angular.z;
  /* Get data from the hardware interface */
  double ws_read = traction_joint_[0].state.get().get_value();     // in radians/s
  double alpha_read = steering_joint_[0].state.get().get_value();  // in radians

  /* Check read data */
  if (std::isnan(ws_read) || std::isnan(alpha_read)) {
    RCLCPP_ERROR(get_node()->get_logger(), "Could not read feedback value");
    return controller_interface::return_type::ERROR;
  }

  odometry_.update(ws_read, alpha_read, period);

  /* publish odometry */
  if (realtime_odometry_publisher_->trylock()) {
    /* Orientation is not set since the robot does not have */
    /* angular translation*/
    auto & odometry_message = realtime_odometry_publisher_->msg_;
    odometry_message.header.stamp = time;
    odometry_message.twist.twist.linear.x = odometry_.getLinear();
    odometry_message.twist.twist.angular.z = odometry_.getAngular();
    realtime_odometry_publisher_->unlockAndPublish();
  }

  /* Publish transform */
  if (odom_params_.enable_odom_tf && realtime_odometry_transform_publisher_->trylock()) {
    /* Orientation is not set since the robot does not have */
    /* angular translation*/
    auto & transform = realtime_odometry_transform_publisher_->msg_.transforms.front();
    transform.header.stamp = time;
    transform.transform.translation.x = odometry_.getX();
    transform.transform.translation.y = odometry_.getY();
    realtime_odometry_transform_publisher_->unlockAndPublish();
  }

  /* Compute wheel velocity */
  auto ws_command = linear_command / wheel_radius_;

  /* Reduce wheel speed until the target angle has been reached */
  double alpha_delta = abs(alpha_command - alpha_read);
  double scale;
  if (alpha_delta < M_PI / 6) {
    scale = 1;
  }
  else if (alpha_delta > M_PI_2) {
    scale = 0.01;
  }
  else {
    /* TODO(anyone): find the best function, e.g convex power functions */
    scale = cos(alpha_delta);
  }
  ws_command *= scale;

  /* Limit the output according to the configuration */
  if (previous_commands_.size() >= 2) {
    auto & last_command = previous_commands_.back();
    auto & second_to_last_command = previous_commands_.front();

    limiter_traction_.limit(
      ws_command, 
      last_command->speed, 
      second_to_last_command->speed, 
      period.seconds());

    limiter_steering_.limit(
      alpha_command, 
      last_command->steering_angle, 
      second_to_last_command->steering_angle,
      period.seconds());
    previous_commands_.pop();
  }

  auto ackermann_command = std::make_shared<AckermannDrive>();
  /* speed in AckermannDrive is defined as desired forward speed (m/s) but it is used here as wheel
    speed (rad/s) */
  ackermann_command->speed = static_cast<float>(ws_command);
  ackermann_command->steering_angle = static_cast<float>(alpha_command);
  previous_commands_.emplace(ackermann_command);

  /* Publish ackermann command */
  if (publish_ackermann_command_ && realtime_ackermann_command_publisher_->trylock()) {
    auto & realtime_ackermann_command = realtime_ackermann_command_publisher_->msg_;
    /* speed in AckermannDrive is defined as desired forward speed (m/s) but it is used here as wheel
      speed (rad/s) */
    realtime_ackermann_command.speed = ackermann_command->speed;
    realtime_ackermann_command.steering_angle = ackermann_command->steering_angle;
    realtime_ackermann_command_publisher_->unlockAndPublish();
  }

  /* Set command values */
  traction_joint_[0].command.get().set_value(ws_command);
  steering_joint_[0].command.get().set_value(alpha_command);

  return controller_interface::return_type::OK;
}

CallbackReturn SychroController::on_configure(const rclcpp_lifecycle::State & /*previous_state*/)
{
  auto logger = get_node()->get_logger();

  /* Get joint names*/
  traction_joint_name_ = get_node()->get_parameter("traction_joint_name").as_string();
  steering_joint_name_ = get_node()->get_parameter("steering_joint_name").as_string();


  if (traction_joint_name_.empty()) {
    RCLCPP_ERROR(logger, "'traction_joint_name' parameter was empty");
    return CallbackReturn::ERROR;
  }
  if (steering_joint_name_.empty()) {
    RCLCPP_ERROR(logger, "'steering_joint_name_' parameter was empty");
    return CallbackReturn::ERROR;
  }

  wheel_radius_ = get_node()->get_parameter("wheel_radius").as_double();

  odometry_.setWheelParams(wheel_radius_);
  odometry_.setVelocityRollingWindowSize(
    get_node()->get_parameter("velocity_rolling_window_size").as_int());

  odom_params_.odom_frame_id = get_node()->get_parameter("odom_frame_id").as_string();
  odom_params_.base_frame_id = get_node()->get_parameter("base_frame_id").as_string();

  auto pose_diagonal = get_node()->get_parameter("pose_covariance_diagonal").as_double_array();
  std::copy(
    pose_diagonal.begin(), pose_diagonal.end(), odom_params_.pose_covariance_diagonal.begin());

  auto twist_diagonal = get_node()->get_parameter("twist_covariance_diagonal").as_double_array();
  std::copy(
    twist_diagonal.begin(), twist_diagonal.end(), odom_params_.twist_covariance_diagonal.begin());

  odom_params_.open_loop = get_node()->get_parameter("open_loop").as_bool();
  odom_params_.enable_odom_tf = get_node()->get_parameter("enable_odom_tf").as_bool();
  odom_params_.odom_only_twist = get_node()->get_parameter("odom_only_twist").as_bool();

  cmd_vel_timeout_ =
    std::chrono::milliseconds{get_node()->get_parameter("cmd_vel_timeout").as_int()};
  publish_ackermann_command_ = get_node()->get_parameter("publish_ackermann_command").as_bool();
  use_stamped_vel_ = get_node()->get_parameter("use_stamped_vel").as_bool();

  try {
    limiter_traction_ = TractionLimiter(
      get_node()->get_parameter("traction.min_velocity").as_double(),
      get_node()->get_parameter("traction.max_velocity").as_double(),
      get_node()->get_parameter("traction.min_acceleration").as_double(),
      get_node()->get_parameter("traction.max_acceleration").as_double(),
      get_node()->get_parameter("traction.min_deceleration").as_double(),
      get_node()->get_parameter("traction.max_deceleration").as_double(),
      get_node()->get_parameter("traction.min_jerk").as_double(),
      get_node()->get_parameter("traction.max_jerk").as_double());
  }
  catch (const std::invalid_argument & e) {
    RCLCPP_ERROR(logger, "Error configuring traction limiter: %s", e.what());
    return CallbackReturn::ERROR;
  }

  try {
    limiter_steering_ = SteeringLimiter(
      get_node()->get_parameter("steering.min_position").as_double(),
      get_node()->get_parameter("steering.max_position").as_double(),
      get_node()->get_parameter("steering.min_velocity").as_double(),
      get_node()->get_parameter("steering.max_velocity").as_double(),
      get_node()->get_parameter("steering.min_acceleration").as_double(),
      get_node()->get_parameter("steering.max_acceleration").as_double());
  }
  catch (const std::invalid_argument & e) {
    RCLCPP_ERROR(logger, "Error configuring steering limiter: %s", e.what());
    return CallbackReturn::ERROR;
  }

  if (!reset()) {
    return CallbackReturn::ERROR;
  }
  received_velocity_msg_ptr_.set(std::make_shared<TwistStamped>());

  /* initialize ackermann command publisher */
  if (publish_ackermann_command_) {
    ackermann_command_publisher_ = get_node()->create_publisher<AckermannDrive>(
      DEFAULT_ACKERMANN_OUT_TOPIC, rclcpp::SystemDefaultsQoS());
    realtime_ackermann_command_publisher_ =
      std::make_shared<realtime_tools::RealtimePublisher<AckermannDrive>>(
        ackermann_command_publisher_);
  }

  /* initialize command subscriber */
  if (use_stamped_vel_) {
    velocity_command_subscriber_ = get_node()->create_subscription<TwistStamped>(
      DEFAULT_COMMAND_TOPIC, rclcpp::SystemDefaultsQoS(),
      [this](const std::shared_ptr<TwistStamped> msg) -> void
      {
        if (!subscriber_is_active_) {
          RCLCPP_WARN(get_node()->get_logger(), "Can't accept new commands. subscriber is inactive");
          return;
        }
        if ((msg->header.stamp.sec == 0) && (msg->header.stamp.nanosec == 0)) {
          RCLCPP_WARN_ONCE(
            get_node()->get_logger(),
            "Received TwistStamped with zero timestamp, setting it to current "
            "time, this message will only be shown once");
          msg->header.stamp = get_node()->get_clock()->now();
        }
        received_velocity_msg_ptr_.set(std::move(msg));
      });
  }
  else {
    /* TODO(alejo2313): Verify this */
    velocity_command_unstamped_subscriber_ = get_node()->create_subscription<Twist>(
      DEFAULT_COMMAND_TOPIC, rclcpp::SystemDefaultsQoS(),
      [this](const std::shared_ptr<Twist> msg) -> void
      {
        if (!subscriber_is_active_) {
          RCLCPP_WARN(get_node()->get_logger(), "Can't accept new commands. subscriber is inactive");
          return;
        }
        /* Write fake header in the stored stamped command */
        std::shared_ptr<TwistStamped> twist_stamped;
        received_velocity_msg_ptr_.get(twist_stamped);
        twist_stamped->twist = *msg;
        twist_stamped->header.stamp = get_node()->get_clock()->now();
      });
  }

  /* initialize odometry publisher and messasge */
  odometry_publisher_ = get_node()->create_publisher<nav_msgs::msg::Odometry>(
    DEFAULT_ODOMETRY_TOPIC, rclcpp::SystemDefaultsQoS());
  realtime_odometry_publisher_ =
    std::make_shared<realtime_tools::RealtimePublisher<nav_msgs::msg::Odometry>>(
      odometry_publisher_);

  auto & odometry_message = realtime_odometry_publisher_->msg_;
  odometry_message.header.frame_id = odom_params_.odom_frame_id;
  odometry_message.child_frame_id = odom_params_.base_frame_id;

  /* initialize odom values zeros */
  odometry_message.twist =
    geometry_msgs::msg::TwistWithCovariance(rosidl_runtime_cpp::MessageInitialization::ALL);


  /* TODO(alejo2313): Verify this */
  constexpr size_t NUM_DIMENSIONS = 6;
  for (size_t index = 0; index < NUM_DIMENSIONS; ++index) {
    // 0, 7, 14, 21, 28, 35
    const size_t diagonal_index = NUM_DIMENSIONS * index + index;
    odometry_message.pose.covariance[diagonal_index] = odom_params_.pose_covariance_diagonal[index];
    odometry_message.twist.covariance[diagonal_index] =
      odom_params_.twist_covariance_diagonal[index];
  }

  /* initialize transform publisher and message */
  if (odom_params_.enable_odom_tf) {
    odometry_transform_publisher_ = get_node()->create_publisher<tf2_msgs::msg::TFMessage>(
      DEFAULT_TRANSFORM_TOPIC, rclcpp::SystemDefaultsQoS());
    realtime_odometry_transform_publisher_ =
      std::make_shared<realtime_tools::RealtimePublisher<tf2_msgs::msg::TFMessage>>(
        odometry_transform_publisher_);

    /* keeping track of odom and base_link transforms only */
    auto & odometry_transform_message = realtime_odometry_transform_publisher_->msg_;
    odometry_transform_message.transforms.resize(1);
    odometry_transform_message.transforms.front().header.frame_id = odom_params_.odom_frame_id;
    odometry_transform_message.transforms.front().child_frame_id = odom_params_.base_frame_id;
  }

  /* Create odom reset service */
  reset_odom_service_ = get_node()->create_service<std_srvs::srv::Empty>(
    DEFAULT_RESET_ODOM_SERVICE, 
    std::bind(
      &SychroController::reset_odometry, this, std::placeholders::_1,
      std::placeholders::_2, std::placeholders::_3));

  return CallbackReturn::SUCCESS;
}

CallbackReturn SychroController::on_activate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_node()->get_logger(), "On activate: Initialize Joints");

  /* Initialize the joints */
  auto traction_result = get_traction(traction_joint_name_, traction_joint_);
  auto steering_result = get_steering(steering_joint_name_, steering_joint_);

  if (traction_result == CallbackReturn::ERROR || steering_result == CallbackReturn::ERROR) {
    return CallbackReturn::ERROR;
  }

  subscriber_is_active_ = true;

  RCLCPP_DEBUG(get_node()->get_logger(), "Subscriber and publisher are now active.");
  return CallbackReturn::SUCCESS;
}

CallbackReturn SychroController::on_deactivate(const rclcpp_lifecycle::State &)
{
  subscriber_is_active_ = false;
  return CallbackReturn::SUCCESS;
}

CallbackReturn SychroController::on_cleanup(const rclcpp_lifecycle::State &)
{
  if (!reset()) {
    return CallbackReturn::ERROR;
  }

  received_velocity_msg_ptr_.set(std::make_shared<TwistStamped>());
  return CallbackReturn::SUCCESS;
}

CallbackReturn SychroController::on_error(const rclcpp_lifecycle::State &)
{
  if (!reset()) {
    return CallbackReturn::ERROR;
  }
  return CallbackReturn::SUCCESS;
}

void SychroController::reset_odometry(
  const std::shared_ptr<rmw_request_id_t> /*request_header*/,
  const std::shared_ptr<std_srvs::srv::Empty::Request> /*req*/,
  std::shared_ptr<std_srvs::srv::Empty::Response> /*res*/)
{
  odometry_.resetOdometry();
  RCLCPP_INFO(get_node()->get_logger(), "Odometry successfully reset");
}

bool SychroController::reset() {
  odometry_.resetOdometry();

  /* release the old queue */
  std::queue<AckermannDrive::SharedPtr> empty_ackermann_drive;
  std::swap(previous_commands_, empty_ackermann_drive);

  traction_joint_.clear();
  steering_joint_.clear();

  subscriber_is_active_ = false;
  velocity_command_subscriber_.reset();
  velocity_command_unstamped_subscriber_.reset();

  received_velocity_msg_ptr_.set(nullptr);
  return true;
}


CallbackReturn SychroController::on_shutdown(const rclcpp_lifecycle::State &)
{
  return CallbackReturn::SUCCESS;
}

CallbackReturn SychroController::get_traction(
  const std::string & traction_joint_name, std::vector<JointHandle> & joint)
{
  RCLCPP_INFO(get_node()->get_logger(), "Get Wheel Joint Instance");

  // Lookup the velocity state interface
  const auto state_handle = std::find_if(
    state_interfaces_.cbegin(), state_interfaces_.cend(),
    [&traction_joint_name](const auto & interface)
    {
      return interface.get_prefix_name() == traction_joint_name &&
             interface.get_interface_name() == HW_IF_VELOCITY;
    });
  if (state_handle == state_interfaces_.cend()) {
    /* Some joints may not have state interfaces */
    RCLCPP_INFO(
      get_node()->get_logger(), "Unable to obtain joint state handle for %s",
      traction_joint_name.c_str());
  }

  /* Lookup the velocity command interface */
  const auto command_handle = std::find_if(
    command_interfaces_.begin(), command_interfaces_.end(),
    [&traction_joint_name](const auto & interface)
    {
      return interface.get_prefix_name() == traction_joint_name &&
             interface.get_interface_name() == HW_IF_VELOCITY;
    });
  if (command_handle == command_interfaces_.end()) {
    RCLCPP_ERROR(
      get_node()->get_logger(), "Unable to obtain joint state handle for %s",
      traction_joint_name.c_str());
    return CallbackReturn::ERROR;
  }

  /* Create the traction joint instance */
  joint.emplace_back(JointHandle{std::ref(*state_handle), std::ref(*command_handle)});
  return CallbackReturn::SUCCESS;
}

CallbackReturn SychroController::get_steering(
  const std::string & steering_joint_name, std::vector<JointHandle> & joint)
{
  RCLCPP_INFO(get_node()->get_logger(), "Get Steering Joint Instance");

  // Lookup the velocity state interface
  const auto state_handle = std::find_if(
    state_interfaces_.cbegin(), state_interfaces_.cend(),
    [&steering_joint_name](const auto & interface)
    {
      return interface.get_prefix_name() == steering_joint_name &&
             interface.get_interface_name() == HW_IF_POSITION;
    });

  
  if (state_handle == state_interfaces_.cend()) {
    /* Some joints may not have state interfaces */
    RCLCPP_INFO(
      get_node()->get_logger(), "Unable to obtain joint state handle for %s",
      steering_joint_name.c_str());
  }

  /* Lookup the velocity command interface */
  const auto command_handle = std::find_if(
    command_interfaces_.begin(), command_interfaces_.end(),
    [&steering_joint_name](const auto & interface)
    {
      return interface.get_prefix_name() == steering_joint_name &&
             interface.get_interface_name() == HW_IF_POSITION;
    });
  if (command_handle == command_interfaces_.end()) {
    RCLCPP_ERROR(
      get_node()->get_logger(), "Unable to obtain joint state handle for %s",
      steering_joint_name.c_str());
    return CallbackReturn::ERROR;
  }

  /* Create the steering joint instance */
  joint.emplace_back(JointHandle{std::ref(*state_handle), std::ref(*command_handle)});
  return CallbackReturn::SUCCESS;
}

}  // namespace synchro_drive_controller

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(
  synchro_drive_controller::SychroController, controller_interface::ControllerInterface)