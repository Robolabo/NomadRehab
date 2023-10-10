#include "nomad_hardware_interface/nomad_hardware_interface.hpp"

namespace nomad_hardware_interface
{
  using hardware_interface::CallbackReturn;
  using hardware_interface::SystemInterface;
  using hardware_interface::return_type;

  NomadHardware::NomadHardware() :
    logger_(rclcpp::get_logger("nomad_hw_if")),
    emulated_mode_(false)
  {}

  CallbackReturn 
  NomadHardware::on_init(const hardware_interface::HardwareInfo & info)
  {
    if (SystemInterface::on_init(info) != CallbackReturn::SUCCESS) {
      return CallbackReturn::ERROR;
    }
    /* Fetch paramters */
    serial_port_ = info_.hardware_parameters["serial_port"];
    baudrate_ = std::stoi(info_.hardware_parameters["baudrate"]);

    if ((serial_port_.size() == 0) || (baudrate_ == 0)) {
      RCLCPP_INFO(logger_, "Invalid serial port configuration!");
      return CallbackReturn::ERROR;
    }

    emulated_mode_ = std::stoi(info_.hardware_parameters["emulate"]);

    RCLCPP_INFO(logger_, "Input parameters:");
    for (auto & param : info_.hardware_parameters) {
      RCLCPP_INFO(logger_, "  key: %s, value: %s", param.first.c_str(), param.second.c_str());
    }
    
    auto get_joint_name = [&](std::string base_name) {
      auto parameter_name = base_name + "_joint_name";
      std::string joint_name = info_.hardware_parameters[parameter_name];
      auto joint = std::find_if(info_.joints.begin(),  info_.joints.end(), 
        [joint_name](auto & i){return i.name == joint_name;});
      if (joint == info_.joints.end()) {
        RCLCPP_ERROR(logger_, "Join %s does not exist!", parameter_name.c_str());
        return std::string("");
      }
      return joint_name;
    };

    auto load_parameters = [&](std::string base_name, auto & data) {
      data.name = base_name;
      data.kp = std::stof(info_.hardware_parameters[base_name + "_kp"]);
      data.kd = std::stof(info_.hardware_parameters[base_name + "_kd"]);
      data.ki = std::stof(info_.hardware_parameters[base_name + "_ki"]);
      data.limit = std::stof(info_.hardware_parameters[base_name + "_limit"]);
      data.cpr = std::stoi(info_.hardware_parameters[base_name + "_cpr"]);
      data.reduction = std::stof(info_.hardware_parameters[base_name + "_reduction"]);
    };

    /* get base parameters */
    load_parameters("base", motors_[BASE_INDEX]);
    base_joint_ = get_joint_name("base");

    /* get wheel traction joints and parameter*/
    load_parameters("velocity", motors_[VEL_INDEX]);
    wheel_joints_.push_back(get_joint_name("velocity"));
    wheel_joints_.push_back(get_joint_name("l_wheel_vel"));
    wheel_joints_.push_back(get_joint_name("r_wheel_vel"));

    /* get wheel steering joints and parameter*/
    load_parameters("steering", motors_[STEERING_INDEX]);
    steering_joints_.push_back(get_joint_name("steering"));
    steering_joints_.push_back(get_joint_name("l_wheel_steering"));
    steering_joints_.push_back(get_joint_name("r_wheel_steering"));

    return CallbackReturn::SUCCESS;
  }

  hardware_interface::CallbackReturn 
  NomadHardware::on_configure(const rclcpp_lifecycle::State & previous_state)
  {
    RCL_UNUSED(previous_state);
    /* Configure communications */
    RCLCPP_INFO(logger_, "Serial port: %s, baudrate: %d", serial_port_.c_str(), baudrate_);
    if (!emulated_mode_) {
      serial_ = std::make_shared<nomad_robot::NomadComm>(serial_port_, baudrate_, 1000U);
    }

    for (int index = 0; index < END_OF_JOINTS; index++) {
      RCLCPP_INFO(logger_, "Setting PID for motor %s -> kp: %lf, Kd: %lf, ki: %lf, lim: %lf, cpr: %d, red: %lf", 
        motors_[index].name.c_str(),
        motors_[index].kp, 
        motors_[index].kd, 
        motors_[index].ki, 
        motors_[index].limit,
        motors_[index].cpr,
        motors_[index].reduction);

      if (!emulated_mode_) {
        serial_->set_pid(
          motors_[index].name.substr(0,1),
          motors_[index].kp, 
          motors_[index].kd, 
          motors_[index].ki, 
          motors_[index].limit,
          motors_[index].cpr,
          motors_[index].reduction);
      }
    }   
    return CallbackReturn::SUCCESS;
  }

  hardware_interface::CallbackReturn
  NomadHardware::on_activate(const rclcpp_lifecycle::State & previous_state)
  {
    RCL_UNUSED(previous_state);

    wheel_command_ = 0.0f;
    wheel_vel_state_ = 0.0f;
    wheel_pos_state_ = 0.0f;
    steering_command_ = 0.0f;
    steering_pos_state_ = 0.0f;
    base_command_ = 0.0f;
    base_pos_state_ = 0.0f;

    return CallbackReturn::SUCCESS;
  }

  hardware_interface::CallbackReturn
  NomadHardware::on_deactivate(const rclcpp_lifecycle::State & previous_state)
  {
    RCL_UNUSED(previous_state);
    return CallbackReturn::SUCCESS;
  }

  hardware_interface::return_type 
  NomadHardware::read(const rclcpp::Time & time, const rclcpp::Duration & period)
  {
    RCL_UNUSED(time);

    if (emulated_mode_) {
      wheel_vel_state_ = wheel_command_;
      steering_pos_state_ = steering_command_;
      base_pos_state_ = base_command_;
    } else {
      serial_->get_state(
        wheel_vel_state_, 
        steering_pos_state_,  
        base_pos_state_);
    }

    wheel_pos_state_ += (wheel_vel_state_*period.seconds());
    return return_type::OK;
  }

  hardware_interface::return_type 
  NomadHardware::write(const rclcpp::Time & time, const rclcpp::Duration & period)
  {
    RCL_UNUSED(time);
    RCL_UNUSED(period);

    if (!emulated_mode_) {
      serial_->set_state(
        wheel_command_, 
        steering_command_,
        base_command_);
    }
    return return_type::OK;
  }

  std::vector<hardware_interface::StateInterface>
  NomadHardware::export_state_interfaces()
  {

    std::vector<hardware_interface::StateInterface> state_interfaces;
    
    /* All wheel  and steering joints have two states: position and velocity */
    /* Since all of them move synchronuslly, they share the same state*/
    for (auto joint_name : wheel_joints_) {
      RCLCPP_INFO(logger_, "Adding position state interface: %s", joint_name.c_str());
      state_interfaces.emplace_back(
        hardware_interface::StateInterface(
          joint_name, 
          hardware_interface::HW_IF_POSITION, 
          &wheel_pos_state_));

      RCLCPP_INFO(logger_, "Adding velocity state interface: %s", joint_name.c_str());
      state_interfaces.emplace_back(
        hardware_interface::StateInterface(
          joint_name, 
          hardware_interface::HW_IF_VELOCITY, 
          &wheel_vel_state_));
    }

    for (auto joint_name : steering_joints_) {
      RCLCPP_INFO(logger_, "Adding position state interface: %s", joint_name.c_str());
      /* Get state interfaces */
      state_interfaces.emplace_back(
        hardware_interface::StateInterface(
          joint_name, 
          hardware_interface::HW_IF_POSITION, 
          &steering_pos_state_));
    }

    RCLCPP_INFO(logger_, "Adding position state interface: %s", base_joint_.c_str());
    /* Get state interfaces */
    state_interfaces.emplace_back(
      hardware_interface::StateInterface(
        base_joint_, 
        hardware_interface::HW_IF_POSITION, 
        &base_pos_state_));
  
    return state_interfaces;
  }

  std::vector<hardware_interface::CommandInterface>
  NomadHardware::export_command_interfaces()
  {
    std::vector<hardware_interface::CommandInterface> state_interfaces;

    RCLCPP_INFO(logger_, "Adding velocity command interface!!!: %s", wheel_joints_[0].c_str());
    state_interfaces.emplace_back(
      hardware_interface::CommandInterface(
        wheel_joints_[0].c_str(), 
        hardware_interface::HW_IF_VELOCITY, 
        &wheel_command_));

    RCLCPP_INFO(logger_, "Adding position command interface: %s", steering_joints_[0].c_str());
    state_interfaces.emplace_back(
    hardware_interface::CommandInterface(
      steering_joints_[0].c_str(), 
      hardware_interface::HW_IF_POSITION, 
      &steering_command_)); 

    RCLCPP_INFO(logger_, "Adding position command interface: %s", base_joint_.c_str());
    state_interfaces.emplace_back(
      hardware_interface::CommandInterface(
        base_joint_.c_str(), 
        hardware_interface::HW_IF_POSITION, 
        &base_command_)); 

    return state_interfaces;
  }
}

#include "pluginlib/class_list_macros.hpp"


PLUGINLIB_EXPORT_CLASS(
  nomad_hardware_interface::NomadHardware,
  hardware_interface::SystemInterface
)