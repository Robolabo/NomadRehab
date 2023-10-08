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

    motors_[BASE_INDEX].name = "base";
    motors_[BASE_INDEX].join_type = hardware_interface::HW_IF_POSITION;
    motors_[VEL_INDEX].name = "velocity";
    motors_[VEL_INDEX].join_type = hardware_interface::HW_IF_VELOCITY;
    motors_[STEERING_INDEX].name = "steering";
    motors_[STEERING_INDEX].join_type = hardware_interface::HW_IF_POSITION;

    motors_[LEFT_WHEEL_VEL].name = "l_wheel_vel";
    motors_[LEFT_WHEEL_VEL].join_type = hardware_interface::HW_IF_VELOCITY;
    motors_[LEFT_WHEEL_STEERING].name = "l_wheel_steering";
    motors_[LEFT_WHEEL_STEERING].join_type = hardware_interface::HW_IF_POSITION;

    motors_[RIGTH_WHEEL_VEL].name = "r_wheel_vel";
    motors_[RIGTH_WHEEL_VEL].join_type = hardware_interface::HW_IF_VELOCITY;
    motors_[RIGHT_WHEEL_STEERING].name = "r_wheel_steering";
    motors_[RIGHT_WHEEL_STEERING].join_type = hardware_interface::HW_IF_POSITION;

   
    RCLCPP_INFO(logger_, "Input parameters:");
    for (auto & param : info_.hardware_parameters) {
      RCLCPP_INFO(logger_, "  key: %s, value: %s", param.first.c_str(), param.second.c_str());
    }
     
    for (int index = 0; index < END_OF_JOINTS; index++) {
      if ( index < PURE_STATE_JOINTS ) {
        motors_[index].kp = std::stof(info_.hardware_parameters[motors_[index].name + "_kp"]);
        motors_[index].kd = std::stof(info_.hardware_parameters[motors_[index].name + "_kd"]);
        motors_[index].ki = std::stof(info_.hardware_parameters[motors_[index].name + "_ki"]);
        motors_[index].limit = std::stof(info_.hardware_parameters[motors_[index].name + "_limit"]);
        motors_[index].cpr = std::stoi(info_.hardware_parameters[motors_[index].name + "_cpr"]);
        motors_[index].reduction = std::stof(info_.hardware_parameters[motors_[index].name + "_reduction"]);
      }
      motors_[index].joint_name = info_.hardware_parameters[motors_[index].name + "_joint_name"];

      auto joint = std::find_if(info_.joints.begin(),  info_.joints.end(), 
        [&name = motors_[index].joint_name](auto & i){return i.name == name;});
      if (joint == info_.joints.end()) {
        RCLCPP_INFO(logger_, "Join %s does not exist!, index: %d", motors_[index].joint_name.c_str(), index);
        return CallbackReturn::ERROR;
      }
    }
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

    for (int index = 0; index < PURE_STATE_JOINTS; index++) {
      if ( index < PURE_STATE_JOINTS )
      {

        RCLCPP_INFO(logger_, "Setting PID for motor %s -> kp: %lf, Kd: %lf, ki: %lf, lim: %lf, cpr: %d, red: %lf", 
          motors_[index].joint_name.c_str(),
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
        motors_[index].command = 0.0f;
      }
      motors_[index].state = 0.0f;
    }
    return CallbackReturn::SUCCESS;
  }

  hardware_interface::CallbackReturn
  NomadHardware::on_activate(const rclcpp_lifecycle::State & previous_state)
  {
    RCL_UNUSED(previous_state);
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
    RCL_UNUSED(period);

    if (emulated_mode_) {
      motors_[BASE_INDEX].state = motors_[BASE_INDEX].command;
      motors_[VEL_INDEX].state = motors_[VEL_INDEX].command;
      motors_[STEERING_INDEX].state = motors_[STEERING_INDEX].command;
    } else {
      serial_->get_state(
        motors_[BASE_INDEX].state, 
        motors_[VEL_INDEX].state,  
        motors_[STEERING_INDEX].state);
    }

    motors_[LEFT_WHEEL_STEERING].state = motors_[STEERING_INDEX].state;
    motors_[LEFT_WHEEL_VEL].state = motors_[VEL_INDEX].state;
    motors_[RIGHT_WHEEL_STEERING].state = motors_[STEERING_INDEX].state;
    motors_[RIGTH_WHEEL_VEL].state = motors_[VEL_INDEX].state;

    return return_type::OK;
  }

  hardware_interface::return_type 
  NomadHardware::write(const rclcpp::Time & time, const rclcpp::Duration & period)
  {
    RCL_UNUSED(time);
    RCL_UNUSED(period);

    if (!emulated_mode_) {
      serial_->set_state(
        motors_[BASE_INDEX].command, 
        motors_[VEL_INDEX].command,  
        motors_[STEERING_INDEX].command);
    }
    return return_type::OK;
  }

  std::vector<hardware_interface::StateInterface>
  NomadHardware::export_state_interfaces()
  {
    std::vector<hardware_interface::StateInterface> state_interfaces;
    for (int index = 0; index < END_OF_JOINTS; index++) {
      RCLCPP_INFO(logger_, "Adding %s state interface: %s", motors_[index].join_type, motors_[index].joint_name.c_str());
      /* Get state interfaces */
      state_interfaces.emplace_back(
        hardware_interface::StateInterface(
          motors_[index].joint_name, 
          motors_[index].join_type, 
          &motors_[index].state)
      );
    }
    return state_interfaces;
  }

  std::vector<hardware_interface::CommandInterface>
  NomadHardware::export_command_interfaces()
  {
    std::vector<hardware_interface::CommandInterface> state_interfaces;
    RCLCPP_INFO(logger_, "Adding interfaces!");
    for (int index = 0; index < PURE_STATE_JOINTS; index++) {
      RCLCPP_INFO(logger_, "Adding %s command interface: %s", motors_[index].join_type, motors_[index].joint_name.c_str());
      /* Get state interfaces */
      state_interfaces.emplace_back(
        hardware_interface::CommandInterface(
          motors_[index].joint_name, 
          motors_[index].join_type, 
          &motors_[index].command)
      );
    }
    return state_interfaces;
  }
}

#include "pluginlib/class_list_macros.hpp"


PLUGINLIB_EXPORT_CLASS(
  nomad_hardware_interface::NomadHardware,
  hardware_interface::SystemInterface
)