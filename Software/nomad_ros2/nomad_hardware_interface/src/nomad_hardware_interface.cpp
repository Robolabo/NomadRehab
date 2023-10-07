#include "nomad_hardware_interface/nomad_hardware_interface.hpp"

namespace nomand_hardware_interface
{
  using hardware_interface::CallbackReturn;
  using hardware_interface::SystemInterface;

  CallbackReturn 
  NomadHardware::on_init(const hardware_interface::HardwareInfo & info)
  {
    if (SystemInterface::on_init(info) != CallbackReturn::SUCCESS) {
      return hardware_interface::CallbackReturn::ERROR;
    }

    /* Fetch paramters */
    serial_port_ = info_.hardware_parameters["serial_port"];
    baudrate_ = std::stoi(info_.hardware_parameters["baudrate"]);

    /* Base PID parameters */
    base_kp_ = std::stof(info_.hardware_parameters["base_kp"]);
    base_ki_ = std::stof(info_.hardware_parameters["base_ki_"]);
    base_kd_ = std::stof(info_.hardware_parameters["base_kd"]);
    base_limit_ = std::stof(info_.hardware_parameters["base_limit"]);
    base_cpr_ = std::stoi(info_.hardware_parameters["base_cpr"]);
    base_reduction_ = std::stof(info_.hardware_parameters["base_reduction"]);

    /* Steering PID parameters */
    steering_kp_ = std::stof(info_.hardware_parameters["steering_kp"]);
    steering_ki_ = std::stof(info_.hardware_parameters["steering_ki_"]);
    steering_kd_ = std::stof(info_.hardware_parameters["steering_kd"]);
    steering_limit_ = std::stof(info_.hardware_parameters["steering_limit"]);
    steering_cpr_ = std::stoi(info_.hardware_parameters["steering_cpr"]);
    steering_reduction_ = std::stof(info_.hardware_parameters["steering_reduction"]);


    /* Velocity PID parameters */
    velocity_kp_ = std::stof(info_.hardware_parameters["velocity_kp"]);
    velocity_ki_ = std::stof(info_.hardware_parameters["velocity_ki_"]);
    velocity_kd_ = std::stof(info_.hardware_parameters["velocity_kd"]);
    velocity_limit_ = std::stof(info_.hardware_parameters["velocity_limit"]);
    velocity_cpr_ = std::stoi(info_.hardware_parameters["velocity_cpr"]);
    velocity_reduction_ = std::stof(info_.hardware_parameters["velocity_reduction"]);

  

  }

  hardware_interface::CallbackReturn 
  NomadHardware::on_configure(const rclcpp_lifecycle::State & previous_state)
  {
    serial_ = nomad_robot::NomadComm(serial_port_, baudrate_, 1000U);

    serial_.set_steering_pid(
      steering_kp_, steering_kd_, steering_ki_, 
      steering_limit_, steering_cpr_, steering_reduction_);

    serial_.set_vel_pid(
      velocity_kp_, velocity_kd_, velocity_ki_, 
      velocity_limit_, velocity_cpr_, velocity_reduction_);

    serial_.set_base_pid(
      base_kp_, base_kd_, base_ki_, 
      base_limit_, base_cpr_, base_reduction_);
  }

  hardware_interface::return_type 
  NomadHardware::read(const rclcpp::Time & time, const rclcpp::Duration & period)
  {

  }

  hardware_interface::return_type 
  NomadHardware::write(const rclcpp::Time & time, const rclcpp::Duration & period)
  {
    
  }





}