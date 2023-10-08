#include "nomad_hardware_interface/nomad_motor_comms.hpp"



namespace nomad_robot
{
  NomadComm::NomadComm(const std::string &serial_device, int32_t baud_rate, int32_t timeout_ms)
  {  
      serial_.setPort(serial_device);
      serial_.setBaudrate(baud_rate);
      serial::Timeout tt = serial::Timeout::simpleTimeout(timeout_ms);
      serial_.setTimeout(tt); // This should be inline except setTimeout takes a reference and so needs a variable
      serial_.open();
  }

  bool NomadComm::get_state(double& velocity, double& steering, double& base)
  {
    char command;
    /* Send data request */
    serial_.write("r\n");
    std::string response = serial_.readline();
    /*process response*/
    std::istringstream response_stream(response);
    response_stream >> command >> velocity  >> steering >> base;

    velocity /= 1000.0;
    steering /= 1000.0;
    base /= 1000.0;

    return ('r' == command);
  }

  void NomadComm::set_state(double velocity, double steering, double base)
  {
    int i_velocity = (int)(velocity*1000);
    int i_steering = (int)(steering*1000);
    int i_base = (int)(base*1000);

    /* Format message */
    std::string message = 
      "c " + std::to_string(i_velocity) + " " + std::to_string(i_steering) + " " + std::to_string(i_base) + "\n";
    /* Write message */
    serial_.write(message);
  }

  void NomadComm::set_pid(std::string command, 
    float kp, float kd, float ki, 
    float limit, uint32_t cpr, float reduction) 
  {
    std::string message = 
      command + " " +
      std::to_string((int)kp*1000) + " " +
      std::to_string((int)kd*1000) + " " +
      std::to_string((int)ki*1000) + " " +
      std::to_string((int)limit*1000) + " " +
      std::to_string((int)cpr) + " " +
      std::to_string((int)reduction*1000) + "\n";
      
      serial_.write(message);
  }

  void NomadComm::set_vel_pid(
    float kp, float kd, float ki, 
    float limit, uint32_t cpr, float reduction)
  {
    set_pid("v", kp, kd, ki, limit, cpr, reduction);
  }

  void NomadComm::set_steering_pid(
    float kp, float kd, float ki, 
    float limit, uint32_t cpr, float reduction)
  {
    set_pid("s", kp, kd, ki, limit, cpr, reduction);
  }

  void NomadComm::set_base_pid(
    float kp, float kd, float ki, 
    float limit, uint32_t cpr, float reduction)
  {
    set_pid("b", kp, kd, ki, limit, cpr, reduction);
  }
}

