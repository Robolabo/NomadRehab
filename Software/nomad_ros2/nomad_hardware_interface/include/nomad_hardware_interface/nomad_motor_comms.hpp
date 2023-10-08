#ifndef NOMAD_MOTOR_COMSS_HPP_
#define NOMAD_MOTOR_COMSS_HPP_

#include <cstring>
#include <cstdint>
#include <serial/serial.h>

namespace nomad_robot
{
class NomadComm
{
public:
  
  /**
   * @brief Construct a new Nomad Comm object
   * 
   * @param serial_device 
   * @param baud_rate 
   * @param timeout_ms 
   */
  NomadComm(const std::string &serial_device, int32_t baud_rate, int32_t timeout_ms);

  /**
   * @brief Get the state object
   * 
   * @param velocity 
   * @param steering 
   * @param base 
   * @return true 
   * @return false 
   */
  bool get_state(double& velocity, double& steering, double& base);

  /**
   * @brief Set the state object
   * 
   * @param velocity 
   * @param steering 
   * @param base 
   */
  void set_state(double velocity, double steering, double base);

  void set_base_pid(
    float kp, float kd, float ki, 
    float limit, uint32_t cpr, float reduction);

  void set_steering_pid(
    float kp, float kd, float ki, 
    float limit, uint32_t cpr, float reduction);

  void set_vel_pid(
    float kp, float kd, float ki, 
    float limit, uint32_t cpr, float reduction);

  void set_pid(std::string command, 
    float kp, float kd, float ki, 
    float limit, uint32_t cpr, float reduction);
  

private:
  serial::Serial serial_;
};

}


#endif /* NOMAD_MOTOR_COMSS_HPP_ */