

#ifndef NOMAD_HARDWARE_INTERFACE_
#define NOMAD_HARDWARE_INTERFACE_

#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "rclcpp/visibility_control.hpp"
#include "std_msgs/msg/string.hpp"


#include "hardware_interface/base_interface.hpp"


namespace nomand_hardware_interface
{
class NomadHardware : public hardware_interface::SystemInterface 
{
public:
  RCLCPP_SMART_PTR_DEFINITIONS(NomadHardware);
  
  RCLCPP_PUBLIC
  hardware_interface::CallbackReturn 
  on_init(const hardware_interface::HardwareInfo & info) override;

  RCLCPP_PUBLIC
  hardware_interface::CallbackReturn 
  on_configure(const rclcpp_lifecycle::State & previous_state) override;

  RCLCPP_PUBLIC
  hardware_interface::CallbackReturn
  on_activate(const rclcpp_lifecycle::State & previous_state) override;

  RCLCPP_PUBLIC
  hardware_interface::CallbackReturn
  on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

  RCLCPP_PUBLIC
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  RCLCPP_PUBLIC
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  RCLCPP_PUBLIC
  hardware_interface::return_type 
  read(const rclcpp::Time & time, const rclcpp::Duration & period) override;

  RCLCPP_PUBLIC
  hardware_interface::return_type 
  write(const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  std::vector<double> hw_states_position_;
  std::vector<double> hw_states_velocity_;

  nomad_robot::NomadComm serial_;
  std::string serial_port_;
  int baudrate_;

  /* Base PID parameters */
  float base_kp_;
  float base_ki_;
  float base_kd_;
  float base_limit_;
  int base_cpr_;
  float base_reduction_;

  /* Steering PID parameters */
  float steering_kp_;
  float steering_ki_;
  float steering_kd_;
  float steering_limit_;
  int steering_cpr_;
  float steering_reduction_;

  /* Velocity PID parameters */
  float velocity_kp_;
  float velocity_ki_;
  float velocity_kd_;
  float velocity_limit_;
  int velocity_cpr_ ;
  float velocity_reduction_;

  float cmd_velocity_;
  float cmd_steering_;
  float cmd_base_;

  float state_velocity_;
  float state_steering_;
  float state_base_;


}
}

#endif /* NOMAD_HARDWARE_INTERFACE_ */