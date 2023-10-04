

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
  hardware_interface::return_type read() override;

  RCLCPP_PUBLIC
  hardware_interface::return_type write() override;

private:
  std::vector<double> hw_states_position_;
  std::vector<double> hw_states_velocity_;
}
}

#endif /* NOMAD_HARDWARE_INTERFACE_ */