

#ifndef NOMAD_HARDWARE_INTERFACE_
#define NOMAD_HARDWARE_INTERFACE_

#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "hardware_interface/system.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/logger.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "rclcpp/visibility_control.hpp"
#include "std_msgs/msg/string.hpp"

#include "nomad_hardware_interface/nomad_motor_comms.hpp"


namespace nomad_hardware_interface
{
class NomadHardware : public hardware_interface::SystemInterface 
{
public:
  rclcpp::Logger logger_;
  RCLCPP_SMART_PTR_DEFINITIONS(NomadHardware)


  NomadHardware();
  
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

  bool emulated_mode_;
  std::shared_ptr<nomad_robot::NomadComm> serial_;
  std::string serial_port_;
  int baudrate_;

  enum motor_index {
    BASE_INDEX = 0,
    VEL_INDEX,
    STEERING_INDEX,
    /* enum size*/
    PURE_STATE_JOINTS,
    /****/
    LEFT_WHEEL_VEL = PURE_STATE_JOINTS,
    LEFT_WHEEL_STEERING,
    RIGTH_WHEEL_VEL,
    RIGHT_WHEEL_STEERING,
    /* last joint*/
    END_OF_JOINTS
  };

  struct motor_data
  {
    std::string name;
    std::string joint_name;
    const char* join_type;
    float kp;
    float kd;
    float ki;
    float limit;
    uint32_t cpr;
    float reduction;

    double command;
    double state;
  };
  
  motor_data motors_[END_OF_JOINTS];
};
}

#endif /* NOMAD_HARDWARE_INTERFACE_ */