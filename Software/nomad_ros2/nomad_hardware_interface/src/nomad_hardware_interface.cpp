#include "nomad_hardware_interface/nomad_hardware_interface.hpp"

namespace nomand_hardware_interface
{
  using hardware_interface::CallbackReturn;
  using hardware_interface::SystemInterface;

  CallbackReturn 
  NomadHardware::on_init(const hardware_interface::HardwareInfo & info) {
    if (SystemInterface::on_init(info) != CallbackReturn::SUCCESS) {
      return CallbackReturn::ERROR;
    }
    
  }

}