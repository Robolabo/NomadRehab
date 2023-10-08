
#include <iostream>
#include <thread>
#include <chrono>
#include "nomad_hardware_interface/nomad_motor_comms.hpp"


int main(int argc, char const *argv[])
{

  if (argc < 3) {
    return 0;
  }
  std::string serial_device = std::string(argv[1]);
  int32_t baud_rate = atoi(argv[2]);

  nomad_robot::NomadComm comm(serial_device, baud_rate, 1000U);
  double counter = 0;
  double velocity, steering,  base;

  comm.set_steering_pid(10, 0, 0, 100, 500, 10947);
  comm.set_vel_pid(10, 0, 0, 100, 500, 126);
  comm.set_base_pid(10, 0, 0, 100, 100, 10);

  while (1) {

    std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
    comm.set_state(counter,counter,counter++);
    comm.get_state(velocity, steering,  base);
    std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();

    std::cout << velocity << " " << steering << " " << base <<" "<< std::chrono::duration_cast<std::chrono::microseconds>(end - begin).count() << std::endl;
  }

  return 0;
}