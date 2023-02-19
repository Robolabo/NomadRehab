/*
 * base_controller.h
 *
 *  Created on: Feb 17, 2023
 *      Author: Alejo
 */

#ifndef APP_INCLUDE_BASE_CONTROLLER_H_
#define APP_INCLUDE_BASE_CONTROLLER_H_

#include <string.h>

struct BASE_CONTROLLER_controller_s;
typedef struct BASE_CONTROLLER_controller_s Base_Controller_t;

/**
 * @brief
 *
 */
typedef struct
{
  float (*execute)(Base_Controller_t* controller, float input);
  void (*reset)(Base_Controller_t* controller);
} Base_control_interface_t;

typedef struct {
  float dt;
  float setpoit;
  float last_input;
  float last_output;
  float last_error;
  float output_limit;
}Base_control_params_t;


struct BASE_CONTROLLER_controller_s {
  Base_control_interface_t functions;
  Base_control_params_t params;
};


static inline void BASE_CONROLLER_reset (Base_Controller_t* controller) {
  memset((void*)controller, 0, sizeof(Base_Controller_t));
}





#endif /* APP_INCLUDE_BASE_CONTROLLER_H_ */
