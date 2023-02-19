/*
 * pid_controller.h
 *
 *  Created on: Feb 17, 2023
 *      Author: Alejo
 */

#ifndef APP_INCLUDE_PID_CONTROLLER_H_
#define APP_INCLUDE_PID_CONTROLLER_H_


#include "base_controller.h"
#include <stddef.h>

/**
 * @brief PID controller parameters
 *
 */
typedef struct {
  float kp;     /*<! Proportional constant. */
  float kd;     /*<! Derivative constant. */
  float ki;     /*<! Integral constant. */
  float limit;  /*<! Output limit. */
}PID_parameters_t;


/**
 * @brief PID controller structure.
 *
 */
typedef struct {
  Base_Controller_t base;   /*<! Base controller implementation */
  PID_parameters_t params;  /*<! Specific PID controller paramters */
}PID_controller_t;


/**
 * @brief Initialize a PID controller.
 *
 * @param controller Pointer to the controller allocation.
 * @param kp  Proportional constant.
 * @param kd  Derivative constant.
 * @param ki  Integral constant.
 * @param limit Output limit.
 */
void PID_CONTROLLER_Init (
    PID_controller_t* controller,
    float kp,
    float kd,
    float ki,
    float limit);
#endif /* APP_INCLUDE_PID_CONTROLLER_H_ */
