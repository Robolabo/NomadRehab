/**                             _____________
 *              /\      /\     /             \
 *             //\\____//\\   |  TUlBVVVVVSE= |
 *            /     '      \   \  ___________/
 *           /   /\ '  /\    \ /_/                / /  ___
 *          |    == o ==      |       /|         / /  / _ \
 *           \      '        /       | |        / /__|  __/
 *             \           /         \ \        \____/\___|
 *             /----<o>---- \         / /        __  __  __  __      __        ___
 *             |            ' \       \ \       |__)/  \|__)/  \ __ /  |__| /\  |
 *             |    |    | '   '\      \ \      | \ \__/|__)\__/    \__|  |/--\ |
 *  _________  | ´´ |  ' |     '  \    / /
 *  |  MAYA  | |  ' |    | '       |__/ /
 *   \______/   \__/ \__/ \_______/____/
 *
 * @file pid_controller.c
 * @author Alejandro Gomez Molina (@Alejo2313)
 * @brief Simple PID controller implementation.
 *
 * @version 0.1
 * @date Feb 17, 2023
 *
 * @copyright Copyright (c) 2023
 *
 */

/************************************************************************
    INCLUDES
************************************************************************/

#include "pid_controller.h"


/************************************************************************
    DECLARATIONS
************************************************************************/

void PID_CONTROLLER_reset (Base_Controller_t* controller);
float PID_CONTROLLER_execute (Base_Controller_t* controller, float input);


/************************************************************************
    FUNCTIONS
************************************************************************/

/**
 * @brief Initialize a controller structure.
 *
 * @param controller PID controller structure.
 * @param kp  Proportional constant.
 * @param kd  Derivative constant.
 * @param ki  Integral threshold
 * @param limit Output limit (anti-windup)
 */
void PID_CONTROLLER_Init (
    PID_controller_t* controller,
    float kp,
    float kd,
    float ki,
    float limit)
{

  if (controller != NULL) {

    BASE_CONROLLER_reset((Base_Controller_t*)controller);

    controller->params.kp = kp;
    controller->params.kd = kd;
    controller->params.ki = kd;
    controller->params.limit = limit;

    controller->base.functions.execute = PID_CONTROLLER_execute;
    controller->base.functions.reset = PID_CONTROLLER_reset;
  }

}


/**
 * @brief Clamp value to a threshold value
 *
 * @param value Input value.
 * @param threshold Max value.
 *        The input value will be limited to +- threshold
 * @return Clamped value.
 */
static float PID_CONTROLLER_clamp (float value, float threshold) {
  if (value > threshold) {
    value = threshold;
  } else if (value < -threshold) {
    value = -threshold;
  }
  return value;
}


/**
 * @brief Calculate the PID output.
 *        Simple PID controller implementation
 *
 * @param controller PID controller structure.
 * @param input Input value from the feedback network.
 */
float PID_CONTROLLER_execute (Base_Controller_t* controller, float input) {
  float result = -1.0;
  float error = 0.0;
  float proportional = 0.0;
  float derivative = 0.0;
  float integral = 0.0;


  PID_parameters_t* pid_params;
  Base_control_params_t* base_params;

  if (controller != NULL) {
    pid_params = &((PID_controller_t*)controller)->params;
    base_params = &controller->params;

    error =  base_params->setpoit - input;
    /* Calculate PID values */
    proportional =  pid_params->kp * error;
    derivative = pid_params->kd * (error - base_params->last_error) / base_params->dt;
    integral = pid_params->ki * error * base_params->dt;

    /* Avoid windup */
    integral = PID_CONTROLLER_clamp(integral, pid_params->limit);

    /* Store values for future use */
    base_params->last_input = input;
    base_params->last_error = error;
    result = proportional + derivative + integral;

    /* Avoid windup */
    result = PID_CONTROLLER_clamp(result, pid_params->limit);
    base_params->last_output = result;

  }

  return result;
}

/**
 * @brief Reset the controller. Set the origin.
 *
 */
void PID_CONTROLLER_reset (Base_Controller_t* controller) {
  PID_controller_t* pid_controller = (PID_controller_t*)(controller);
  if (pid_controller != NULL) {
    pid_controller->base.params.last_error = 0.0;
    pid_controller->base.params.last_input = 0.0;
    pid_controller->base.params.last_output = 0.0;
    pid_controller->base.params.setpoit = 0.0;
  }
}


