/*
 * NOMAD_wheel_controller.c
 *
 *  Created on: 12 feb. 2023
 *      Author: Alejo
 */


#include "nomad_wheel_controller.h"

static bool NOMAD_WHEEL_isEnabled = false;
static Nomad_Wheel_Control_t NOMAD_WHEEL_controller;

/**
 * @brief Clamp value to a threshold value
 *
 * @param value Input value.
 * @param threshold Max value.
 *        The input value will be limited to +- threshold
 * @return Clamped value.
 */
static float NOMAD_WHEEL_clamp (float value, float threshold) {
  if (value > threshold) {
    value = threshold;
  } else if (value < -threshold) {
    value = -threshold;
  }
  return value;
}


/**
 * @brief Reset the motor controller.
 *
 * @param controller
 */
static void NOMAD_WHEEL_resetController (Control_pid_t* controller) {
  if (controller != NULL) {
    controller->last_error = 0.0;
    controller->last_input = 0.0;
    controller->last_output = 0.0;
    controller->setpoit = 0.0;
  }
}

/**
 * @brief Initialize a controller structure.
 *
 * @param controller PID controller structure.
 * @param kp  Proportional constant.
 * @param kd  Derivative constant.
 * @param ki  Integral threshold
 * @param limit Output limit (anti-windup)
 */
static void NOMAD_WHEEL_InitController(
    Control_pid_t* controller,
    float kp,
    float kd,
    float ki,
    float limit)
{

  if (controller != NULL) {
    controller->kp = kp;
    controller->ki = ki;
    controller->kd = kd;
    controller->output_limit = limit;
    controller->dt = ((float)(NOMAD_WHEEL_TASK_PERIOD_MS))/1000.0f;

    NOMAD_WHEEL_resetController(controller);
  }
}


/**
 * @brief Calculate the PID output.
 *        Simple PID controller implementation
 *
 * @param controller PID controller structure.
 * @param input Input value from the feedback network.
 */
static float NOMAD_WHEEL_execute(Control_pid_t* controller, float input) {
  float error = 0.0;
  float proportional = 0.0;
  float derivative = 0.0;
  float integral = 0.0;

  // ToDo: Add sanity checks for the PID input

  error =  controller->setpoit - input;
  /* Calculate PID values */
  proportional = controller->kp*error;
  derivative =controller->kd*(error - controller->last_error)/controller->dt;
  integral = controller->ki*error*controller->dt;
  /* Avoid windup */
  integral = NOMAD_WHEEL_clamp(integral, controller->output_limit);

  /* Store values for future use */
  controller->last_input = input;
  controller->last_error = error;
  controller->last_output = proportional + derivative + integral;
  /* Avoid windup */
  controller->last_output = NOMAD_WHEEL_clamp(controller->last_output, controller->output_limit);

  return controller->last_output;
}


/**
 * @brief Main control task
 *
 */
void NOMAD_WHEEL_TaskFn() {
  float rotation_input = 0.0;
  float rotation_output = 0.0;
  float speed_input = 0.0;
  float speed_output = 0.0;
  float position = 0.0;
  float last_position = 0;

  /* Initialize hardware encoders */
  Encoder_controller_t* rotation_enc = ENC_CONTROL_init(
      NOMAD_WHEEL_ROTATION_ENC,
      NOMAD_WHEEL_ROTATION_CPR,
      NOMAD_WHEEL_ROTATION_REDUCTION);

  Encoder_controller_t* wheel_enc = ENC_CONTROL_init(
      NOMAD_WHEEL_WHEEL_ENC,
      NOMAD_WHEEL_WHEEL_CPR,
      NOMAD_WHEEL_WHEEL_REDUCTION);

  /* Initialize PWM */

  NOMAD_PWM_init();
  NOMAD_PWM_setDuty(NOMAD_WHEEL_WHEEL_PWM_CH, 0.0);
  NOMAD_PWM_setDuty(NOMAD_WHEEL_ROTATION_PWM_CH, 0.0);
  NOMAD_PWM_start(NOMAD_WHEEL_WHEEL_PWM_CH);
  NOMAD_PWM_start(NOMAD_WHEEL_ROTATION_PWM_CH);

  /* Initialize controllers */
  NOMAD_WHEEL_InitController(
      &NOMAD_WHEEL_controller.speed,
      NOMAD_WHEEL_WHEEL_KP,
      NOMAD_WHEEL_WHEEL_KD,
      NOMAD_WHEEL_WHEEL_KI,
      NOMAD_WHEEL_WHEEL_LIMIT);

  NOMAD_WHEEL_InitController(
      &NOMAD_WHEEL_controller.rotation,
      NOMAD_WHEEL_ROTATION_KP,
      NOMAD_WHEEL_ROTATION_KD,
      NOMAD_WHEEL_ROTATION_KI,
      NOMAD_WHEEL_ROTATION_LIMIT);

  /* Sanity check */
  if ((wheel_enc == NULL) || (rotation_enc == NULL)) {
    /* Something went wrong */
    while (1);
  }

  ENC_CONTROL_reset(NOMAD_WHEEL_ROTATION_ENC);
  ENC_CONTROL_reset(NOMAD_WHEEL_WHEEL_ENC);
  while (1) {
    /* Get controller inputs */
    rotation_input = ENC_CONTROL_getPostion(NOMAD_WHEEL_ROTATION_ENC);
    position = ENC_CONTROL_getPostion(NOMAD_WHEEL_WHEEL_ENC);

    /* ToDo: Speed also depends on the wheel radius. Update calculation */
    speed_input = (position - last_position)/(NOMAD_WHEEL_controller.speed.dt);
    last_position = position;

    /* Calculate control signals */
    speed_output = NOMAD_WHEEL_execute(&NOMAD_WHEEL_controller.speed, speed_input);
    rotation_output = NOMAD_WHEEL_execute(&NOMAD_WHEEL_controller.rotation, rotation_input);

    /* Update motor values */
    if (NOMAD_WHEEL_isEnabled) {
      NOMAD_PWM_setDuty(NOMAD_WHEEL_WHEEL_PWM_CH, speed_output);
      NOMAD_PWM_setDuty(NOMAD_WHEEL_ROTATION_PWM_CH, rotation_output);

    } else {
      /* This allows to move the motor manually */
      NOMAD_PWM_setDuty(NOMAD_WHEEL_WHEEL_PWM_CH, 0.0);
      NOMAD_PWM_setDuty(NOMAD_WHEEL_ROTATION_PWM_CH, 0.0);
    }

    /* Wait until next activation */
    vTaskDelay(pdMS_TO_TICKS(NOMAD_WHEEL_TASK_PERIOD_MS));
  }
}

/**
 * @brief Set controller reference point.
 *
 * @param speed Reference speed.
 * @param rotation reference position.
 */
void NOMAD_WHEEL_setPoint (float speed, float rotation) {

  /* ToDO: protect with mutex */
  NOMAD_WHEEL_controller.speed.setpoit = speed;
  NOMAD_WHEEL_controller.rotation.setpoit = rotation;
}


/**
 * @brief Get current wheel speed in radians per second.
 *
 * @return Wheel speed in radians per second.
 */
float NOMAD_WHEEL_getSpeed () {

  return NOMAD_WHEEL_controller.speed.last_input;
}

/**
 * @brief Get current wheel angle in radians.
 *
 * @return wheel angle in radians.
 */
float NOMAD_WHEEL_getRotation () {
  return NOMAD_WHEEL_controller.rotation.last_input;
}

/**
 * @brief Check if the control is enabled.
 *
 * @return True if is enabled, else false.
 */
bool NOMAD_WHEEL_isControlEnabled () {
  return NOMAD_WHEEL_isEnabled;
}

/**
 * @brief Enable the controller.
 *
 */
void NOMAD_WHEEL_enableControl () {
  NOMAD_WHEEL_isEnabled = true;
}

/**
 * @brief Disable the controller.
 *
 */
void NOMAD_WHEEL_disableControl () {
  NOMAD_WHEEL_isEnabled = false;
}

/**
 * @brief Reset the controller. Set the origin.
 *
 */
void NOMAD_WHEEL_reset () {
  /* Disable control */
  NOMAD_WHEEL_disableControl();
  /* Set both PWM to 0 */
  NOMAD_PWM_setDuty(NOMAD_WHEEL_WHEEL_PWM_CH, 0.0);
  NOMAD_PWM_setDuty(NOMAD_WHEEL_ROTATION_PWM_CH, 0.0);

  /* Wait until the motors are complete stopped */
  vTaskDelay(pdMS_TO_TICKS(NOMAD_WHEEL_RESET_TIME_MS));

  /* Reset encoders */
  ENC_CONTROL_reset(NOMAD_WHEEL_WHEEL_ENC);
  ENC_CONTROL_reset(NOMAD_WHEEL_ROTATION_ENC);

  /* Reset controllers */
  NOMAD_WHEEL_resetController(&NOMAD_WHEEL_controller.rotation);
  NOMAD_WHEEL_resetController(&NOMAD_WHEEL_controller.speed);

  /* Enable control */
  NOMAD_WHEEL_enableControl();
}


/**
 * @brief Initialze wheel controller
 *
 */
void NOMAD_WHEEL_Init () {
  // ToDo: Create software timer watchdog
  // ToDo: Create pose calculation for odometry
  // ToDo: Add sanity check
  xTaskCreate(
      NOMAD_WHEEL_TaskFn,
      NOMAD_WHEEL_TASK_NAME,
      NOMAD_WHEEL_TASK_STACK,
      NULL,
      NOMAD_WHEEL_TASK_PRIO,
      NULL);

}