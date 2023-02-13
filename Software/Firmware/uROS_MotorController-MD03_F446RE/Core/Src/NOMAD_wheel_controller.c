/*
 * NOMAD_wheel_controller.c
 *
 *  Created on: 12 feb. 2023
 *      Author: Alejo
 */


#include "tim.h"
#include "encoder_controller.h"
#include "nomad_pwm.h"
#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"


#define NOMAD_WHEEL_TASK_NAME           "Wheel_controller_task"
#define NOMAD_WHEEL_TASK_PRIO           (tskIDLE_PRIORITY) + 1
#define NOMAD_WHEEL_TASK_STACK          (configMINIMAL_STACK_SIZE) + 512U



#define NOMAD_WHEEL_WHEEL_ENC           &htim2
#define NOMAD_WHEEL_WHEEL_PWM_CH        NOMAD_PWM_CHANNEL_1
#define NOMAD_WHEEL_WHEEL_CPR           500U
#define NOMAD_WHEEL_WHEEL_REDUCTION     4.5f
#define NOMAD_WHEEL_WHEEL_KP            1.0f
#define NOMAD_WHEEL_WHEEL_KD            0.0f
#define NOMAD_WHEEL_WHEEL_KI            0.0f
#define NOMAD_WHEEL_WHEEL_LIMIT         100.0f


#define NOMAD_WHEEL_ROTATION_ENC        &htim3
#define NOMAD_WHEEL_ROTATION_PWM_CH    NOMAD_PWM_CHANNEL_2
#define NOMAD_WHEEL_ROTATION_CPR        500U
#define NOMAD_WHEEL_ROTATION_REDUCTION  4.5f
#define NOMAD_WHEEL_ROTATION_KP         1.0f
#define NOMAD_WHEEL_ROTATION_KD         0.0f
#define NOMAD_WHEEL_ROTATION_KI         0.0f
#define NOMAD_WHEEL_ROTATION_LIMIT      100.0f


#define NOMAD_WHEEL_TASK_PERIOD_MS 10U

/**
 * @brief
 *
 */
typedef struct {
  float kp;
  float kd;
  float ki;

  float setpoit;
  float dt;

  float last_input;
  float last_output;
  float last_error;

  float output_limit;
}Control_pid_t;


typedef struct {
  Control_pid_t speed;
  Control_pid_t rotation;
}Nomad_Wheel_Control_t;



static Nomad_Wheel_Control_t NOMAD_WHEEL_controller;


/**
 * @brief Clamp value to a threshold value
 *
 * @param value Input value.
 * @param threshold Max value.
 *        The input value will be limited to +- threshold
 * @return Clamped value.
 */
inline float NOMAD_WHEEL_clamp (float value, float threshold) {
  if (value > threshold) {
    value = threshold;
  }
  else if (value < -threshold) {
    value = -threshold;
  }
  return value;
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
  controller->kp = kp;
  controller->ki = ki;
  controller->kd = kd;
  controller->output_limit = limit;
  controller->dt = ((float)(NOMAD_WHEEL_TASK_PERIOD_MS))/1000.0f;

  controller->last_error = 0.0;
  controller->last_input = 0.0;
  controller->last_output = 0.0;
  controller->setpoit = 0.0;
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
  derivative = controller->kd*(error - controller->last_error)/controller->dt;
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

  while (1) {
    /* Get controller inputs */
    rotation_input = ENC_CONTROL_getPostion(NOMAD_WHEEL_ROTATION_ENC);
    position = ENC_CONTROL_getPostion(NOMAD_WHEEL_WHEEL_ENC);
    speed_input = (position - NOMAD_WHEEL_controller.speed.last_input)/(NOMAD_WHEEL_controller.speed.dt);

    /* Calculate control signals */
    speed_output = NOMAD_WHEEL_execute(&NOMAD_WHEEL_controller.speed, speed_input);
    rotation_output = NOMAD_WHEEL_execute(&NOMAD_WHEEL_controller.rotation, rotation_input);

    /* Update motor values */
    NOMAD_PWM_setDuty(NOMAD_WHEEL_WHEEL_PWM_CH, speed_output);
    NOMAD_PWM_setDuty(NOMAD_WHEEL_ROTATION_PWM_CH, rotation_output);

    /* Wait until next activation */
    vTaskDelay(pdMS_TO_TICKS(NOMAD_WHEEL_TASK_PERIOD_MS));
  }
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
