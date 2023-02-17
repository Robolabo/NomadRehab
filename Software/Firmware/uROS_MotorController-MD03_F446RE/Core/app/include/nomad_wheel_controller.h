/*
 * nomad_wheel_controller.h
 *
 *  Created on: Feb 15, 2023
 *      Author: Alejo
 */

#ifndef INC_NOMAD_WHEEL_CONTROLLER_H_
#define INC_NOMAD_WHEEL_CONTROLLER_H_

#include <stdbool.h>

#include "tim.h"
#include "encoder_controller.h"
#include "nomad_pwm.h"
#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"
#include "semphr.h"


#define NOMAD_WHEEL_TASK_NAME           "Wheel_controller_task"
#define NOMAD_WHEEL_TASK_PRIO           (tskIDLE_PRIORITY) + 1
#define NOMAD_WHEEL_TASK_STACK          (configMINIMAL_STACK_SIZE) + 512U


#define NOMAD_WHEEL_WHEEL_ENC           &htim2
#define NOMAD_WHEEL_WHEEL_PWM_CH        NOMAD_PWM_CHANNEL_3
#define NOMAD_WHEEL_WHEEL_CPR           48U
#define NOMAD_WHEEL_WHEEL_REDUCTION     10.0f
#define NOMAD_WHEEL_WHEEL_KP            10.0f
#define NOMAD_WHEEL_WHEEL_KD            0.0f
#define NOMAD_WHEEL_WHEEL_KI            0.0f
#define NOMAD_WHEEL_WHEEL_LIMIT         100.0f


#define NOMAD_WHEEL_ROTATION_ENC        &htim3
#define NOMAD_WHEEL_ROTATION_PWM_CH     NOMAD_PWM_CHANNEL_2
#define NOMAD_WHEEL_ROTATION_CPR        48U
#define NOMAD_WHEEL_ROTATION_REDUCTION  10.0f
#define NOMAD_WHEEL_ROTATION_KP         10.0f
#define NOMAD_WHEEL_ROTATION_KD         0.0f
#define NOMAD_WHEEL_ROTATION_KI         0.0f
#define NOMAD_WHEEL_ROTATION_LIMIT      100.0f


#define NOMAD_WHEEL_TASK_PERIOD_MS  10U
#define NOMAD_WHEEL_RESET_TIME_MS   100U

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


void NOMAD_WHEEL_Init ();
float NOMAD_WHEEL_getRotation ();
float NOMAD_WHEEL_getSpeed ();
void NOMAD_WHEEL_setPoint (float speed, float rotation);


#endif /* INC_NOMAD_WHEEL_CONTROLLER_H_ */
