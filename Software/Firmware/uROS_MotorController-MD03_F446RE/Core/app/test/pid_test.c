/*
 * pid_test.c
 *
 *  Created on: Feb 15, 2023
 *      Author: Alejo
 */

#include "pid_test.h"


#define PID_TEST_KP            10.0f
#define PID_TEST_KD            0.0f
#define PID_TEST_KI            0.0f
#define PID_TEST_LIMIT         100.0f

static float PID_TEST_position [] = {M_PI_4, -M_PI_4, M_PI_2, -M_PI_2, M_PI, -M_PI, 2*M_PI,  -2*M_PI};
static size_t PID_TEST_posSize = sizeof(PID_TEST_position)/sizeof(float);

float setPoint = 2*M_PI;
float real_pos = 0.0;
/**
 * @brief PWM test task function.
 *
 */
static void PID_TEST_taskFn () {

  uint32_t index = 0;


  /* Initialize controller */
  //NOMAD_WHEEL_Init();

  PID_controller_t speed_controller;
  PID_controller_t rotation_encoder;

  PID_CONTROLLER_Init(
      &speed_controller,
      PID_TEST_KP,
      PID_TEST_KD,
      PID_TEST_KI,
      PID_TEST_LIMIT);

  PID_CONTROLLER_Init(
      &rotation_encoder,
      PID_TEST_KP,
      PID_TEST_KD,
      PID_TEST_KI,
      PID_TEST_LIMIT);

  NOMAD_WHEEL_Init((Base_Controller_t*)&speed_controller, (Base_Controller_t*)&rotation_encoder);

  /* Test position controller */
  while (1) {
    for (index = 0; index < PID_TEST_posSize; index++) {
      NOMAD_WHEEL_setPoint(setPoint, 0.0);
      real_pos = NOMAD_WHEEL_getSpeed();
      vTaskDelay(10);
    }

    /* Test speed controller */
    /*
    for (index = 0; index < PID_TEST_posSize; index++) {
      vTaskDelay(PID_TEST_PERIOD_TICKS);
    }
    */
  }
}


/**
 * @brief Initialize PWM task.
 *
 */
void PID_TEST_Init () {
  xTaskCreate(
      PID_TEST_taskFn,
      PID_TEST_TASK_NAME,
      PID_TEST_TASK_STACK,
      NULL,
      PID_TEST_TASK_PRIO,
      NULL);
}
