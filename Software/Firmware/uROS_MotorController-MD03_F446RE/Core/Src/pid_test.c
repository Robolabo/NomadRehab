/*
 * pid_test.c
 *
 *  Created on: Feb 15, 2023
 *      Author: Alejo
 */

#include "pid_test.h"


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
  NOMAD_WHEEL_Init();

  /* Test position controller */
  while (1) {
    for (index = 0; index < PID_TEST_posSize; index++) {
      NOMAD_WHEEL_setPoint(0.0, setPoint);
      real_pos = NOMAD_WHEEL_getRotation();
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
