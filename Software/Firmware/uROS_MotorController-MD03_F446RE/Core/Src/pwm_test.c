/*
 * pwm_test.c
 *
 *  Created on: 15 feb. 2023
 *      Author: Alejo
 */

#include "pwm_test.h"


static float PWM_TEST_duties [] = {10.0, -10.0, 25.0, -25.0, 50.0, -50.0, 75.0, -75.0, 100.0, -100.0};
static size_t PWM_TEST_size = sizeof(PWM_TEST_duties)/sizeof(float);

/**
 * @brief PWM test task function.
 *
 */
static void PWM_TEST_taskFn () {

  uint32_t index = 0;

  /* Initialize timer in PWM mode */
  NOMAD_PWM_init();

  /* Start PWM in channel */
  NOMAD_PWM_start(PWM_TEST_PWM_CH);

  /* Test different levels in both directions */
  while (1) {
    for (index = 0; index < PWM_TEST_size; index++) {
      NOMAD_PWM_setDuty(PWM_TEST_PWM_CH, PWM_TEST_duties[index]);
      vTaskDelay(PWM_TEST_PERIOD_TICKS);
    }
  }
}


/**
 * @brief Initialize PWM task.
 *
 */
void PWM_TEST_Init () {
  xTaskCreate(
      PWM_TEST_taskFn,
      PWM_TEST_TASK_NAME,
      PWM_TEST_TASK_STACK,
      NULL,
      PWM_TEST_TASK_PRIO,
      NULL);
}
