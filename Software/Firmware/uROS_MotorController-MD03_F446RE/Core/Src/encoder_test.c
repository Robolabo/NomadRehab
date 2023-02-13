/*
 * encoder_test.c
 *
 *  Created on: Feb 13, 2023
 *      Author: Alejo
 */

#include "encoder_test.h"


static void ENCODER_TEST_taskFN() {

  float position = 0.0;
  Encoder_controller_t* controller = ENC_CONTROL_init(
      ENCODER_TEST_TIM_HANDLE,
      ENCODER_TEST_CPR,
      ENCODER_TEST_REDUCTION);

  if (controller == NULL) {
    /* Something went wrong */
    while (1);
  }

  ENC_CONTROL_reset(ENCODER_TEST_TIM_HANDLE);
  while (1) {

    /* Move one turn to the left */
    do {
      position = ENC_CONTROL_getPostion(ENCODER_TEST_TIM_HANDLE);
      vTaskDelay(pdMS_TO_TICKS(ENCODER_TEST_RATE_MS));
    }
    while (position > 2*M_PI);

    /* Move one turn to the right */
    do {
      position = ENC_CONTROL_getPostion(ENCODER_TEST_TIM_HANDLE);
      vTaskDelay(pdMS_TO_TICKS(ENCODER_TEST_RATE_MS));
    }
    while (position < 0.0);


    while (1) {
      position = ENC_CONTROL_getPostion(ENCODER_TEST_TIM_HANDLE);
      vTaskDelay(pdMS_TO_TICKS(ENCODER_TEST_RATE_MS));
    }
  }
}


void ENCODER_TEST_Init () {
  xTaskCreate(
      ENCODER_TEST_taskFN,
      ENCODER_TEST_TASK_NAME,
      ENCODER_TEST_TASK_STACK,
      NULL,
      ENCODER_TEST_TASK_PRIO,
      NULL);
}
