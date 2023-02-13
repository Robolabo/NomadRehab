/*
 * encoder_test.h
 *
 *  Created on: Feb 13, 2023
 *      Author: Alejo
 */

#ifndef INC_ENCODER_TEST_H_
#define INC_ENCODER_TEST_H_

#include <math.h>

#include "stm32f4xx_hal.h"
#include "tim.h"
#include "encoder_controller.h"
#include "FreeRTOS.h"
#include "task.h"

#define ENCODER_TEST_TASK_NAME        "Encoder_test_task"
#define ENCODER_TEST_TASK_STACK       (configMINIMAL_STACK_SIZE) + 512U
#define ENCODER_TEST_TASK_PRIO        (tskIDLE_PRIORITY) + 1U

#define ENCODER_TEST_TIM_HANDLE       &htim2
#define ENCODER_TEST_CPR              48U
#define ENCODER_TEST_REDUCTION        24.0f

#define ENCODER_TEST_RATE_MS          10U


void ENCODER_TEST_Init ();

#endif /* INC_ENCODER_TEST_H_ */
