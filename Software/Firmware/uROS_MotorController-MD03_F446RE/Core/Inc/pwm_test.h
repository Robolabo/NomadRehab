/*
 * pwm_test.h
 *
 *  Created on: Feb 15, 2023
 *      Author: Alejo
 */

#ifndef INC_PWM_TEST_H_
#define INC_PWM_TEST_H_

#include "FreeRTOS.h"
#include "task.h"

#include "nomad_pwm.h"

#define PWM_TEST_TASK_NAME        "PWM_test_task"
#define PWM_TEST_TASK_STACK       (configMINIMAL_STACK_SIZE) + 512U
#define PWM_TEST_TASK_PRIO        (tskIDLE_PRIORITY) + 1U

#define PWM_TEST_PWM_CH            (NOMAD_PWM_CHANNEL_3)
#define PWM_TEST_PERIOD_MS         2000U
#define PWM_TEST_PERIOD_TICKS      pdMS_TO_TICKS(PWM_TEST_PERIOD_MS)



void PWM_TEST_Init ();

#endif /* INC_PWM_TEST_H_ */
