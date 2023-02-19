/*
 * pid_test.h
 *
 *  Created on: Feb 15, 2023
 *      Author: Alejo
 */

#ifndef INC_PID_TEST_H_
#define INC_PID_TEST_H_


#include <math.h>
#include "pid_controller.h"
#include "nomad_wheel_controller.h"

#include "FreeRTOS.h"
#include "task.h"


#define PID_TEST_TASK_NAME        "PID_TEST_task"
#define PID_TEST_TASK_STACK       (configMINIMAL_STACK_SIZE) + 512U
#define PID_TEST_TASK_PRIO        (tskIDLE_PRIORITY) + 1U

#define PID_TEST_PWM_CH            (NOMAD_PWM_CHANNEL_3)
#define PID_TEST_PERIOD_MS         5000U
#define PID_TEST_PERIOD_TICKS      pdMS_TO_TICKS(PID_TEST_PERIOD_MS)


void PID_TEST_Init ();


#endif /* INC_PID_TEST_H_ */
