/*
 * uros_sonar.h
 *
 *  Created on: Oct 30, 2022
 *      Author: agome
 */

#ifndef INC_UROS_SONAR_H_
#define INC_UROS_SONAR_H_


#include <unistd.h>

/* Platform */
#include "stm32f7xx_hal.h"
#include "nomad_sonar.h"

/* micro-ROS */
#include "rcl/rcl.h"
#include "rclc/rclc.h"
#include "rclc/executor.h"
#include "rclc/node.h"
#include "rclc/publisher.h"
#include "rclc/subscription.h"
#include "rmw_microxrcedds_c/config.h"
#include "ucdr/microcdr.h"
#include "uxr/client/client.h"
#include "rmw_microros/rmw_microros.h"

/* Messages */
#include "sonar_msgs/msg/sonar_data.h"
#include "sonar_msgs/srv/sonar_service.h"

/* Custom allocator */
#include "allocators.h"

/* FreeRTOS */
#include "FreeRTOS.h"
#include "task.h"



/* Task parameters */

#define UROS_SONAR_TASK_NAME        "UROS_sonar_tsk"
#define UROS_SONAR_TASK_STACK       3000U
#define UROS_SONAR_TASK_PRIO        (tskIDLE_PRIORITY) + 1U


/* Application parameters */

#define UROS_SONAR_NODE_NAME        "sonar_node"
#define UROS_SONAR_NAMESPACE        "sonar"
#define UROS_SONAR_ENABLE_PERIODIC  1U
#define UROS_SONAR_PERIOD_MS        50U
#define UROS_SONAR_TOPIC_NAME       "distance"
#define UROS_SONAR_SERVICE_NAME     "get_distance"
#define UROS_SONAR_MAX_HANDLES      2U
#define UROS_SONAR_EXEC_PERIOD_MS   10U


void UROS_sonar_create_app();


#endif /* INC_UROS_SONAR_H_ */
