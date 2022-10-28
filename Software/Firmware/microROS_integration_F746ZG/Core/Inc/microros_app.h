/*
 * microros_app.h
 *
 *  Created on: Oct 12, 2022
 *      Author: alejo
 */

#ifndef INC_MICROROS_APP_H_
#define INC_MICROROS_APP_H_


/************** Includes **************/

/* Platform */
#include "stm32f7xx_hal.h"
#include "gpio.h"
#include "usart.h"

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
#include "sensor_telemetry/msg/telemetry.h"
#include "std_msgs/msg/bool.h"

/* custom transport */

/* Custom allocator */
#include "allocators.h"

/* FreeRTOS */
#include "FreeRTOS.h"
#include "task.h"


/************** Defines **************/
#define ROS_TASK_NAME		"uROS_task"
#define ROS_TASK_STACK		(configMINIMAL_STACK_SIZE) + 1024U
#define ROS_TASK_PRIO		(tskIDLE_PRIORITY) + 3U


/************** Functions **************/
void create_ros_task();


#endif /* INC_MICROROS_APP_H_ */
