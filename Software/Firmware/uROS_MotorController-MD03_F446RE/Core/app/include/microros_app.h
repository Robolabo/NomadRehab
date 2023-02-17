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
#include "stm32f4xx_hal.h"
#include "gpio.h"
#include "usart.h"
#include "nomad_pwm.h"
#include "nomad_encoder.h"

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
#include "motor_msgs/msg/motor_data.h"
#include "motor_msgs/msg/motor_position.h"
#include "motor_msgs/srv/motor_service.h"

/* custom transport */
#include "microros_transports.h"

/* Custom allocator */
#include "allocators.h"

/* FreeRTOS */
#include "FreeRTOS.h"
#include "task.h"


/************** Defines **************/
#define UROS_TASK_NAME		  "uROS_task"
#define UROS_TASK_STACK	    (configMINIMAL_STACK_SIZE) + 1024U
#define UROS_TASK_PRIO		  (tskIDLE_PRIORITY) + 1U
#define UROS_SPIN_TIME_MS   1U
#define UROS_DELAY_TIME_MS  10U


#define UROS_MOTOR_DUTY_TOPIC     "/control/motor_duty"
#define UROS_MOTOR_SERVICE        "/control/get_duty"
#define UROS_MOTOR_POS_TX_TOPIC   "/control/position"
#define UROS_MOTOR_POS_RX_TOPIC   "/control/set_position"
#define UROS_NODE_NAME            "motor_node"
#define UROS_MOTOR_POS_TIME_MS    10U
#define UROS_MOTOR_N_INSTANCES    6U


/************** Functions **************/
void create_ros_task();


#endif /* INC_MICROROS_APP_H_ */
