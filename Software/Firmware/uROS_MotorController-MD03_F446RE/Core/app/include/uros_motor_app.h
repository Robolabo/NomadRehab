/**                             _____________
 *              /\      /\     /             \
 *             //\\____//\\   |  TUlBVVVVVSE= |
 *            /     '      \   \  ___________/
 *           /   /\ '  /\    \ /_/                / /  ___
 *          |    == o ==      |       /|         / /  / _ \
 *           \      '        /       | |        / /__|  __/
 *             \           /         \ \        \____/\___|
 *             /----<o>---- \         / /        __  __  __  __      __        ___
 *             |            ' \       \ \       |__)/  \|__)/  \ __ /  |__| /\  |
 *             |    |    | '   '\      \ \      | \ \__/|__)\__/    \__|  |/--\ |
 *  _________  | ´´ |  ' |     '  \    / /
 *  |  MAYA  | |  ' |    | '       |__/ /
 *   \______/   \__/ \__/ \_______/____/
 *
 * @file nomad_rotation_controller.h
 * @author Alejandro Gomez Molina (@Alejo2313)
 * @brief Rotatory base controller.
 *
 * @version 0.1
 * @date 19 feb. 2023
 *
 * @copyright Copyright (c) 2023
 *
 */
#ifndef APP_INCLUDE_UROS_MOTOR_APP_H_
#define APP_INCLUDE_UROS_MOTOR_APP_H_

/************************************************************************
    INCLUDES
************************************************************************/
#include <string.h>

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

/* Custom transports */
#include "microros_transports.h"

/* Custom allocator */
#include "allocators.h"

/* FreeRTOS */
#include "FreeRTOS.h"
#include "task.h"

/* Message types */
#include "geometry_msgs/msg/twist_stamped.h"
#include "geometry_msgs/msg/pose_stamped.h"
#include "geometry_msgs/msg/twist.h"
#include "geometry_msgs/msg/pose.h"
#include "nav_msgs/msg/odometry.h"

/* Drivers */
#include "usart.h"
#include "nomad_wheel_controller.h"
#include "nomad_rotation_controller.h"

/************************************************************************
    DEFINES AND TYPES
************************************************************************/
#define UROS_MOTOR_TASK_NAME      "uROS_motor_task"                     /*<! Task name */
#define UROS_MOTOR_TASK_STACK     (configMINIMAL_STACK_SIZE) + 2048U    /*<! Task priority*/
#define UROS_MOTOR_TASK_PRIO      (tskIDLE_PRIORITY) + 1U               /*<! Task stack size*/

#define UROS_MOTOR_NODE_NAME      "wheels"
#define UROS_MOTOR_NODE_NS        ""

#define UROS_MOTOR_SUB_SPEED      "/motor/cmd_vel"        /*<! Speed command topic */
#define UROS_MOTOR_PUB_SPEED      "/motor/speed_state"      /*<! Speed state topic */
#define UROS_MOTOR_SUB_ROTATION   "/motor/rotation_cmd"     /*<! Rotation command topic */
#define UROS_MOTOR_PUB_ROTATION   "/motor/rotation_state"   /*<! Rotation state topic */
#define UROS_MOTOR_PUB_ODOMETRY   "/motor/odometry"         /*<! Odometry state topic (ToDo: impelement)*/

#define UROS_MOTOR_N_INSTANCES    6U   /*<! Total number of instance (pub+sub+tim+...)*/
#define UROS_MOTOR_PUB_PERIOD_MS  50U   /*<! Publication period (milliseconds) */
#define UROS_MOTOR_SPIN_TIME_MS   1U    /*<! uROS spin time (milliseconds) */
#define UROS_MOTOR_DELAY_TIME_MS  9U    /*<! uROS task period (milliseconds) */


#define UROS_MOTOR_STAMPED_DATA   0  /* Use stamped messages */

/************************************************************************
    FUNCTIONS
************************************************************************/

/**
 * @brief Initalize microROS task
 */
void UROS_MOTOR_init ();

#endif /* APP_INCLUDE_UROS_MOTOR_APP_H_ */
