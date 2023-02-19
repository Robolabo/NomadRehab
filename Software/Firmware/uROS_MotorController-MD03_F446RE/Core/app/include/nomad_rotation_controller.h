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
 * @file nomad_robot_rotation_controller.h
 * @author Alejandro Gomez Molina (@Alejo2313)
 * @brief Rotatory base controller.
 *
 * @version 0.1
 * @date 19 feb. 2023
 *
 * @copyright Copyright (c) 2023
 *
 */

#ifndef APP_INCLUDE_NOMAD_ROTATION_CONTROLLER_H_
#define APP_INCLUDE_NOMAD_ROTATION_CONTROLLER_H_

/************************************************************************
    INCLUDES
************************************************************************/
#include <stdbool.h>

#include "tim.h"
#include "encoder_controller.h"
#include "nomad_pwm.h"
#include "pid_controller.h"

#include "FreeRTOS.h"
#include "task.h"

/************************************************************************
    DEFINES AND TYPES
************************************************************************/
#define NOMAD_ROTATION_TASK_NAME           "rotation_controller_task"         /*<! Task name */
#define NOMAD_ROTATION_TASK_PRIO           (tskIDLE_PRIORITY) + 1             /*<! Task priority*/
#define NOMAD_ROTATION_TASK_STACK          (configMINIMAL_STACK_SIZE) + 512U  /*<! Task stack size*/


#define NOMAD_ROTATION_TASK_PERIOD_MS      10U   /*<! Control period (milliseconds)*/
#define NOMAD_ROTATION_RESET_TIME_MS       100U  /*<! Reset time (milliseconds).
                                                 Time elapsed since the the motor is stopped
                                                 to the control values are reset */

#define NOMAD_ROTATION_ENC        &htim3              /*<! Timer handle associated to the movement motor */
#define NOMAD_ROTATION_PWM_CH     NOMAD_PWM_CHANNEL_2 /*<! PWM channel associated to the movement motor */
#define NOMAD_ROTATION_CPR        48U                 /*<! Count per revolution */
#define NOMAD_ROTATION_REDUCTION  10.0f               /*<! Motor reduction factor */


#define NOMAD_ROTATION_KP          10.0f
#define NOMAD_ROTATION_KD          0.0f
#define NOMAD_ROTATION_KI          0.0f
#define NOMAD_ROTATION_LIMIT       100.0f

/************************************************************************
    FUNCTIONS
************************************************************************/

/**
 * @brief Set controller reference point.
 *
 * @param rotation reference position.
 */
void NOMAD_ROTATION_setPoint (float rotation);

/**
 * @brief Get current wheel angle in radians.
 *
 * @return wheel angle in radians.
 */
float NOMAD_ROTATION_getRotation ();

/**
 * @brief Check if the control is enabled.
 *
 * @return True if is enabled, else false.
 */
bool NOMAD_ROTATION_isControlEnabled ();

/**
 * @brief Enable the controller.
 *
 */
void NOMAD_ROTATION_enableControl ();

/**
 * @brief Disable the controller.
 *
 */
void NOMAD_ROTATION_disableControl ();

/**
 * @brief Reset the controller. Set the origin.
 *
 */
void NOMAD_ROTATION_reset ();

/**
 * @brief Initialize rotation controller task.
 *
 */
void NOMAD_ROTATION_Init ();

#endif /* APP_INCLUDE_NOMAD_ROTATION_CONTROLLER_H_ */
