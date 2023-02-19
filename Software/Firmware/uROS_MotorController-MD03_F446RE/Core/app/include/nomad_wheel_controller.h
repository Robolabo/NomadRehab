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
 * @file nomad_wheel_controller.h
 * @author Alejandro Gomez Molina (@Alejo2313)
 * @brief Nomad 200 wheel controller. Source code for controlling
 *        the speed and the position of the synchro driver.
 *
 * @version 0.1
 * @date 12 feb. 2023
 *
 * @copyright Copyright (c) 2023
 *
 */
#ifndef INC_NOMAD_WHEEL_CONTROLLER_H_
#define INC_NOMAD_WHEEL_CONTROLLER_H_
/************************************************************************
    INCLUDES
************************************************************************/
/* Standard lib */
#include <stdbool.h>
/* Scheduler */
#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"
#include "semphr.h"
/* Drivers */
#include "tim.h"
#include "nomad_pwm.h"
#include "encoder_controller.h"
#include "base_controller.h"

/************************************************************************
    DEFINES AND TYPES
************************************************************************/

#define NOMAD_WHEEL_TASK_NAME           "Wheel_controller_task"           /*<! Task name */
#define NOMAD_WHEEL_TASK_PRIO           (tskIDLE_PRIORITY) + 1            /*<! Task priority*/
#define NOMAD_WHEEL_TASK_STACK          (configMINIMAL_STACK_SIZE) + 512U /*<! Task stack size*/


#define NOMAD_WHEEL_TASK_PERIOD_MS      10U   /*<! Control period (milliseconds)*/
#define NOMAD_WHEEL_RESET_TIME_MS       100U  /*<! Reset time (milliseconds).
                                              Time elapsed since the the motor is stopped
                                              to the control values are reset */

#define NOMAD_WHEEL_WHEEL_ENC           &htim2              /*<! Timer handle associated to the movement motor */
#define NOMAD_WHEEL_WHEEL_PWM_CH        NOMAD_PWM_CHANNEL_3 /*<! PWM channel associated to the movement motor */
#define NOMAD_WHEEL_WHEEL_CPR           48U                 /*<! Count per revolution */
#define NOMAD_WHEEL_WHEEL_REDUCTION     10.0f               /*<! Motor reduction factor */

#define NOMAD_WHEEL_ROTATION_ENC        &htim3              /*<! Timer handle associated to the movement motor */
#define NOMAD_WHEEL_ROTATION_PWM_CH     NOMAD_PWM_CHANNEL_2 /*<! PWM channel associated to the movement motor */
#define NOMAD_WHEEL_ROTATION_CPR        48U                 /*<! Count per revolution */
#define NOMAD_WHEEL_ROTATION_REDUCTION  10.0f               /*<! Motor reduction factor */

/************************************************************************
    FUNCTIONS
************************************************************************/

float NOMAD_WHEEL_getRotation ();

float NOMAD_WHEEL_getSpeed ();

void NOMAD_WHEEL_setPoint (float speed, float rotation);

void NOMAD_WHEEL_Init (Base_Controller_t* speed_controller, Base_Controller_t* rotation_controller);

#endif /* INC_NOMAD_WHEEL_CONTROLLER_H_ */
