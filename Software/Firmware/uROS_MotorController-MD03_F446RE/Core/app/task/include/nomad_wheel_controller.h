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
#include "pid_controller.h"


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
#define NOMAD_WHEEL_WHEEL_CPR           500U                 /*<! Count per revolution */
#define NOMAD_WHEEL_WHEEL_REDUCTION     126.0f               /*<! Motor reduction factor */

#define NOMAD_WHEEL_ROTATION_ENC        &htim3              /*<! Timer handle associated to the movement motor */
#define NOMAD_WHEEL_ROTATION_PWM_CH     NOMAD_PWM_CHANNEL_2 /*<! PWM channel associated to the movement motor */
#define NOMAD_WHEEL_ROTATION_CPR        500U                 /*<! Count per revolution */
#define NOMAD_WHEEL_ROTATION_REDUCTION  10947.0f               /*<! Motor reduction factor */


#define NOMAD_WHEEL_ROTATION_KP         50.0f   /*<! Proportional constant */
#define NOMAD_WHEEL_ROTATION_KD         0.0f    /*<! Derivative constant */
#define NOMAD_WHEEL_ROTATION_KI         0.0f    /*<! Integral constant */
#define NOMAD_WHEEL_ROTATION_LIMIT      100.0f  /*<! Controller max output */

#define NOMAD_WHEEL_WHEEL_KP            10.0f   /*<! Proportional constant */
#define NOMAD_WHEEL_WHEEL_KD            0.0f    /*<! Derivative constant */
#define NOMAD_WHEEL_WHEEL_KI            0.0f    /*<! Integral constant */
#define NOMAD_WHEEL_WHEEL_LIMIT         100.0f  /*<! Controller max output */
#define WHEEL_RADIUS_M                  0.05f   /*<! Wheel radius */


/************************************************************************
    FUNCTIONS
************************************************************************/


/**
 * @brief Get current wheel speed in radians per second.
 *
 * @return Wheel speed in radians per second.
 */
int NOMAD_WHEEL_getSpeed ();

/**
 * @brief Get current wheel angle in radians.
 *
 * @return wheel angle in radians.
 */
int NOMAD_WHEEL_getRotation ();

/**
 * @brief Check if the control is enabled.
 *
 * @return True if is enabled, else false.
 */
bool NOMAD_WHEEL_isControlEnabled ();

/**
 * @brief Enable the controller.
 *
 */
void NOMAD_WHEEL_enableControl ();

/**
 * @brief Disable the controller.
 *
 */
void NOMAD_WHEEL_disableControl ();

/**
 * @brief Reset the controller. Set the origin.
 *
 */
void NOMAD_WHEEL_reset ();

/**
 * @brief Initialize the wheel controller.
 *
 * @param speed_controller Controller structure used for speed control.
 * @param rotation_controller Controller structure used for ration control.
 */
void NOMAD_WHEEL_Init ();

/**
 * @brief store the context in ram to restore in case of soft reset.
 */
void NOMAD_WHEEL_saveContext ();

/**
 * @brief Restore context.
 */
void NOMAD_WHEEL_restoreContext ();

void NOMAD_WHEEL_setVelocity (int velocity);

void NOMAD_WHEEL_setSteering (int rotation);

void NOMAD_WHEEL_setSteeringPID (
    int kp, int kd,
    int ki, int limit,
    uint32_t cpr, int reduction);


void NOMAD_WHEEL_setTractionPID (
    int kp, int kd,
    int ki, int limit,
    uint32_t cpr, int reduction);


#endif /* INC_NOMAD_WHEEL_CONTROLLER_H_ */
