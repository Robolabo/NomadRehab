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
 * @file pid_controller.h
 * @author Alejandro Gomez Molina (@Alejo2313)
 * @brief Simple PID controller implementation.
 *
 * @version 0.1
 * @date Feb 17, 2023
 *
 * @copyright Copyright (c) 2023
 *
 */
#ifndef APP_INCLUDE_PID_CONTROLLER_H_
#define APP_INCLUDE_PID_CONTROLLER_H_

/************************************************************************
    INCLUDES
************************************************************************/

#include "base_controller.h"
#include <stddef.h>

/************************************************************************
    DEFINES AND TYPES
************************************************************************/

/**
 * @brief PID controller parameters
 *
 */
typedef struct {
  float kp;     /*<! Proportional constant. */
  float kd;     /*<! Derivative constant. */
  float ki;     /*<! Integral constant. */
  float limit;  /*<! Output limit. */
}PID_parameters_t;


/**
 * @brief PID controller structure.
 *
 */
typedef struct {
  Base_Controller_t base;   /*<! Base controller implementation */
  PID_parameters_t params;  /*<! Specific PID controller paramters */
}PID_controller_t;


/************************************************************************
    FUNCTIONS
************************************************************************/

/**
 * @brief Initialize a PID controller.
 *
 * @param controller Pointer to the controller allocation.
 * @param kp  Proportional constant.
 * @param kd  Derivative constant.
 * @param ki  Integral constant.
 * @param limit Output limit.
 */
void PID_CONTROLLER_Init (
    PID_controller_t* controller,
    float kp,
    float kd,
    float ki,
    float limit);

#endif /* APP_INCLUDE_PID_CONTROLLER_H_ */
