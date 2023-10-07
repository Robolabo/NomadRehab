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
 * @file base_controller.h
 * @author Alejandro Gomez Molina (@Alejo2313)
 * @brief Base controller interface. Creates a generic
 *        controller abstraction for creating controllers.
 *        Extend the base structure a implement the "execute"
 *        and "reset" functions to implement your own controller"
 * @version 0.1
 * @date Feb 17, 2023
 *
 * @copyright Copyright (c) 2023
 *
 */
#ifndef APP_INCLUDE_BASE_CONTROLLER_H_
#define APP_INCLUDE_BASE_CONTROLLER_H_

/************************************************************************
    INCLUDES
************************************************************************/
#include <string.h>

struct BASE_CONTROLLER_controller_s;
typedef struct BASE_CONTROLLER_controller_s Base_Controller_t;

/**
 * @brief Function interface.
 *
 */
typedef struct
{
  float (*execute)(Base_Controller_t* controller, float input);   /*<! Execute the control function for a given input */
  void (*reset)(Base_Controller_t* controller);                   /*<! Reset the controller parameters */
} Base_control_interface_t;

/**
 * @brief Parameters interface.
 *
 */
typedef struct {
  float dt;               /*<! Time elapsed between two executions (in seconds) */
  float setpoit;          /*<! Reference point to achieve */
  float last_input;       /*<! Stores the last input used */
  float last_output;      /*<! Store the last calculated output */
  float last_error;       /*<! Stores the last error */
  float output_limit;     /*<! Stores the output limit */
}Base_control_params_t;


/**
 * @brief Base controller structure.
 *
 */
struct BASE_CONTROLLER_controller_s {
  Base_control_interface_t functions;   /*<! Function interface */
  Base_control_params_t params;         /*<! Parameters storage */
};


/**
 * @brief Reset a base controller structure.
 *
 * @param controller Pointer o a base controller.
 */
static inline void BASE_CONROLLER_reset (Base_Controller_t* controller) {
  memset((void*)controller, 0, sizeof(Base_Controller_t));
}

#endif /* APP_INCLUDE_BASE_CONTROLLER_H_ */
