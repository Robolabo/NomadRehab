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
 * @file nomad_motor_task.h
 * @author Alejo
 * @brief 
 *
 * @version 0.1
 * @date Oct 6, 2023
 *
 * @copyright Copyright (c) 2023
 *
 */

#ifndef APP_TASK_INCLUDE_NOMAD_MOTOR_TASK_H_
#define APP_TASK_INCLUDE_NOMAD_MOTOR_TASK_H_

/************************************************************************
    INCLUDES
************************************************************************/

#include <stdint.h>
#include <stdlib.h>

#include "stm32f4xx_hal.h"
#include "usart.h"
#include "nomad_rotation_controller.h"
#include "nomad_wheel_controller.h"
#include "serial_transport.h"

/************************************************************************
    DEFINES AND TYPES
************************************************************************/

#define NOMAD_APP_TASK1_NAME        "processInput_task"               /*<! Task name */
#define NOMAD_APP_TASK1_PRIO        (tskIDLE_PRIORITY) + 1            /*<! Task priority*/
#define NOMAD_APP_TASK1_STACK       (configMINIMAL_STACK_SIZE) + 512U /*<! Task stack size*/

#define NOMAD_APP_TASK2_NAME        "sendFeedback_task"               /*<! Task name */
#define NOMAD_APP_TASK2_PRIO        (tskIDLE_PRIORITY) + 1            /*<! Task priority*/
#define NOMAD_APP_TASK2_STACK       (configMINIMAL_STACK_SIZE) + 512U /*<! Task stack size*/

#define NOMAD_APP_UART_HANDLE       (&huart3)
#define NOMAND_APP_PERIOD_MS        10U
#define NOMAND_APP_MAX_BUFFER_SIZE  512U
#define NOMAND_APP_READ_TIMEOUT     1000U

#define NOMAND_APP_CMD_VEL_WHEEL    'c'
#define NOMAD_APP_FEEDBACK_DATA     'r'


#define NOMAND_APP_CMD_CONF_VEL      'w'
#define NOMAND_APP_CMD_CONF_STEERING 's'
#define NOMAND_APP_CMD_CONF_BASE     'b'


#define NOMAND_APP_MAX_PARAMETERS   6U
#define NOMAD_APP_CMD_FORMAT        "%c %d %d %d %d %d %d\n"
#define NOMAD_APP_TX_FORMAT         "%c %d %d %d\n"

/************************************************************************
    FUNCTIONS
************************************************************************/

/**
 * @brief Initialize main robot application.
 */
void UROS_MOTOR_init ();
#endif /* APP_TASK_INCLUDE_NOMAD_MOTOR_TASK_H_ */
