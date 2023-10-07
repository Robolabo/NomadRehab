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
 * @file nomad_motor_task.c
 * @author Alejo
 * @brief 
 *
 * @version 0.1
 * @date Oct 5, 2023
 *
 * @copyright Copyright (c) 2023
 *
 */

/************************************************************************
    INCLUDES
************************************************************************/
#include "nomad_motor_task.h"

/************************************************************************
    PRIVATE DEFINES AND TYPES
************************************************************************/

/************************************************************************
    PRIVATE DECLARATIONS
************************************************************************/

/************************************************************************
    FUNCTIONS
************************************************************************/

/**
 * @brief
 *
 * @param param
 */
void NOMAD_APP_processInputTaskFn (void* param)
{
  size_t len = 0;
  char command;
  int parametersLen;
  int parameters[NOMAND_APP_MAX_PARAMETERS];
  uint8_t buffer[NOMAND_APP_MAX_BUFFER_SIZE];

  while(1) {
    len = serial_readLine(buffer, sizeof(buffer) - 1, NOMAND_APP_READ_TIMEOUT);
    /* Make sure the frame is complete */
    if ((len > 0) && (buffer[len - 1] == '\n')) {
      buffer[len] = '\0';
      /* Parse incoming frame */
      parametersLen = sscanf(
        (char*)(buffer),
        NOMAD_APP_CMD_FORMAT,
        &command,
        &parameters[0],
        &parameters[1],
        &parameters[2],
        &parameters[3],
        &parameters[4],
        &parameters[5]);

      switch (command) {
        case NOMAND_APP_CMD_VEL_WHEEL:
          if (parametersLen == 4) {
            /*Send command parameters*/
            NOMAD_WHEEL_setVelocity(parameters[0]);
            NOMAD_WHEEL_setSteering(parameters[1]);
            NOMAD_ROTATION_setPoint(parameters[2]);
          }
          break;

        case NOMAD_APP_FEEDBACK_DATA:
          if (parametersLen == 1) {
            parameters[0] = NOMAD_WHEEL_getSpeed();
            parameters[1] = NOMAD_WHEEL_getRotation();
            parameters[2] = NOMAD_ROTATION_getRotation();

            len = snprintf((char*)(buffer),
              sizeof(buffer),
              NOMAD_APP_TX_FORMAT,
              NOMAD_APP_FEEDBACK_DATA,
              parameters[0], parameters[1], parameters[2]);

            serial_write(buffer, len, NOMAND_APP_READ_TIMEOUT);
          }
          break;
        case NOMAND_APP_CMD_CONF_VEL:
          if (parametersLen == 7) {
            NOMAD_WHEEL_setTractionPID(
                parameters[0],
                parameters[1],
                parameters[2],
                parameters[3],
                parameters[4],
                parameters[5]);

            NOMAD_WHEEL_enableControl();

          }
          break;
        case NOMAND_APP_CMD_CONF_STEERING:
          if (parametersLen == 7) {
            NOMAD_WHEEL_setSteeringPID(
                parameters[0],
                parameters[1],
                parameters[2],
                parameters[3],
                parameters[4],
                parameters[5]);

            NOMAD_WHEEL_enableControl();

          }
          break;
        case NOMAND_APP_CMD_CONF_BASE:
          if (parametersLen == 7) {
            NOMAD_ROTATION_setPID(
                parameters[0],
                parameters[1],
                parameters[2],
                parameters[3],
                parameters[4],
                parameters[5]);

            NOMAD_ROTATION_enableControl();
          }
          break;
        default:
          break;
      }
    }
  }
}



/**
 * @brief
 */
void NOMAD_APP_setup () {
  serial_open(NOMAD_APP_UART_HANDLE);
  NOMAD_WHEEL_Init();
  NOMAD_ROTATION_Init();
}

/**
 * @brief Initalize Nomad task
 */
void UROS_MOTOR_init () {
  /* ToDo: Check if the task has been created correctly */
  NOMAD_APP_setup();

  xTaskCreate(
    NOMAD_APP_processInputTaskFn,
    NOMAD_APP_TASK1_NAME,
    NOMAD_APP_TASK1_STACK,
    NULL,
    NOMAD_APP_TASK1_PRIO,
    NULL);
}
