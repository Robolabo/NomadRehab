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
 * @file nomad_wheel_controller.c
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

/************************************************************************
    INCLUDES
************************************************************************/

#include "nomad_wheel_controller.h"

/************************************************************************
    DEFINES AND TYPES
************************************************************************/

#define NOMAD_WHEEL_DT                  ((float)(NOMAD_WHEEL_TASK_PERIOD_MS))/1000.0f

/************************************************************************
    DECLARATIONS
************************************************************************/

static bool NOMAD_WHEEL_isEnabled = false;                   /*<! Enable flag used to enable/disable the controllers */
static Base_Controller_t*  NOMAD_WHEEL_speed_controller;     /*<! Pointer to the speed controller structure */
static Base_Controller_t*  NOMAD_WHEEL_rotation_controller;  /*<! Pointer to the rotation controller structure */

/************************************************************************
    FUNCTIONS
************************************************************************/

/**
 * @brief Main control task
 *
 */
static void NOMAD_WHEEL_TaskFn() {
  float rotation_input = 0.0;
  float rotation_output = 0.0;
  float speed_input = 0.0;
  float speed_output = 0.0;
  float position = 0.0;
  float last_position = 0;

  /* Initialize hardware encoders */
  Encoder_controller_t* rotation_enc = ENC_CONTROL_init(
      NOMAD_WHEEL_ROTATION_ENC,
      NOMAD_WHEEL_ROTATION_CPR,
      NOMAD_WHEEL_ROTATION_REDUCTION);

  Encoder_controller_t* wheel_enc = ENC_CONTROL_init(
      NOMAD_WHEEL_WHEEL_ENC,
      NOMAD_WHEEL_WHEEL_CPR,
      NOMAD_WHEEL_WHEEL_REDUCTION);

  /* Initialize PWM */

  NOMAD_PWM_init();
  NOMAD_PWM_setDuty(NOMAD_WHEEL_WHEEL_PWM_CH, 0.0);
  NOMAD_PWM_setDuty(NOMAD_WHEEL_ROTATION_PWM_CH, 0.0);
  NOMAD_PWM_start(NOMAD_WHEEL_WHEEL_PWM_CH);
  NOMAD_PWM_start(NOMAD_WHEEL_ROTATION_PWM_CH);

  /* Sanity check */
  if ((rotation_enc == NULL) || (wheel_enc == NULL)) {
    while(1);
  }

  ENC_CONTROL_reset(NOMAD_WHEEL_ROTATION_ENC);
  ENC_CONTROL_reset(NOMAD_WHEEL_WHEEL_ENC);
  while (1) {
    /* Get controller inputs */
    rotation_input = ENC_CONTROL_getPostion(NOMAD_WHEEL_ROTATION_ENC);
    position = ENC_CONTROL_getPostion(NOMAD_WHEEL_WHEEL_ENC);

    /* ToDo: Speed also depends on the wheel radius. Update calculation */
    speed_input = (position - last_position)/(NOMAD_WHEEL_DT);
    last_position = position;

    /* Calculate control signals */
    speed_output = NOMAD_WHEEL_speed_controller->functions.execute(NOMAD_WHEEL_speed_controller, speed_input);
    rotation_output = NOMAD_WHEEL_rotation_controller->functions.execute(NOMAD_WHEEL_rotation_controller, rotation_input);

    /* Update motor values */
    if (NOMAD_WHEEL_isEnabled) {
      NOMAD_PWM_setDuty(NOMAD_WHEEL_WHEEL_PWM_CH, speed_output);
      NOMAD_PWM_setDuty(NOMAD_WHEEL_ROTATION_PWM_CH, rotation_output);

    } else {
      /* This allows to move the motor manually */
      NOMAD_PWM_setDuty(NOMAD_WHEEL_WHEEL_PWM_CH, 0.0);
      NOMAD_PWM_setDuty(NOMAD_WHEEL_ROTATION_PWM_CH, 0.0);
    }

    /* Wait until next activation */
    vTaskDelay(pdMS_TO_TICKS(NOMAD_WHEEL_TASK_PERIOD_MS));
  }
}

/**
 * @brief Set controller reference point.
 *
 * @param speed Reference speed.
 * @param rotation reference position.
 */
void NOMAD_WHEEL_setPoint (float speed, float rotation) {

  /* ToDO: protect with mutex */
  NOMAD_WHEEL_speed_controller->params.setpoit = speed;
  NOMAD_WHEEL_rotation_controller->params.setpoit = rotation;
}


/**
 * @brief Get current wheel speed in radians per second.
 *
 * @return Wheel speed in radians per second.
 */
float NOMAD_WHEEL_getSpeed () {
  return NOMAD_WHEEL_speed_controller->params.last_input;
}

/**
 * @brief Get current wheel angle in radians.
 *
 * @return wheel angle in radians.
 */
float NOMAD_WHEEL_getRotation () {
  return NOMAD_WHEEL_rotation_controller->params.last_input;
}

/**
 * @brief Check if the control is enabled.
 *
 * @return True if is enabled, else false.
 */
bool NOMAD_WHEEL_isControlEnabled () {
  return NOMAD_WHEEL_isEnabled;
}

/**
 * @brief Enable the controller.
 *
 */
void NOMAD_WHEEL_enableControl () {
  NOMAD_WHEEL_isEnabled = true;
}

/**
 * @brief Disable the controller.
 *
 */
void NOMAD_WHEEL_disableControl () {
  NOMAD_WHEEL_isEnabled = false;
}

/**
 * @brief Reset the controller. Set the origin.
 *
 */
void NOMAD_WHEEL_reset () {
  /* Disable control */
  NOMAD_WHEEL_disableControl();
  /* Set both PWM to 0 */
  NOMAD_PWM_setDuty(NOMAD_WHEEL_WHEEL_PWM_CH, 0.0);
  NOMAD_PWM_setDuty(NOMAD_WHEEL_ROTATION_PWM_CH, 0.0);

  /* Wait until the motors are complete stopped */
  vTaskDelay(pdMS_TO_TICKS(NOMAD_WHEEL_RESET_TIME_MS));

  /* Reset encoders */
  ENC_CONTROL_reset(NOMAD_WHEEL_WHEEL_ENC);
  ENC_CONTROL_reset(NOMAD_WHEEL_ROTATION_ENC);

  /* Reset controllers */
  NOMAD_WHEEL_rotation_controller->functions.reset(NOMAD_WHEEL_rotation_controller);
  NOMAD_WHEEL_speed_controller->functions.reset(NOMAD_WHEEL_speed_controller);

  /* Enable control */
  NOMAD_WHEEL_enableControl();
}


/**
 * @brief Initialize the wheel controller.
 *
 * @param speed_controller Controller structure used for speed control.
 * @param rotation_controller Controller structure used for ration control.
 */
void NOMAD_WHEEL_Init (Base_Controller_t* speed_controller, Base_Controller_t* rotation_controller) {

  if ((speed_controller != NULL) && (rotation_controller != NULL)) {
    // ToDo: Create software timer watchdog
    // ToDo: Create pose calculation for odometry
    // ToDo: Add sanity check

    NOMAD_WHEEL_rotation_controller = rotation_controller;
    NOMAD_WHEEL_speed_controller = speed_controller;

    /* initialize period constant */
    NOMAD_WHEEL_rotation_controller->params.dt = NOMAD_WHEEL_DT;
    NOMAD_WHEEL_speed_controller->params.dt = NOMAD_WHEEL_DT;


    xTaskCreate(
        NOMAD_WHEEL_TaskFn,
        NOMAD_WHEEL_TASK_NAME,
        NOMAD_WHEEL_TASK_STACK,
        NULL,
        NOMAD_WHEEL_TASK_PRIO,
        NULL);
  }

}

