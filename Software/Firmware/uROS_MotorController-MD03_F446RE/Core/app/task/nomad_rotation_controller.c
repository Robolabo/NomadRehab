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
 * @file nomad_rotation_controller.c
 * @author Alejandro Gomez Molina (@Alejo2313)
 * @brief Rotatory base controller.
 *
 * @version 0.1
 * @date 19 feb. 2023
 *
 * @copyright Copyright (c) 2023
 *
 */

/************************************************************************
    INCLUDES
************************************************************************/
#include "nomad_rotation_controller.h"
/************************************************************************
    PRIVATE DEFINES AND TYPES
************************************************************************/

#define NOMAD_ROTATION_DT                  (((float)(NOMAD_ROTATION_TASK_PERIOD_MS))/1000.0f)
#define NOMAG_WHEEL_MAGIC   0xDEADBEEFU

struct NOMAD_ROTATION_context_s {
  PID_controller_t rotation_controller;
  Encoder_controller_t rotation_encoder;
  uint32_t magicNumber;
};

struct NOMAD_ROTATION_context_s NOMAD_ROTATION_context __attribute__ ((section (".no_init")));

#define NOMAD_ROTATION_IS_VALID_CONTEXT() (NOMAD_ROTATION_context.magicNumber == NOMAG_WHEEL_MAGIC)
#define NOMAD_ROTATION_INVALIDATE_CONTEXT() (NOMAD_ROTATION_context.magicNumber = 0)

/************************************************************************
    PRIVATE DECLARATIONS
************************************************************************/

static bool NOMAD_ROTATION_isEnabled = false;      /*<! Enable flag used to enable/disable the controllers */
static PID_controller_t NOMAD_ROTATION_controller; /*<! Rotation controller structure */
static Encoder_controller_t* NOMAD_ROTATION_rotation_enc;

/************************************************************************
    FUNCTIONS
************************************************************************/

/**
 * @brief Main control task
 *
 */
static void NOMAD_ROTATION_TaskFn() {
  float rotation_input = 0.0;
  float rotation_output = 0.0;

  /* Initialize hardware encoders */
  NOMAD_ROTATION_rotation_enc = ENC_CONTROL_init(
      NOMAD_ROTATION_ENC,
      NOMAD_ROTATION_CPR,
      NOMAD_ROTATION_REDUCTION);

  /* Initialize PWM */

  NOMAD_PWM_init();
  NOMAD_PWM_setDuty(NOMAD_ROTATION_PWM_CH, 0.0);
  NOMAD_PWM_start(NOMAD_ROTATION_PWM_CH);

  /* Initialize encoder */
  if (NOMAD_ROTATION_IS_VALID_CONTEXT()) {
    NOMAD_ROTATION_restoreContext();
    NOMAD_ROTATION_INVALIDATE_CONTEXT();
  }
  else {
    ENC_CONTROL_reset(NOMAD_ROTATION_ENC);
  }



  while (1) {
    /* Get controller inputs */
    rotation_input = ENC_CONTROL_getPostion(NOMAD_ROTATION_ENC);

    /* Calculate control signals */
    rotation_output = NOMAD_ROTATION_controller.base.functions.execute(
        (Base_Controller_t*)&NOMAD_ROTATION_controller,
        rotation_input);

    /* Update motor values */
    if (NOMAD_ROTATION_isEnabled) {
      NOMAD_PWM_setDuty(NOMAD_ROTATION_PWM_CH, rotation_output);

    } else {
      /* This allows to move the motor manually */
      NOMAD_PWM_setDuty(NOMAD_ROTATION_PWM_CH, 0.0);
    }
    /* Wait until next activation */
    vTaskDelay(pdMS_TO_TICKS(NOMAD_ROTATION_TASK_PERIOD_MS));
  }
}

/**
 * @brief Set controller reference point.
 *
 * @param rotation reference position.
 */
void NOMAD_ROTATION_setPoint (float rotation) {
  NOMAD_ROTATION_controller.base.params.setpoit = rotation;
}

/**
 * @brief Get current wheel angle in radians.
 *
 * @return wheel angle in radians.
 */
float NOMAD_ROTATION_getRotation () {
  return NOMAD_ROTATION_controller.base.params.last_input;
}

/**
 * @brief Check if the control is enabled.
 *
 * @return True if is enabled, else false.
 */
bool NOMAD_ROTATION_isControlEnabled () {
  return NOMAD_ROTATION_isEnabled;
}

/**
 * @brief Enable the controller.
 *
 */
void NOMAD_ROTATION_enableControl () {
  NOMAD_ROTATION_isEnabled = true;
}

/**
 * @brief Disable the controller.
 *
 */
void NOMAD_ROTATION_disableControl () {
  NOMAD_ROTATION_isEnabled = false;
}


/**
 * @brief store the context in ram to restore in case of soft reset.
 */
void NOMAD_ROTATION_saveContext () {
  NOMAD_ROTATION_rotation_enc->cnt = NOMAD_ROTATION_rotation_enc->htim->Instance->CNT;

  memcpy(&NOMAD_ROTATION_context.rotation_controller, &NOMAD_ROTATION_controller, sizeof(PID_controller_t));
  memcpy(&NOMAD_ROTATION_context.rotation_encoder, NOMAD_ROTATION_rotation_enc, sizeof(Encoder_controller_t));

  NOMAD_ROTATION_context.magicNumber = NOMAG_WHEEL_MAGIC;
}

/**
 * @brief Restore context.
 */
void NOMAD_ROTATION_restoreContext () {
  memcpy(&NOMAD_ROTATION_controller, &NOMAD_ROTATION_context.rotation_controller, sizeof(PID_controller_t));
  memcpy(NOMAD_ROTATION_rotation_enc, &NOMAD_ROTATION_context.rotation_encoder, sizeof(Encoder_controller_t));

  NOMAD_ROTATION_rotation_enc->htim->Instance->CNT = NOMAD_ROTATION_rotation_enc->cnt;
}

/**
 * @brief Reset the controller. Set the origin.
 *
 */
void NOMAD_ROTATION_reset () {
  /* Disable control */
  NOMAD_ROTATION_disableControl();
  /* Set both PWM to 0 */
  NOMAD_PWM_setDuty(NOMAD_ROTATION_PWM_CH, 0.0);

  /* Wait until the motors are complete stopped */
  vTaskDelay(pdMS_TO_TICKS(NOMAD_ROTATION_RESET_TIME_MS));

  /* Reset encoders */
  ENC_CONTROL_reset(NOMAD_ROTATION_ENC);

  /* Reset controllers */
  NOMAD_ROTATION_controller.base.functions.reset((Base_Controller_t*)&NOMAD_ROTATION_controller);

  /* Enable control */
  NOMAD_ROTATION_enableControl();
}

/**
 * @brief Initialize rotation controller task.
 *
 */
void NOMAD_ROTATION_Init () {

  /* Initialize encoder controller */
  PID_CONTROLLER_Init(
      &NOMAD_ROTATION_controller,
      NOMAD_ROTATION_KP,
      NOMAD_ROTATION_KD,
      NOMAD_ROTATION_KI,
      NOMAD_ROTATION_LIMIT);
  /* ToDO: include this in the Controller init function */
  NOMAD_ROTATION_controller.base.params.dt = NOMAD_ROTATION_DT;

  /* Initialize main task */
  xTaskCreate(
      NOMAD_ROTATION_TaskFn,
      NOMAD_ROTATION_TASK_NAME,
      NOMAD_ROTATION_TASK_STACK,
      NULL,
      NOMAD_ROTATION_TASK_PRIO,
      NULL);
}
