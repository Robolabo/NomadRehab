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

#define NOMAD_ROTATION_DT (((float)(NOMAD_ROTATION_TASK_PERIOD_MS))/1000.0f) /*<! time between to consecutive measurements */
#define NOMAG_WHEEL_MAGIC 0xDEADBEEFU                                        /*<! Magic number: no random sequence */

/**
 * Wheel context structure.
 * Stores the current state in case of soft-reset.
 */
struct NOMAD_ROTATION_context_s {
  PID_controller_t rotation_controller;
  Encoder_controller_t rotation_encoder;
  uint32_t magicNumber;
};

#define NOMAD_ROTATION_IS_VALID_CONTEXT() (NOMAD_ROTATION_context.magicNumber == NOMAG_WHEEL_MAGIC) /*<! Check if is a valid context by checking the magic number */
#define NOMAD_ROTATION_INVALIDATE_CONTEXT() (NOMAD_ROTATION_context.magicNumber = 0)                /*<! Reset the magic number to invalidate the context */

/************************************************************************
    PRIVATE DECLARATIONS
************************************************************************/

static bool NOMAD_ROTATION_isEnabled = false;             /*<! Enable flag used to enable/disable the controllers */
static PID_controller_t NOMAD_ROTATION_controller;        /*<! Rotation controller structure */
static Encoder_controller_t* NOMAD_ROTATION_rotation_enc; /*<! Associated rotation encoder handle */

struct NOMAD_ROTATION_context_s NOMAD_ROTATION_context __attribute__ ((section (".no_init"))); /*<! Context. Allocated in no_init section to */
                                                                                               /*<! avoid overriding in case of soft reset */

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
  TickType_t elapsed_ticks = 0;

  /* Initialize PWM */
  NOMAD_PWM_init();
  NOMAD_PWM_setDuty(NOMAD_ROTATION_PWM_CH, 0.0);
  NOMAD_PWM_start(NOMAD_ROTATION_PWM_CH);

  ENC_CONTROL_reset(NOMAD_ROTATION_ENC);

  while (1) {
    elapsed_ticks = xTaskGetTickCount();
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

    elapsed_ticks = xTaskGetTickCount() - elapsed_ticks;
    /* Wait until next activation */
    vTaskDelay(pdMS_TO_TICKS(NOMAD_ROTATION_TASK_PERIOD_MS));
  }
}

/**
 * @brief Set controller reference point.
 */
void NOMAD_ROTATION_setPoint (int rotation) {
  NOMAD_ROTATION_controller.base.params.setpoit = (float)(rotation)/1000.0;
}

/**
 * @brief Get current wheel angle in radians.
 *
 * @return wheel angle in radians.
 */
int NOMAD_ROTATION_getRotation () {
  return (int)(NOMAD_ROTATION_controller.base.params.last_input * 1000.0);
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
  NOMAD_PWM_setDuty(NOMAD_ROTATION_PWM_CH, 0.0);
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



void NOMAD_ROTATION_setPID (
    int kp, int kd,
    int ki, int limit,
    uint32_t cpr, int reduction)
{
  float f_kp = ((float)kp)/1000.0;
  float f_kd = ((float)kd)/1000.0;
  float f_ki = ((float)ki)/1000.0;
  float f_limit = ((float)limit)/1000.0;
  uint32_t ui_cpr = ((uint32_t)cpr);
  float f_reduction = ((float)reduction)/1000.0;

  NOMAD_ROTATION_disableControl();
  /* Initialize PID controller */
  PID_CONTROLLER_Init(
    &NOMAD_ROTATION_controller, f_kp, f_kd, f_ki, f_limit);

  /* Initialize encoder */
  NOMAD_ROTATION_rotation_enc =
      ENC_CONTROL_init(NOMAD_ROTATION_ENC, ui_cpr, f_reduction);
  NOMAD_ROTATION_controller.base.params.dt = NOMAD_ROTATION_DT;
  NOMAD_ROTATION_reset();
}

/**
 * @brief Initialize rotation controller task.
 *
 */
void NOMAD_ROTATION_Init () {

  NOMAD_ROTATION_setPID (
      NOMAD_ROTATION_KP * 1000, NOMAD_ROTATION_KD * 1000,
      NOMAD_ROTATION_KD * 1000, NOMAD_ROTATION_LIMIT * 1000,
      NOMAD_ROTATION_CPR, NOMAD_ROTATION_REDUCTION * 1000);


  /* Initialize main task */
  xTaskCreate(
      NOMAD_ROTATION_TaskFn,
      NOMAD_ROTATION_TASK_NAME,
      NOMAD_ROTATION_TASK_STACK,
      NULL,
      NOMAD_ROTATION_TASK_PRIO,
      NULL);
}
