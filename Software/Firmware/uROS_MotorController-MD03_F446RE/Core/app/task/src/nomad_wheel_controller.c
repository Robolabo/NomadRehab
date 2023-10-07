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

#define NOMAD_WHEEL_DT                  ((float)(NOMAD_WHEEL_TASK_PERIOD_MS))/1000.0f   /*<! time between to consecutive measurements */
#define NOMAG_WHEEL_MAGIC               0xDEADBEEFU                                     /*<! Magic number: no random sequence */

/**
 * Wheel context structure.
 * Stores the current state in case of soft-reset.
 */

struct NOMAD_WHEEL_context_s {
  PID_controller_t speed_controller;
  PID_controller_t rotation_controller;
  Encoder_controller_t speed_encoder;
  Encoder_controller_t rotation_encoder;
  uint32_t magicNumber; /*<! Delimiter. Used to check if it is valid context */
};


#define NOMAD_WHEEL_IS_VALID_CONTEXT() (NOMAD_WHEEL_context.magicNumber == NOMAG_WHEEL_MAGIC) /*<! Check if is a valid context by checking the magic number */
#define NOMAD_WHEEL_INVALIDATE_CONTEXT() (NOMAD_WHEEL_context.magicNumber = 0U)               /*<! Reset the magic number to invalidate the context */

/* Debug purpose */
static TickType_t max_control_ticks = 0;
#define MAX_TICKS(tick) max_control_ticks = (max_control_ticks < (tick) ? (tick) : max_control_ticks)

/************************************************************************
    DECLARATIONS
************************************************************************/

static bool NOMAD_WHEEL_isEnabled = false;                       /*<! Enable flag used to enable/disable the controllers */

static PID_controller_t NOMAD_WHEEL_speed_controller;            /*<! PID speed controller */
static PID_controller_t NOMAD_WHEEL_rotation_controller;         /*<! PID rotation controller */
static Encoder_controller_t* NOMAD_WHEEL_wheel_enc;              /*<! Speed encoder handle */
static Encoder_controller_t* NOMAD_WHEEL_rotation_enc;           /*<! Rotation encoder handle */



static Base_Controller_t*  NOMAD_WHEEL_speed_controller_ptr =
    (Base_Controller_t*)&NOMAD_WHEEL_speed_controller;     /*<! Pointer to the speed controller structure */

static Base_Controller_t*  NOMAD_WHEEL_rotation_controller_ptr =
    (Base_Controller_t*)&NOMAD_WHEEL_rotation_controller;  /*<! Pointer to the rotation controller structure */



/* ToDo: use this instead the previous definitions? */
struct NOMAD_WHEEL_context_s NOMAD_WHEEL_context __attribute__ ((section (".no_init")));  /*<! Context. Allocated in no_init section to */
                                                                                          /*<! avoid overriding in case of soft reset */

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
  float last_position = 0.0;

  TickType_t elapsed_ticks = 0;

  /* Initialize PWM */

  NOMAD_PWM_init();
  NOMAD_PWM_setDuty(NOMAD_WHEEL_WHEEL_PWM_CH, 0.0);
  NOMAD_PWM_setDuty(NOMAD_WHEEL_ROTATION_PWM_CH, 0.0);
  NOMAD_PWM_start(NOMAD_WHEEL_WHEEL_PWM_CH);
  NOMAD_PWM_start(NOMAD_WHEEL_ROTATION_PWM_CH);

  /* Sanity check */
  if ((NOMAD_WHEEL_rotation_enc == NULL) || (NOMAD_WHEEL_wheel_enc == NULL)) {
    while(1);
  }

  ENC_CONTROL_reset(NOMAD_WHEEL_ROTATION_ENC);
  ENC_CONTROL_reset(NOMAD_WHEEL_WHEEL_ENC);

  while (1) {
    elapsed_ticks = xTaskGetTickCount();
    /* Get controller inputs */
    rotation_input = ENC_CONTROL_getPostion(NOMAD_WHEEL_ROTATION_ENC);
    position = ENC_CONTROL_getPostion(NOMAD_WHEEL_WHEEL_ENC);

    speed_input = (position - last_position)*(WHEEL_RADIUS_M)/(NOMAD_WHEEL_DT);

    /* Calculate control signals */
    speed_output = NOMAD_WHEEL_speed_controller_ptr->functions.execute(NOMAD_WHEEL_speed_controller_ptr, speed_input);
    rotation_output = NOMAD_WHEEL_rotation_controller_ptr->functions.execute(NOMAD_WHEEL_rotation_controller_ptr, rotation_input);

    /* Update motor values */
    if (NOMAD_WHEEL_isEnabled) {
      NOMAD_PWM_setDuty(NOMAD_WHEEL_WHEEL_PWM_CH, speed_output);
      NOMAD_PWM_setDuty(NOMAD_WHEEL_ROTATION_PWM_CH, rotation_output);

    } else {
      /* This allows to move the motor manually */
      NOMAD_PWM_setDuty(NOMAD_WHEEL_WHEEL_PWM_CH, 0.0);
      NOMAD_PWM_setDuty(NOMAD_WHEEL_ROTATION_PWM_CH, 0.0);
    }

    last_position = position;

    elapsed_ticks = xTaskGetTickCount() - elapsed_ticks;
    MAX_TICKS(elapsed_ticks);
    /* Wait until next activation */
    vTaskDelay(pdMS_TO_TICKS(NOMAD_WHEEL_TASK_PERIOD_MS) - elapsed_ticks);
  }
}

void NOMAD_WHEEL_setVelocity (int velocity)
{
  NOMAD_WHEEL_speed_controller_ptr->params.setpoit = (float)(velocity)/1000.0;
}

void NOMAD_WHEEL_setSteering (int rotation)
{
  NOMAD_WHEEL_rotation_controller_ptr->params.setpoit = (float)(rotation)/1000.0;
}


/**
 * @brief Get current wheel speed in radians per second.
 */
int NOMAD_WHEEL_getSpeed () {
  return (int)(NOMAD_WHEEL_speed_controller_ptr->params.last_input * 1000.0);
}

/**
 * @brief Get current wheel angle in radians.
 */
int NOMAD_WHEEL_getRotation () {
  return (int)(NOMAD_WHEEL_rotation_controller_ptr->params.last_input * 1000.0);
}


/**
 * @brief Check if the control is enabled.
 */
bool NOMAD_WHEEL_isControlEnabled () {
  return NOMAD_WHEEL_isEnabled;
}

/**
 * @brief Enable the controller.
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
  NOMAD_PWM_setDuty(NOMAD_WHEEL_WHEEL_PWM_CH, 0.0);
  NOMAD_PWM_setDuty(NOMAD_WHEEL_ROTATION_PWM_CH, 0.0);
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
  NOMAD_WHEEL_rotation_controller_ptr->functions.reset(NOMAD_WHEEL_rotation_controller_ptr);
  NOMAD_WHEEL_speed_controller_ptr->functions.reset(NOMAD_WHEEL_speed_controller_ptr);

  /* Enable control */
  NOMAD_WHEEL_enableControl();
}

void NOMAD_WHEEL_setTractionPID (
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

  NOMAD_WHEEL_disableControl();
  /* Initialize PID controller */
  PID_CONTROLLER_Init(
    &NOMAD_WHEEL_speed_controller, f_kp, f_kd, f_ki, f_limit);
  /* Initialize encoder */
  NOMAD_WHEEL_wheel_enc =
      ENC_CONTROL_init(NOMAD_WHEEL_WHEEL_ENC, ui_cpr, f_reduction);

  NOMAD_WHEEL_speed_controller_ptr->params.dt = NOMAD_WHEEL_DT;
  NOMAD_WHEEL_speed_controller_ptr->functions.reset(NOMAD_WHEEL_speed_controller_ptr);
}


void NOMAD_WHEEL_setSteeringPID (
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

  NOMAD_WHEEL_disableControl();
  /* Initialize PID controller */
  PID_CONTROLLER_Init(
    &NOMAD_WHEEL_rotation_controller, f_kp, f_kd, f_ki, f_limit);

  /* Initialize encoder */
  NOMAD_WHEEL_rotation_enc =
      ENC_CONTROL_init(NOMAD_WHEEL_ROTATION_ENC, ui_cpr, f_reduction);

  NOMAD_WHEEL_rotation_controller_ptr->params.dt = NOMAD_WHEEL_DT;
  NOMAD_WHEEL_rotation_controller_ptr->functions.reset(NOMAD_WHEEL_rotation_controller_ptr);
}



/**
 * @brief Initialize the wheel controller.
 */
void NOMAD_WHEEL_Init () {


  /* Initialize controllers */
  NOMAD_WHEEL_setSteeringPID (
      NOMAD_WHEEL_WHEEL_KP * 1000, NOMAD_WHEEL_WHEEL_KD * 1000,
      NOMAD_WHEEL_WHEEL_KD * 1000, NOMAD_WHEEL_WHEEL_LIMIT * 1000,
      NOMAD_WHEEL_WHEEL_CPR, NOMAD_WHEEL_WHEEL_REDUCTION * 1000);

  NOMAD_WHEEL_setTractionPID (
      NOMAD_WHEEL_ROTATION_KP * 1000, NOMAD_WHEEL_ROTATION_KD * 1000,
      NOMAD_WHEEL_ROTATION_KD * 1000, NOMAD_WHEEL_ROTATION_LIMIT * 1000,
      NOMAD_WHEEL_ROTATION_CPR, NOMAD_WHEEL_ROTATION_REDUCTION * 1000);

  NOMAD_WHEEL_reset();

  xTaskCreate(
      NOMAD_WHEEL_TaskFn,
      NOMAD_WHEEL_TASK_NAME,
      NOMAD_WHEEL_TASK_STACK,
      NULL,
      NOMAD_WHEEL_TASK_PRIO,
      NULL);
}

