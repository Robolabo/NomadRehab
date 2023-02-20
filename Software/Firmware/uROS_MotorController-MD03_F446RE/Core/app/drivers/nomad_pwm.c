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
 * @file nomad_pwm.c
 * @author Alejandro Gomez Molina (@Alejo2313)
 * @brief Nomad PWM controller.
 *
 * @version 0.1
 * @date Nov 21, 2022
 *
 * @copyright Copyright (c) 2023
 *
 */

/************************************************************************
    INCLUDES
************************************************************************/
#include "nomad_pwm.h"

/************************************************************************
    PRIVATE DECLARATIONS
************************************************************************/

static void NOMAD_PWM_set_direction(NOMAD_PWM_Channel_t channel, NOMAD_PWM_Direction_t dir);

/************************************************************************
    FUNCTIONS
************************************************************************/

/**
 * @brief Initialize PWM timer.
 *        WARNING: The timer handler must be initialized.
 *
 */
void NOMAD_PWM_init () {
  HAL_TIM_Base_Start(&NOMAD_TIMER_HANDLE);
}

/**
 * @brief Start PWM for a given channel.
 *
 * @param channel PWM channel to initialize.
 */
void NOMAD_PWM_start(NOMAD_PWM_Channel_t channel) {
  if (NOMAD_PWM_IS_CHANNEL(channel)) {
    HAL_TIM_PWM_Start(&NOMAD_TIMER_HANDLE, channel);
  }
}

/**
 * @brief Set the dutyCycle for a given channel.
 *        the direction is implicit in the duty sign.
 *
 * @param channel PWM channel.
 * @param duty Duty clycle (-100.0 - 100.0)
 */
void NOMAD_PWM_setDuty(NOMAD_PWM_Channel_t channel, float duty) {
  NOMAD_PWM_Direction_t direction = NOMAD_PWM_DIR_FORWARD;
  uint16_t autoreload = 0;

  /* Sanity check */
#if 0 /* ToDo: this condition is not working */
  if (!NOMAD_PWM_IS_CHANNEL(channel)) {
    return;
  }
#endif
  /* Check motor direction */
  if (duty < 0.0) {
    direction = NOMAD_PWM_DIR_BACKWARD;
    duty = -duty;
  }

  /* Check duty boundaries */
  if (duty >= 100.0) {
    duty = 100;
  }

  /* Scale ARR value according to the duty cycle */
  autoreload = __HAL_TIM_GET_AUTORELOAD(&NOMAD_TIMER_HANDLE);
  float aux = (duty*(float)(autoreload))/100.0;

  /* Update compare register and motor direction*/
  __HAL_TIM_SET_COMPARE(&NOMAD_TIMER_HANDLE, channel, (uint32_t)(aux));
  NOMAD_PWM_set_direction(channel, direction);
}



/**
 * @brief Get the current duty cycle.
 *
 * @param channel
 * @return
 */
float NOMAD_PWM_get_duty_cycle(NOMAD_PWM_Channel_t channel) {
  uint16_t autoreload_reg = 1U;
  uint16_t compare_reg = 0U;
  float duty = 0.0;

  if (NOMAD_PWM_IS_CHANNEL(channel)) {
    autoreload_reg = __HAL_TIM_GET_AUTORELOAD(&NOMAD_TIMER_HANDLE);
    compare_reg = __HAL_TIM_GET_COMPARE(&NOMAD_TIMER_HANDLE, channel);

    duty = (float)(compare_reg*100)/(float)(autoreload_reg);
  }
  return duty;
}


/**
 * @brief Update motor direction GPIO.
 *        The HW GPIO configuration is in the
 *        header file.
 *
 * @param channel Channel to be updated.
 * @param dir Motor direction (@see NOMAD_PWM_Direction_t).
 */
static void NOMAD_PWM_set_direction(NOMAD_PWM_Channel_t channel, NOMAD_PWM_Direction_t dir) {
  switch (channel) {
    case NOMAD_PWM_CHANNEL_1:
      HAL_GPIO_WritePin(DIR1, dir);
      break;
    case NOMAD_PWM_CHANNEL_2:
      HAL_GPIO_WritePin(DIR2, dir);
      break;
    case NOMAD_PWM_CHANNEL_3:
      HAL_GPIO_WritePin(DIR3, dir);
      break;
    default:
      break;
  }
}

