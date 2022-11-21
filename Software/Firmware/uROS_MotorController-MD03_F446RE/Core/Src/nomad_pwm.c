/*
 * nomad_motor.c
 *
 *  Created on: Nov 21, 2022
 *      Author: alejo
 */

#include "nomad_pwm.h"


void NOMAD_PWM_init() {
  HAL_TIM_Base_Start(&htim1);
}

void NOMAD_PWM_start(NOMAD_PWM_Channel_t channel) {
  if (NOMAD_PWM_IS_CHANNEL(channel)) {
    HAL_TIM_PWM_Start(&htim1, channel);
  }
}


void NOMAD_PWM_setDuty(NOMAD_PWM_Channel_t channel, float duty) {

  uint16_t autoreload = 0;
  NOMAD_PWM_Direction_t direction = NOMAD_PWM_DIR_FORWARD;

  if (!NOMAD_PWM_IS_CHANNEL(channel)) {
    return;
  }

  if (duty < 0) {
    direction = NOMAD_PWM_DIR_BACKWARD;
    duty = -duty;
  }

  if (duty >= 100) {
    duty = 100;
  }

  autoreload = __HAL_TIM_GET_AUTORELOAD(&htim1);
  float aux = (duty*(float)(autoreload))/100.0;
  __HAL_TIM_SET_COMPARE(&htim1, channel, (uint32_t)(aux));
  NOMAD_PWM_set_direction(channel, direction);
}

float NOMAD_PWM_get_duty_cycle(NOMAD_PWM_Channel_t channel) {
  uint16_t autoreload_reg = 1U;
  uint16_t compare_reg = 0U;
  float duty = 0.0;

  if (NOMAD_PWM_IS_CHANNEL(channel)) {
    autoreload_reg = __HAL_TIM_GET_AUTORELOAD(&htim1);
    compare_reg = __HAL_TIM_GET_COMPARE(&htim1, channel);

    duty = (float)(compare_reg*100)/(float)(autoreload_reg);
  }
  return duty;
}

void NOMAD_PWM_set_direction(NOMAD_PWM_Channel_t channel, NOMAD_PWM_Direction_t dir) {
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

