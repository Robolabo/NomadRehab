/*
 * nomad_encoder.c
 *
 *  Created on: Nov 27, 2022
 *      Author: agome
 */


#include "nomad_encoder.h"


static NOMAD_Encoder_data_t NOMAD_encoders[NOMAD_ENCODER_NUMBER];


static void NOMAD_ENCODER_callback(TIM_HandleTypeDef* timer) {
  uint8_t index = 0;
  for (index = 0; index < NOMAD_ENCODER_NUMBER; index++) {
    if (NOMAD_encoders[index].timer == timer) {
      if (__HAL_TIM_IS_TIM_COUNTING_DOWN(timer)) {
        NOMAD_encoders[index].revolution_counter--;
      }
      else {
        NOMAD_encoders[index].revolution_counter++;
      }
    }
    break;
  }
}


void NOMAD_ENCODER_init(TIM_HandleTypeDef* timer, NOMAD_Encoder_Index_t index, uint16_t cpr, float reduction) {
  uint32_t count_per_revolution =  (uint32_t)(cpr*reduction);

  if ((timer != NULL) && (index < NOMAD_ENCODER_NUMBER)) {
    if (count_per_revolution <= 0xFFFF) {

      __HAL_TIM_SET_AUTORELOAD(timer,(uint16_t)(count_per_revolution));

      memset(&NOMAD_encoders[index], 0x00, sizeof(NOMAD_Encoder_data_t));
      NOMAD_encoders[index].timer = timer;
      NOMAD_encoders[index].count_per_revolution = count_per_revolution;
      __HAL_TIM_SET_COUNTER(timer, 0U);
      __HAL_TIM_ENABLE_IT(timer, TIM_IT_UPDATE);
      timer->PeriodElapsedCallback = NOMAD_ENCODER_callback;
      HAL_TIM_Encoder_Start(timer, TIM_CHANNEL_ALL);
    }
  }
}

float NOMAD_ENCODER_get_position(NOMAD_Encoder_Index_t index) {
  float position = 0.0;
  uint16_t counter_reg = 0.0;
  if ((index < NOMAD_ENCODER_NUMBER) && (NOMAD_encoders[index].timer != NULL)) {

    counter_reg = __HAL_TIM_GET_COUNTER(NOMAD_encoders[index].timer);
    position = NOMAD_encoders[index].revolution_counter;
    position += ((float)(counter_reg)/(float)(NOMAD_encoders[index].count_per_revolution));
    position = position*2*NOMAD_PI;
  }
  return position;
}

void NOMAD_ENCODER_reset_position(NOMAD_Encoder_Index_t index) {
  if ((index < NOMAD_ENCODER_NUMBER) && (NOMAD_encoders[index].timer != NULL)) {
    NOMAD_encoders[index].revolution_counter = 0U;
    __HAL_TIM_SET_COUNTER(NOMAD_encoders[index].timer, 1U);
  }
}

