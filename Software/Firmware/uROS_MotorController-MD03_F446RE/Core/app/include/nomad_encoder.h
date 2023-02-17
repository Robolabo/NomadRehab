/*
 * nomad_encoder.h
 *
 *  Created on: Nov 27, 2022
 *      Author: agome
 */

#ifndef INC_NOMAD_ENCODER_H_
#define INC_NOMAD_ENCODER_H_

#include <stdint.h>
#include <string.h>
#include "tim.h"


#define NOMAD_ENCODER_NUMBER  3U
#define NOMAD_PI              3.14

typedef struct {
  TIM_HandleTypeDef* timer;
  uint16_t count_per_revolution;

  uint32_t revolution_counter;
  uint32_t revolution_counter_prev;
  uint16_t timer_counter_prev;
}NOMAD_Encoder_data_t;


typedef enum {
  NOMAD_ENCODER_1,
  NOMAD_ENCODER_2,
  NOMAD_ENCODER_3
}NOMAD_Encoder_Index_t;

void NOMAD_ENCODER_init(TIM_HandleTypeDef* timer, NOMAD_Encoder_Index_t index, uint16_t cpr, float reduction);
float NOMAD_ENCODER_get_position(NOMAD_Encoder_Index_t index);
void NOMAD_ENCODER_reset_position(NOMAD_Encoder_Index_t index);


#endif /* INC_NOMAD_ENCODER_H_ */
