/*
 * encoder_controller.h
 *
 *  Created on: 12 feb. 2023
 *      Author: Alejo
 */

#ifndef INC_ENCODER_CONTROLLER_H_
#define INC_ENCODER_CONTROLLER_H_

#include "stm32f4xx_hal.h"
#include <math.h>


#define ENC_CONTROL_MAX_ENCODERS  4U    /*<! Maximum number of encoders that the controller can handle */


/**
 * @brief Encoder handle structure.
 *
 */
typedef struct {
  TIM_HandleTypeDef* htim;        /*<! Associated timer handle*/
  uint32_t countPerRevolution;    /*<! Encoder resolution */
  float reduction;                /*<! Motor speed reduction factor */
  uint32_t revolutions;           /*<! Total number of revolutions */

} Encoder_controller_t;



/************** Prototypes **************/

Encoder_controller_t* ENC_CONTROL_init (
    TIM_HandleTypeDef* htim,
    uint32_t countPerRevolution,
    float reductionFactor);

void ENC_CONTROL_reset (TIM_HandleTypeDef* htim);

double ENC_CONTROL_getPostion (TIM_HandleTypeDef* htim);



#endif /* INC_ENCODER_CONTROLLER_H_ */
