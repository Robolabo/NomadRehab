/*
 * nomad_pwm.h
 *
 *  Created on: Nov 21, 2022
 *      Author: alejo
 */

#ifndef INC_NOMAD_NOMAD_PWM_H_
#define INC_NOMAD_NOMAD_PWM_H_

#include "tim.h"
#include "gpio.h"
#include "main.h"


/**
 * @brief Nomad PWM channels.
 *
 */
typedef enum {
  NOMAD_PWM_CHANNEL_1 = TIM_CHANNEL_1, /**< NOMAD_PWM_CHANNEL_1 */
  NOMAD_PWM_CHANNEL_2 = TIM_CHANNEL_2, /**< NOMAD_PWM_CHANNEL_2 */
  NOMAD_PWM_CHANNEL_3 = TIM_CHANNEL_3, /**< NOMAD_PWM_CHANNEL_3 */
}NOMAD_PWM_Channel_t;


/**
 * @brief NOMAD direction.
 *
 */
typedef enum {
  NOMAD_PWM_DIR_FORWARD  = 0U,  /**< NOMAD_PWM_DIR_FORWARD */
  NOMAD_PWM_DIR_BACKWARD = 1U   /**< NOMAD_PWM_DIR_BACKWARD */
}NOMAD_PWM_Direction_t;



/********* Helper macros *********/

/**
 * @brief Check if is a correct Nomad PWM channel
 *
 */
#define NOMAD_PWM_IS_CHANNEL(CHANNEL)\
  ((CHANNEL) == NOMAD_PWM_CHANNEL_1) ||\
  ((CHANNEL) == NOMAD_PWM_CHANNEL_2) ||\
  ((CHANNEL) == NOMAD_PWM_CHANNEL_3)



/********* Hardware configuration *********/


#define  NOMAD_TIMER_HANDLE  htim1
#define DIR1    PWM_DIR_1_GPIO_Port, PWM_DIR_1_Pin
#define DIR2    PWM_DIR_2_GPIO_Port, PWM_DIR_2_Pin
#define DIR3    PWM_DIR_3_GPIO_Port, PWM_DIR_3_Pin


/********** Prototyped **********/

void NOMAD_PWM_init();
void NOMAD_PWM_start(NOMAD_PWM_Channel_t channel);
void NOMAD_PWM_setDuty(NOMAD_PWM_Channel_t channel, float duty);
float NOMAD_PWM_get_duty_cycle(NOMAD_PWM_Channel_t channel);



#endif /* INC_NOMAD_NOMAD_PWM_H_ */
