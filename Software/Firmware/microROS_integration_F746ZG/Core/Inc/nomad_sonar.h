/*
 * nomad_sonar.h
 *
 *  Created on: Oct 27, 2022
 *      Author: agome
 */

#ifndef INC_NOMAD_SONAR_H_
#define INC_NOMAD_SONAR_H_

#include "stm32f7xx_hal.h"
#include "FreeRTOS.h"
#include "semphr.h"
#include "queue.h"

/* Configuration */
#define SN_STABILIZATION_TIME_MS   10U
#define SN_ECHO_TIMEOUT_US         0xFFFFU  /* 10.7 m at 340 m/s */
#define SN_MEASURE_TIMEOUT         ((SN_ECHO_TIMEOUT_US)/1000U) + (STABILIZATION_TIME_MS)
#define SN_ECHO_TIMEOUT_MS         ((SN_ECHO_TIMEOUT_US)/1000U) + 1U

/* 6500 Ranging Module control pins */
#define ECHO_Pin 	      GPIO_PIN_2
#define ECHO_GPIO_Port  GPIOD
#define ECHO_EXTI_IRQn 	EXTI2_IRQn
#define ECHO_EN_Clk     __HAL_RCC_GPIOD_CLK_ENABLE

#define INIT_Pin 		    GPIO_PIN_12
#define INIT_GPIO_Port 	GPIOC
#define INIT_EN_Clk     __HAL_RCC_GPIOC_CLK_ENABLE

#define BINH_Pin 		    GPIO_PIN_3
#define BINH_GPIO_Port 	GPIOG
#define BINH_EN_Clk     __HAL_RCC_GPIOG_CLK_ENABLE

#define BLNK_Pin 		    GPIO_PIN_2
#define BLNK_GPIO_Port 	GPIOG
#define BLNK_EN_Clk     __HAL_RCC_GPIOG_CLK_ENABLE

/* Sonar selector GPIOs */
#define SN_A0_Pin 		  GPIO_PIN_8
#define SN_A0_GPIO_Port GPIOC
#define SN_A0_EN_Clk    __HAL_RCC_GPIOC_CLK_ENABLE

#define SN_A1_Pin 		  GPIO_PIN_9
#define SN_A1_GPIO_Port GPIOC
#define SN_A1_EN_Clk    __HAL_RCC_GPIOC_CLK_ENABLE

#define SN_A2_Pin 		  GPIO_PIN_10
#define SN_A2_GPIO_Port GPIOC
#define SN_A2_EN_Clk    __HAL_RCC_GPIOC_CLK_ENABLE

#define SN_A3_Pin 		  GPIO_PIN_11
#define SN_A3_GPIO_Port GPIOC
#define SN_A3_EN_Clk    __HAL_RCC_GPIOC_CLK_ENABLE

/* Timer settings */
#define SN_TIMER		    TIM13
#define SN_TIMER_IRQn	  TIM8_UP_TIM13_IRQn
#define SN_TIMER_EN_Clk	__HAL_RCC_TIM13_CLK_ENABLE
#define SN_TIMER_FREQ   108000000UL


#define SN_US_TO_METERS(US)  (float)((US) * 340.0)/(2.0*1000000.0)

typedef enum {
  SONAR_INDEX_1  = 0U,
  SONAR_INDEX_2  = 1U,
  SONAR_INDEX_3  = 2U,
  SONAR_INDEX_4  = 3U,
  SONAR_INDEX_5  = 4U,
  SONAR_INDEX_6  = 5U,
  SONAR_INDEX_7  = 6U,
  SONAR_INDEX_8  = 7U,
  SONAR_INDEX_9  = 8U,
  SONAR_INDEX_10 = 9U,
  SONAR_INDEX_11 = 10U,
  SONAR_INDEX_12 = 11U,
  SONAR_INDEX_13 = 12U,
  SONAR_INDEX_14 = 13U,
  SONAR_INDEX_15 = 14U,
  SONAR_INDEX_16 = 15U,
}  SONAR_index_t;

#define IS_CORRECT_SONAR_INDEX(index) (((index) >= SONAR_INDEX_1) && ((index) <= SONAR_INDEX_16))


void NOMAD_sonar_init();
float NOMAD_get_sonar_distance(SONAR_index_t index);
uint16_t NOMAD_get_sonar_time();
void NOMAD_echo_callback();
void NOMAD_timer_callback();

#endif /* INC_NOMAD_  SONAR_H_ */
