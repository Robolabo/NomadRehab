/*
 * nomad_sonar.c
 *
 *  Created on: Oct 27, 2022
 *      Author: agome
 */


#include "nomad_sonar.h"


static TIM_HandleTypeDef htim_sonar;
static uint8_t measureFinished = 0U;



static void NOMAD_period_elapsed_callback(TIM_HandleTypeDef *htim) {

  if ( (measureFinished != 0) || htim->Instance != SN_TIMER) {
    return;
  }
  HAL_TIM_Base_Stop(&htim_sonar);
  measureFinished = 1U;
}




static void NOMAD_gpio_init() {

  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* Initialize clocks */
  ECHO_EN_Clk();
  INIT_EN_Clk();
  BLNK_EN_Clk();
  BINH_EN_Clk();
  SN_A0_EN_Clk();
  SN_A1_EN_Clk();
  SN_A2_EN_Clk();
  SN_A3_EN_Clk();

  /* Set default values */
  HAL_GPIO_WritePin(ECHO_GPIO_Port, ECHO_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(INIT_GPIO_Port, INIT_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(BINH_GPIO_Port, BINH_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(BLNK_GPIO_Port, BLNK_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(SN_A0_GPIO_Port, SN_A0_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(SN_A1_GPIO_Port, SN_A1_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(SN_A2_GPIO_Port, SN_A2_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(SN_A3_GPIO_Port, SN_A3_Pin, GPIO_PIN_RESET);

  /* Configure output pins */
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;

  GPIO_InitStruct.Pin = SN_A0_Pin;
  HAL_GPIO_Init(SN_A0_GPIO_Port, &GPIO_InitStruct);
  GPIO_InitStruct.Pin = SN_A1_Pin;
  HAL_GPIO_Init(SN_A0_GPIO_Port, &GPIO_InitStruct);
  GPIO_InitStruct.Pin = SN_A2_Pin;
  HAL_GPIO_Init(SN_A0_GPIO_Port, &GPIO_InitStruct);
  GPIO_InitStruct.Pin = SN_A3_Pin;
  HAL_GPIO_Init(SN_A0_GPIO_Port, &GPIO_InitStruct);
  GPIO_InitStruct.Pin = INIT_Pin;
  HAL_GPIO_Init(INIT_GPIO_Port, &GPIO_InitStruct);
  GPIO_InitStruct.Pin = BINH_Pin;
  HAL_GPIO_Init(BINH_GPIO_Port, &GPIO_InitStruct);
  GPIO_InitStruct.Pin = BLNK_Pin;
  HAL_GPIO_Init(BLNK_GPIO_Port, &GPIO_InitStruct);

  /* Configure echo pin */
  GPIO_InitStruct.Pin = ECHO_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(ECHO_GPIO_Port, &GPIO_InitStruct);

}

static void NOMAD_timer_init() {
  uint32_t              uwPrescalerValue = 0U;
  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* Enable TIM2 clock */
  SN_TIMER_EN_Clk();

  /* Compute the prescaler value to have TIM2 counter clock equal to 1MHz */
  uwPrescalerValue = (uint32_t) (((SN_TIMER_FREQ) / 1000000U) - 1U);

  /* Initialize timer */
  htim_sonar.Instance = SN_TIMER;
  htim_sonar.Init.Period = 31470; // 10.7 m aprox
  htim_sonar.Init.Prescaler = uwPrescalerValue;
  htim_sonar.Init.ClockDivision = 0;
  htim_sonar.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim_sonar.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

  if (HAL_TIM_Base_Init(&htim_sonar) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim_sonar, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim_sonar, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }

  HAL_NVIC_SetPriority(SN_TIMER_IRQn, 5, 0);

  htim_sonar.PeriodElapsedCallback = NOMAD_period_elapsed_callback;
}

static void NOMAD_set_mux(SONAR_index_t index) {
  HAL_GPIO_WritePin(SN_A0_GPIO_Port, SN_A0_Pin, ((index >> 0) & 0x01));
  HAL_GPIO_WritePin(SN_A1_GPIO_Port, SN_A1_Pin, ((index >> 1) & 0x01));
  HAL_GPIO_WritePin(SN_A2_GPIO_Port, SN_A2_Pin, ((index >> 2) & 0x01));
  HAL_GPIO_WritePin(SN_A3_GPIO_Port, SN_A3_Pin, ((index >> 3) & 0x01));
}

void NOMAD_sonar_init() {
  NOMAD_gpio_init();
  NOMAD_timer_init();
}

float NOMAD_get_sonar_distance(SONAR_index_t index) {
  if (!IS_CORRECT_SONAR_INDEX(index)) {
    return 0.0;
  }
  NOMAD_set_mux(index);
  HAL_Delay(STABILIZATION_TIME_MS);

  uint16_t time_us = NOMAD_get_sonar_time();
  return (float)(time_us * 340.0)/(2*1000000.0);

}

uint16_t NOMAD_get_sonar_time() {
  uint16_t counter = 0xFFFF;
  /* Reset the counter */
  __HAL_TIM_SET_COUNTER(&htim_sonar, 0U);
  /* start the timer */
  measureFinished = 1U;
  NVIC_ClearPendingIRQ(ECHO_EXTI_IRQn);
  NVIC_ClearPendingIRQ(SN_TIMER_IRQn);
  NVIC_EnableIRQ(ECHO_EXTI_IRQn);
  NVIC_EnableIRQ(SN_TIMER_IRQn);

  HAL_TIM_Base_Start_IT(&htim_sonar);
  HAL_GPIO_WritePin(INIT_GPIO_Port, INIT_Pin, GPIO_PIN_SET);
  measureFinished = 0U;

  /* Set the INIT pin */
  while (!measureFinished) {
    /* Do nothing */
  }
  counter = __HAL_TIM_GET_COUNTER(&htim_sonar);

  HAL_GPIO_WritePin(INIT_GPIO_Port, INIT_Pin, GPIO_PIN_RESET);
  return counter;
}


void NOMAD_echo_callback() {

  if(__HAL_GPIO_EXTI_GET_IT(ECHO_Pin) != RESET)
  {
    __HAL_GPIO_EXTI_CLEAR_IT(ECHO_Pin);

    if (measureFinished != 0) {
      return;
    }
    HAL_TIM_Base_Stop(&htim_sonar);
    measureFinished = 1U;
  }
}

void NOMAD_timer_callback() {
  HAL_TIM_IRQHandler(&htim_sonar);
}
