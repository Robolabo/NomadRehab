/*
 * encoder_controller.c
 *
 *  Created on: 12 feb. 2023
 *      Author: Alejo
 */

#include "encoder_controller.h"


/*************** Variables **************/
static Encoder_controller_t ENC_CONTROL_encoders[ENC_CONTROL_MAX_ENCODERS];     /*<! Encoder handle allocation */
static uint32_t ENC_CONTROL_encoderIndex = 0U;                                  /*<! Encoder allocation index */

/************** Prototypes **************/
static Encoder_controller_t* ENC_CONTROL_getEncoderInstance (TIM_HandleTypeDef* htim);
static void ENC_CONTROL_periodElapsedCallback (TIM_HandleTypeDef* htim);

/************** Source code **************/


/**
 * @brief Initialize an encoder structure.
 *        WARNING: The timer handle must be previously initialized
 *        and the interrupt enabled!!!!
 *
 * @param htim Associated timer handle.
 * @param countPerRevolution Encoder resolution
 * @param reductionFactor Motor speed reduction factor
 * @return
 */
Encoder_controller_t* ENC_CONTROL_init(
    TIM_HandleTypeDef* htim,
    uint32_t countPerRevolution,
    float reductionFactor)
{
  Encoder_controller_t* instance =  ENC_CONTROL_getEncoderInstance (htim);

  /* Check the parameter boundaries */

  if ((countPerRevolution == 0) || (reductionFactor <= 0)) {
    /* Incorrect parameters */
    return NULL;
  }

  /* Create new instance */
  if (instance == NULL){
    /* Sanity check! */
    if (ENC_CONTROL_encoderIndex < ENC_CONTROL_MAX_ENCODERS) {
      instance = &ENC_CONTROL_encoders[ENC_CONTROL_encoderIndex];
      ENC_CONTROL_encoderIndex++;
    }
    else {
      /* Something went wrong */
      while(1);
    }
  }


  /* Initialize encoder parameters */
  instance->htim = htim;
  instance->countPerRevolution = countPerRevolution;
  instance->reduction = reductionFactor;
  instance->revolutions = 0;


  /* Set the period to the total count of pulses in a complete revolution */
  /* This way the interrupt will be called every time a full revolution is performed */

  instance->htim->Init.Period = (uint32_t)(countPerRevolution*reductionFactor);
  instance->htim->Instance->ARR = instance->htim->Init.Period;
  instance->htim->PeriodElapsedCallback = ENC_CONTROL_periodElapsedCallback;

  __HAL_TIM_ENABLE_IT(htim, TIM_IT_UPDATE);
  HAL_TIM_Encoder_Start(htim, TIM_CHANNEL_ALL);

  return instance;
}


/**
 * @brief Reset the the relative position.
 *
 * @param htim timer handle.
 */
void ENC_CONTROL_reset (TIM_HandleTypeDef* htim) {
  Encoder_controller_t* instance = ENC_CONTROL_getEncoderInstance(htim);
  if (instance != NULL) {
    /* Reset the counter values */
    instance->revolutions = 0;
    instance->htim->Instance->CNT = 0U;
  }
}


/**
 * @brief Get the relative angular position
 *
 * @param htim timer handle.
 * @return Relative position in radians.
 */
float ENC_CONTROL_getPostion (TIM_HandleTypeDef* htim) {

  float position = 0.0;
  uint32_t counts = 0;
  Encoder_controller_t* instance = ENC_CONTROL_getEncoderInstance(htim);

  if (instance != NULL) {
    counts = instance->htim->Instance->CNT;

    position = (float)(instance->revolutions);
    position += ((float)counts/instance->htim->Init.Period);
    position =  2*M_PI*position;
  }

  return position;
}


/**
 * @brief Get encoder structure associated to a given handle.
 *
 * @param htim timer handle.
 * @return Encoder controller structure.
 */
static Encoder_controller_t* ENC_CONTROL_getEncoderInstance (TIM_HandleTypeDef* htim) {
  uint32_t index = 0U;
  Encoder_controller_t* result = NULL;

  for (index = 0; index < ENC_CONTROL_MAX_ENCODERS; index++) {
    if (ENC_CONTROL_encoders[index].htim == htim) {
      result = &ENC_CONTROL_encoders[index];
    }
  }
  return result;
}



/**
 * @brief Period elapsed callback. Triggered every revolution
 *
 * @param htim timer handler.
 */
static void ENC_CONTROL_periodElapsedCallback (TIM_HandleTypeDef* htim) {
  Encoder_controller_t* instance = ENC_CONTROL_getEncoderInstance(htim);

  if (instance != NULL) {
    if (__HAL_TIM_IS_TIM_COUNTING_DOWN(htim)) {
      instance->revolutions--;
    }
    else {
      instance->revolutions++;
    }
  }
}
