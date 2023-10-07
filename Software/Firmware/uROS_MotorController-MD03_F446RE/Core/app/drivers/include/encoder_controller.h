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
 * @file encoder_controller.h
 * @author Alejandro Gomez Molina (@Alejo2313)
 * @brief Rotatory encoder controller.
 *
 * @version 0.1
 * @date 12 feb. 2023
 *
 * @copyright Copyright (c) 2023
 *
 */

#ifndef INC_ENCODER_CONTROLLER_H_
#define INC_ENCODER_CONTROLLER_H_

/************************************************************************
    INCLUDES
************************************************************************/
#include <math.h>
#include "stm32f4xx_hal.h"

/************************************************************************
    DEFINES AND TYPES
************************************************************************/
#define ENC_CONTROL_MAX_ENCODERS  4U    /*<! Maximum number of encoders that the controller can handle */


/**
 * @brief Encoder handle structure.
 *
 */
typedef struct {
  TIM_HandleTypeDef* htim;        /*<! Associated timer handle*/
  uint32_t countPerRevolution;    /*<! Encoder resolution */
  float reduction;                /*<! Motor speed reduction factor */
  int32_t revolutions;           /*<! Total number of revolutions */
  uint32_t cnt;
  uint32_t scale;
} Encoder_controller_t;



/************************************************************************
    FUNCTIONS
************************************************************************/
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
Encoder_controller_t* ENC_CONTROL_init (
    TIM_HandleTypeDef* htim,
    uint32_t countPerRevolution,
    float reductionFactor);

/**
 * @brief Reset the the relative position.
 *
 * @param htim timer handle.
 */
void ENC_CONTROL_reset (TIM_HandleTypeDef* htim);

/**
 * @brief Get the relative angular position
 *
 * @param htim timer handle.
 * @return Relative position in radians.
 */
float ENC_CONTROL_getPostion (TIM_HandleTypeDef* htim);

/**
 * @brief
 *
 * @param instance
 * @param countPerRevolution
 * @param reductionFactor
 */
void ENC_CONTROL_setCPR (
    Encoder_controller_t* instance,
    uint32_t countPerRevolution,
    float reductionFactor);


#endif /* INC_ENCODER_CONTROLLER_H_ */
