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
 * @file nomad_pwm.h
 * @author Alejandro Gomez Molina (@Alejo2313)
 * @brief Nomad PWM controller.
 *
 * @version 0.1
 * @date Nov 21, 2022
 *
 * @copyright Copyright (c) 2023
 *
 */

#ifndef INC_NOMAD_NOMAD_PWM_H_
#define INC_NOMAD_NOMAD_PWM_H_

/************************************************************************
    INCLUDES
************************************************************************/

#include "tim.h"
#include "gpio.h"
#include "main.h"

/************************************************************************
    DEFINES AND TYPES
************************************************************************/

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


/************************************************************************
    FUNCTIONS
************************************************************************/


/**
 * @brief Initialize PWM timer.
 *        WARNING: The timer handler must be initialized.
 */
void NOMAD_PWM_init ();

/**
 * @brief Start PWM for a given channel.
 *
 * @param channel PWM channel to initialize.
 */
void NOMAD_PWM_start(NOMAD_PWM_Channel_t channel);

/**
 * @brief Set the dutyCycle for a given channel.
 *        the direction is implicit in the duty sign.
 *
 * @param channel PWM channel.
 * @param duty Duty clycle (-100.0 - 100.0)
 */
void NOMAD_PWM_setDuty(NOMAD_PWM_Channel_t channel, float duty);

/**
 * @brief Get the current duty cycle.
 *
 * @param channel
 * @return
 */
float NOMAD_PWM_get_duty_cycle(NOMAD_PWM_Channel_t channel);


#endif /* INC_NOMAD_NOMAD_PWM_H_ */
