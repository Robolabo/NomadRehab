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
 * @file serial_transport.h
 * @author Alejo
 * @brief 
 *
 * @version 0.1
 * @date Oct 5, 2023
 *
 * @copyright Copyright (c) 2023
 *
 */

#ifndef APP_INCLUDE_SERIAL_TRANSPORT_H_
#define APP_INCLUDE_SERIAL_TRANSPORT_H_

/************************************************************************
    INCLUDES
************************************************************************/

#include <unistd.h>
#include <stdio.h>
#include <string.h>
#include <stdbool.h>

#include "stm32f4xx_hal.h"

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

/************************************************************************
    DEFINES AND TYPES
************************************************************************/

#define SERIAL_QUEUE_TX_SIZE 2048U
#define SERIAL_QUEUE_RX_SIZE 2048U

/************************************************************************
    FUNCTIONS
************************************************************************/

/**
 * @brief Initialize serial API.
 *
 * @param uart_handle   UART handle. Must peripheral must be initialized.
 *                      before invoking this function.
 * @return boolean indicating if the API has been initialized correctly.
 */
bool serial_open(UART_HandleTypeDef* uart_handle);

/**
 * @brief
 *
 * @param buffer
 * @param bufferSize
 * @param timeout
 * @return
 */
size_t serial_write(uint8_t* buffer, size_t bufferSize, TickType_t timeout);

/**
 * @brief
 *
 * @param buffer
 * @param bufferSize
 * @param timeout
 * @return
 */
size_t serial_readLine(uint8_t* buffer, size_t bufferSize, TickType_t timeout);

/**
 * @brief
 *
 * @param buffer
 * @param bufferSize
 * @param timeout
 * @return
 */
size_t serial_readLine(uint8_t* buffer, size_t bufferSize, TickType_t timeout);

#endif /* APP_INCLUDE_SERIAL_TRANSPORT_H_ */
