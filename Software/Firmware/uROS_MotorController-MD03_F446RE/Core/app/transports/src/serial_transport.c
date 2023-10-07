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
 * @file serial_transport.c
 * @author Alejo
 * @brief 
 *
 * @version 0.1
 * @date Oct 5, 2023
 *
 * @copyright Copyright (c) 2023
 *
 */

/************************************************************************
    INCLUDES
************************************************************************/

#include "serial_transport.h"

/************************************************************************
    PRIVATE DEFINES AND TYPES
************************************************************************/

/************************************************************************
    PRIVATE DECLARATIONS
************************************************************************/

static uint8_t SERIAL_inBuffer;
static uint8_t SERIAL_outbuffer;

static QueueHandle_t SERIAL_queueTX;
static QueueHandle_t SERIAL_queueRX;

static UART_HandleTypeDef* SERIAL_handle;

/************************************************************************
    FUNCTIONS
************************************************************************/
/**
 * @brief
 *
 */
bool serial_open(UART_HandleTypeDef* uart_handle)
{

  if (uart_handle == NULL) {
    return false;
  }
  SERIAL_handle = uart_handle;

  SERIAL_queueRX = xQueueCreate(SERIAL_QUEUE_RX_SIZE, sizeof(uint8_t));
  SERIAL_queueTX = xQueueCreate(SERIAL_QUEUE_TX_SIZE, sizeof(uint8_t));

  if ((SERIAL_queueTX == NULL) || (SERIAL_queueRX == NULL)) {
    return false;
  }

  return (HAL_OK == HAL_UART_Receive_IT(SERIAL_handle, &SERIAL_inBuffer, 1U));
}

/**
 * @brief
 *
 */
size_t serial_write(uint8_t* buffer, size_t bufferSize, TickType_t timeout)
{
  size_t len = 0;
  TickType_t tickEnd = xTaskGetTickCount() + timeout;
  /* Trigger transmission */
  if (SERIAL_handle->gState == HAL_UART_STATE_READY) {
    HAL_UART_Transmit_IT(SERIAL_handle, &buffer[len++], 1U);
  }
  /* Add data to the output buffer */
  do {
    if (xQueueSend(SERIAL_queueTX, &buffer[len], 1U) == pdTRUE) {
      len++;
    }
  } while ((len < bufferSize) && (xTaskGetTickCount() < tickEnd));
  return len;
}

/**
 * @brief
 *
 */
size_t serial_readLine(uint8_t* buffer, size_t bufferSize, TickType_t timeout)
{
  size_t len = 0;
  TickType_t tickEnd = xTaskGetTickCount() + timeout;

  if (bufferSize == 0U) {
    return 0;
  }

  do {
    if (xQueueReceive(SERIAL_queueRX, &buffer[len], 1U) == pdTRUE) {
      len++;
      if (buffer[len - 1] == '\n') {
        break;
      }
    }
  } while ((len < bufferSize) && (xTaskGetTickCount() < tickEnd));
   return len;
}

/**
 * @brief
 *
 */
size_t serial_read(uint8_t* buffer, size_t bufferSize, TickType_t timeout)
{
  size_t len = 0;
  TickType_t tickEnd = xTaskGetTickCount() + timeout;

  if (bufferSize == 0U) {
    return 0;
  }

  do {
    if (xQueueReceive(SERIAL_queueRX, &buffer[len], 1U) == pdTRUE) {
      len++;
    }
  } while ((len < bufferSize) && (xTaskGetTickCount() < tickEnd));

  return len;
}



/************************************************************************
    Interrupt handlers
************************************************************************/


/**
  * @brief  Tx Transfer completed callbacks.
  * @param  huart  Pointer to a UART_HandleTypeDef structure that contains
  *                the configuration information for the specified UART module.
  * @retval None
  */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(huart);

  if (xQueueReceiveFromISR(SERIAL_queueTX, &SERIAL_outbuffer, NULL) == pdTRUE) {
    HAL_UART_Transmit_IT(huart, &SERIAL_outbuffer, 1U);
  }
}


/**
  * @brief  Tx Transfer completed callbacks.
  * @param  huart  Pointer to a UART_HandleTypeDef structure that contains
  *                the configuration information for the specified UART module.
  * @retval None
  */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(huart);

  xQueueSendFromISR(SERIAL_queueRX, &SERIAL_inBuffer, NULL);
  HAL_UART_Receive_IT(huart, &SERIAL_inBuffer, 1U);
}
