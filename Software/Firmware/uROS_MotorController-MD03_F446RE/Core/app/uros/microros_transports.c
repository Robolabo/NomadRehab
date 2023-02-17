

#include <unistd.h>
#include <stdio.h>
#include <string.h>
#include <stdbool.h>

#include "stm32f4xx_hal.h"
#include "uxr/client/transport.h"
#include "usart.h"

#include "FreeRTOS.h"
#include "queue.h"

#define RX_QUEUE_SIZE 1024U
#define TX_QUEUE_SIZE 1024U

static uint8_t in_byte = 0;
static uint8_t out_byte = 0;

static QueueHandle_t tx_queue;
static QueueHandle_t rx_queue;


bool transport_serial_open(struct uxrCustomTransport * transport) {

	rx_queue = xQueueCreate(RX_QUEUE_SIZE, sizeof(uint8_t));
	tx_queue = xQueueCreate(TX_QUEUE_SIZE, sizeof(uint8_t));

	HAL_UART_Receive_IT(&huart2, &in_byte, 1);
    return true;
}

bool transport_serial_close(struct uxrCustomTransport * transport) {

    return true;
}

size_t transport_serial_write(struct uxrCustomTransport* transport, const uint8_t * buf, size_t len, uint8_t * err) {

	size_t index = 0;

	if (huart2.gState == HAL_UART_STATE_READY) {
		HAL_UART_Transmit_IT(&huart2, &buf[index], 1U);
		index++;
	}

	while ((index < len) && (xQueueSend(tx_queue, &buf[index], 1U) == pdTRUE)) {
		index++;
	}

	return index;
}

size_t transport_serial_read(struct uxrCustomTransport* transport, uint8_t* buf, size_t len, int timeout, uint8_t* err) {
	size_t index = 0;

	while ( (index < len) && (xQueueReceive(rx_queue, &buf[index], timeout) == pdTRUE)) {
		index++;
	}
	return index;

}

/*************** Interrupt handlers ***************/


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

  if (xQueueReceiveFromISR(tx_queue, &out_byte, NULL) == pdTRUE) {
	  HAL_UART_Transmit_IT(huart, &out_byte, 1U);
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

  xQueueSendFromISR(rx_queue, &in_byte, NULL);
  HAL_UART_Receive_IT(huart, &in_byte, 1U);

}
