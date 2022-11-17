/*
 * DM03.c
 *
 *  Created on: Nov 17, 2022
 *      Author: agome
 */
#include "DM03.h"

uint8_t MD03_isInitializaed = 0U;


MD03_error_t MD03_Init_port() {
  return MD03_ERROR_OK;
}

MD03_error_t MD03_RegisterRead_port(uint8_t dev_address, uint8_t reg_address, uint8_t* data) {
  MD03_error_t status = MD03_ERROR_ERROR;

  if (HAL_I2C_Master_Transmit(&hi2c1, dev_address, &reg_address, 1, 5U) == HAL_OK) {
    if (HAL_I2C_Master_Receive(&hi2c1, dev_address, data, 1U, 5U) == HAL_OK) {
      status = MD03_ERROR_OK;
    }
  }

  return status;
}

MD03_error_t MD03_RegisterWrite_port(uint8_t dev_address, uint8_t reg_address, uint8_t data) {
  MD03_error_t status = MD03_ERROR_OK;
  uint8_t buffer[2];
  buffer[0] = reg_address;
  buffer[1] = data;

  if (HAL_I2C_Master_Transmit(&hi2c1, dev_address, buffer, 2, 10U) != HAL_OK) {
    status = MD03_ERROR_ERROR;
  }
/*
  if (HAL_I2C_Mem_Write(&hi2c1, dev_address, reg_address, 1, &data, 1, 10U) != HAL_OK ) {
    status = MD03_ERROR_ERROR;
  }*/
  return status;
}


MD03_error_t MD03_init() {
  MD03_error_t status = MD03_ERROR_ERROR;
  if (!MD03_isInitializaed) {
    if (MD03_Init_port() == MD03_ERROR_OK) {
      MD03_isInitializaed = 1U;
      status = MD03_ERROR_OK;
    }
  }
  return status;
}

MD03_error_t MD03_get_register(uint8_t dev_address, uint8_t reg_address, uint8_t* data) {
  MD03_error_t status = MD03_ERROR_ERROR;
  if (MD03_isInitializaed) {
    if (MD03_RegisterRead_port(dev_address, reg_address, data) == MD03_ERROR_OK) {
      status = MD03_ERROR_OK;
    }
  }
  return status;
}


MD03_error_t MD03_set_register(uint8_t dev_address, uint8_t reg_address, uint8_t data) {
  MD03_error_t status = MD03_ERROR_ERROR;
  if (MD03_isInitializaed) {
    if (MD03_RegisterWrite_port(dev_address, reg_address, data) == MD03_ERROR_OK) {
      status = MD03_ERROR_OK;
    }
  }
  return status;
}

MD03_error_t MD03_test(uint8_t address) {
  uint8_t buffer;
  MD03_error_t status = MD03_ERROR_ERROR;
  if (MD03_get_register(address, 0x00, &buffer) != MD03_ERROR_OK) {
      status = MD03_ERROR_OK;
  }
  return status;
}


uint8_t MD03_get_version(uint8_t address) {
  uint8_t buffer;
  if (MD03_get_register(address, MD03_ADDR_VERSION, &buffer) != MD03_ERROR_OK) {
    buffer = 0xFFU;
  }
  return buffer;
}

uint8_t MD03_get_status(uint8_t address) {
  uint8_t buffer;
  if (MD03_get_register(address, MD03_ADDR_STATUS, &buffer) != MD03_ERROR_OK) {
    buffer = 0xFFU;
  }
  return buffer;
}

uint8_t MD03_get_temp_raw(uint8_t address) {
  uint8_t buffer;

  if (MD03_get_register(address, MD03_ADDR_TEMPERATURE, &buffer) != MD03_ERROR_OK) {
    buffer = 0x00U;
  }
  return buffer;
}

uint8_t MD03_get_curremt_raw(uint8_t address) {
  uint8_t buffer;

  if (MD03_get_register(address, MD03_ADDR_CURRENT, &buffer) != MD03_ERROR_OK) {
    buffer = 0x00U;
  }
  return buffer;
}


uint8_t MD03_get_speed_raw(uint8_t address) {
  uint8_t buffer;
  if (MD03_get_register(address, MD03_ADDR_SPEED, &buffer) != MD03_ERROR_OK) {
    buffer = 0xFFU;
  }
  return buffer;
}

MD03_error_t MD03_set_speed_raw(uint8_t address, uint8_t speed) {
  return MD03_set_register(address, MD03_ADDR_SPEED, speed);
}

uint8_t MD03_get_acceleration_raw(uint8_t address) {
  uint8_t buffer;
  if (MD03_get_register(address, MD03_ADDR_ACCELERATION, &buffer) != MD03_ERROR_OK) {
    buffer = 0xFFU;
  }
  return buffer;
}

MD03_error_t MD03_set_acceleration_raw(uint8_t address, uint8_t acceleration) {
  return MD03_set_register(address, MD03_ADDR_ACCELERATION, acceleration);
}


MD03_error_t MD03_set_direction(uint8_t address, MD03_direction_t direction) {
  /* Sanity check */
  if (MD03_isValidDirection(direction)) {
    return MD03_set_register(address, MD03_ADDR_COMMAND, direction);
  }
  else {
    return MD03_ERROR_PARAM_ERROR;
  }
}

MD03_direction_t MD03_get_direction(uint8_t address) {
  uint8_t buffer;

  if (MD03_get_register(address, MD03_ADDR_COMMAND, &buffer) != MD03_ERROR_OK) {
    buffer = 0x00U;
  }
  return (MD03_direction_t)buffer;
}
