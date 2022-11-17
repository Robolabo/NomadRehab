/*
 * DM03.h
 *
 *  Created on: Nov 17, 2022
 *      Author: agome
 */

#ifndef SRC_DM03_H_
#define SRC_DM03_H_

#include "i2c.h"
#include <stdint.h>

#define MD03_TEMPERATURE_RATIO  1.42


typedef enum {
  MD03_ERROR_OK = 0,
  MD03_ERROR_ERROR = -1,
  MD03_ERROR_PARAM_ERROR = -2
}MD03_error_t;

typedef enum {
  MD03_DIRECTION_STOP     = 0x00,
  MD03_DIRECTION_FORWARD  = 0x01,
  MD03_DIRECTION_BACKWARD = 0x02
}MD03_direction_t;

#define MD03_isValidDirection(dir)  (((dir) >= MD03_DIRECTION_STOP) && ((dir) <= MD03_DIRECTION_BACKWARD))

typedef enum {
  MD03_STATUS_ACCELERATION  = 0x01,
  MD03_STATUS_OVERCURRENT   = 0x02,
  MD03_STATUS_OVERTEMP      = 0x04,
  MD03_STATUS_BUSY          = 0x80
}MD03_Status_t;

typedef enum {
  MD03_ADDR_COMMAND       = 0x00,
  MD03_ADDR_STATUS        = 0x01,
  MD03_ADDR_SPEED         = 0x02,
  MD03_ADDR_ACCELERATION  = 0x03,
  MD03_ADDR_TEMPERATURE   = 0x04,
  MD03_ADDR_CURRENT       = 0x05,
  /* 0x06 unused */
  MD03_ADDR_VERSION       = 0x07,
}MD03_register_t;


#endif /* SRC_DM03_H_ */
