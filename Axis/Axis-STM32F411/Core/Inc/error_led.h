//
// Created by Joris on 07/08/2023.
//

#ifndef AXIS_STM32F411_ERROR_LED_H
#define AXIS_STM32F411_ERROR_LED_H

typedef enum axis_error_t {
  AXIS_NO_ERROR = 0,
  AXIS_ERROR_FATAL,
  AXIS_ERROR_UNDEFINED,
  AXIS_ERROR_NO_ROTATION,
  AXIS_ERROR_MAX_VALUE
} axis_error_t;

void error_led_init(void);
void error_led_set(axis_error_t error);

#endif //AXIS_STM32F411_ERROR_LED_H
