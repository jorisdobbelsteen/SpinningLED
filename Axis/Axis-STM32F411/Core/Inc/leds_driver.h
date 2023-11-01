//
// Created by Joris on 10/08/2023.
//

#ifndef AXIS_STM32F411_LEDS_DRIVER_H
#define AXIS_STM32F411_LEDS_DRIVER_H

#include <stdint.h>

void leds_driver_init(void);
void leds_driver_transmit(void);
void leds_driver_transmit_blank(void);

void leds_driver_update_hue(uint16_t hue);
void leds_driver_set_brightness(uint8_t brightness);

#endif //AXIS_STM32F411_LEDS_DRIVER_H
