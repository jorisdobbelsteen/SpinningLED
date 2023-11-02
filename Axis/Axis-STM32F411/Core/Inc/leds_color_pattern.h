//
// Created by Joris on 01/11/2023.
//

#ifndef AXIS_STM32F411_LEDS_COLOR_PATTERN_H
#define AXIS_STM32F411_LEDS_COLOR_PATTERN_H

#include "leds_driver.h"

void update_leds_hue(int line, led_data_buffer_t* buffer);
void update_leds_rgb(int line, led_data_buffer_t* buffer);
void update_leds_interlace(int line, led_data_buffer_t* buffer);

#endif //AXIS_STM32F411_LEDS_COLOR_PATTERN_H
