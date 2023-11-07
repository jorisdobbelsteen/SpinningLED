//
// Created by Joris on 06/11/2023.
//

#ifndef AXIS_STM32F411_LEDS_IMAGE_H
#define AXIS_STM32F411_LEDS_IMAGE_H

#include "leds_driver.h"

void update_leds_image_init(void);

void update_leds_image(int x, led_data_buffer_t* buffer);

#endif //AXIS_STM32F411_LEDS_IMAGE_H
