//
// Created by Joris on 10/08/2023.
//

#ifndef AXIS_STM32F411_LEDS_DRIVER_H
#define AXIS_STM32F411_LEDS_DRIVER_H

#include <stdint.h>
#include "main.h"

#define LEDS_DRIVER_EOF_PIXELS 1

typedef uint32_t pixel_t;

typedef struct led_pixel_column_t {
  pixel_t sof;
  pixel_t pixel[LEDS_Y];
  pixel_t eof[LEDS_DRIVER_EOF_PIXELS];
} led_pixel_column_t;

typedef union led_data_buffer_t {
  struct {
    struct led_pixel_column_t d0;
#ifdef FOUR_CHANNEL
    struct led_pixel_column_t d90i;
    struct led_pixel_column_t d180;
    struct led_pixel_column_t d270i;
  };
  struct led_pixel_column_t channel[4];
#else
    struct led_pixel_column_t d180i;
  };
  struct led_pixel_column_t channel[2];
#endif
} led_data_buffer_t;

void leds_driver_init(void);
void leds_driver_transmit(void);
void leds_driver_transmit_blank(void);

uint8_t leds_driver_get_brightness(void);
void leds_driver_set_brightness(uint8_t brightness);

led_data_buffer_t* leds_driver_get_next_buffer(void);
void leds_driver_update_hue(uint16_t hue);

#endif //AXIS_STM32F411_LEDS_DRIVER_H
