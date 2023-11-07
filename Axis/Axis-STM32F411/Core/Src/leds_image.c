//
// Created by Joris on 01/11/2023.
//

#include "leds_image.h"
#include "main.h"
#include "img_black_and_white_cat.h"

#include <string.h>

// Buffer is in the order of [column0, column1, column2, ...]
uint16_t image_buffer[LEDS_X * (2 * LEDS_Y)];

void update_leds_image_init(void) {
  _Static_assert(sizeof(img_black_and_white_cat) == sizeof(image_buffer));
  memcpy(image_buffer, img_black_and_white_cat, sizeof(image_buffer));
}

static inline pixel_t rgb565_to_pixel(uint16_t input, const pixel_t pixel_base) {
  pixel_detail_t p; p.value = pixel_base;
  p.r = (input & 0xf800) >> 8;
  p.g = (input & 0x07e0) >> 2;
  p.b = (input & 0x001f) << 3;
  return p.value;
}

static inline void update_image_column(uint16_t* pixels, led_pixel_column_t* col, const pixel_t pixel_base) {
  uint32_t* const output = col->pixel;

  for (size_t y = 0; y < LEDS_Y; ++y) {
    uint16_t in = pixels[2 * y];
    pixel_t res = rgb565_to_pixel(in, pixel_base);
    output[y] = res;
  }
}

static inline uint16_t* image_buffer_offset(int x, int y) {
  if (x >= LEDS_X)
    x = x - LEDS_X;
  return image_buffer + x * 2 * LEDS_Y + y;
}

void update_leds_image(int x, led_data_buffer_t* buffer) {
  const pixel_t pixel_base = leds_driver_get_pixel_base();

  update_image_column(image_buffer_offset(x, 0), &buffer->d0, pixel_base);
#if FOUR_CHANNEL
  update_image_column(image_buffer_offset(x + LEDS_X / 4, 1), &buffer->d90i, pixel_base);
  update_image_column(image_buffer_offset(x + LEDS_X / 2, 0), &buffer->d180, pixel_base);
  update_image_column(image_buffer_offset(x + LEDS_X * 3 / 4, 1), &buffer->d270i, pixel_base);
#else
  update_image_column(image_buffer_offset(x + LEDS_X / 2, 1), &buffer->d180i, pixel_base);
#endif
}
