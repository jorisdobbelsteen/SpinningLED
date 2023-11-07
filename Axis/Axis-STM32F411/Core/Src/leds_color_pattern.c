//
// Created by Joris on 01/11/2023.
//

#include "leds_color_pattern.h"
#include "main.h"

static inline void update_pattern_column(uint16_t x, led_pixel_column_t* col, pixel_t(*function)(uint8_t), const pixel_t pixel_base) {
  uint32_t* const pixel = col->pixel;

  for (size_t y = 0; y < LEDS_Y; ++y) {
    pixel[y] = function(x / 256) | pixel_base;
    x += 72; // staggered pattern
  }
}

static void update_pattern(int x, led_data_buffer_t* buffer, pixel_t(*function)(uint8_t)) {
  uint16_t hue = x * 0xffff / LEDS_X;

  const pixel_t pixel_base = leds_driver_get_pixel_base();

  update_pattern_column(hue, &buffer->d0, function, pixel_base);
#if FOUR_CHANNEL
  update_pattern_column(hue + 0x4000, &buffer->d90i, function, pixel_base);
  update_pattern_column(hue + 0x8000, &buffer->d180, function, pixel_base);
  update_pattern_column(hue + 0xc000, &buffer->d270i, function, pixel_base);
#else
  update_pattern_column(hue + 0x8000, &buffer->d180i, function, pixel_base);
#endif
}

static pixel_t HueToRgb(uint8_t h) {
  const unsigned int ha = (unsigned int)h * 6;
  const unsigned int x = ha & 0xFF;
  const unsigned int nx = ~ha & 0xFF;
  pixel_detail_t p = {0};
  switch(ha >> 8) {
    case 0: p.r = 0xFF; p.g = x; return p.value;
    case 1: p.g = 0xFF; p.r = nx; return p.value;
    case 2: p.g = 0xFF; p.b = x; return p.value;
    case 3: p.b = 0xFF; p.g = nx; return p.value;
    case 4: p.b = 0xFF; p.r = x; return p.value;
    case 5: p.r = 0xFF; p.b = nx; return p.value;
    default: return p.value;
  }
}

void update_leds_hue(int x, led_data_buffer_t* buffer) {
  update_pattern(x, buffer, &HueToRgb);
}


static pixel_t PickRgb(uint8_t h) {
  const unsigned int ha = (unsigned int)h * 3;
  pixel_detail_t p = {0};
  switch(ha >> 8) {
    case 0: p.r = 0xFF; return p.value;
    case 1: p.g = 0xFF; return p.value;
    case 2: p.b = 0xFF; return p.value;
    default: return p.value;
  }
}

void update_leds_rgb(int x, led_data_buffer_t* buffer) {
  update_pattern(x, buffer, &PickRgb);
}

void update_leds_interlace(int x, led_data_buffer_t* buffer) {
  (void)x;
  const pixel_t base = leds_driver_get_pixel_base();

  buffer->d0.pixel[0] = buffer->d0.pixel[LEDS_Y - 1] = pixel_rgbb(0xff, 0, 0, base);
#if FOUR_CHANNEL
  buffer->d90i.pixel[0] = buffer->d90i.pixel[LEDS_Y - 1] = pixel_rgbb(0, 0xff, 0, base);
  buffer->d180.pixel[0] = buffer->d180.pixel[LEDS_Y - 1] = pixel_rgbb(0xff, 0, 0, base);
  buffer->d270i.pixel[0] = buffer->d270i.pixel[LEDS_Y - 1] = pixel_rgbb(0, 0xff, 0, base);
#else
  buffer->d180i.pixel[0] = buffer->d180i.pixel[LEDS_Y - 1] = pixel_rgbb(0, 0xff, 0, base);
#endif

  for (size_t i = 1; i < LEDS_Y - 1; ++i) {
    buffer->d0.pixel[i] = pixel_rgbb(0x01, 0, 0, base);
#if FOUR_CHANNEL
    buffer->d90i.pixel[i] = pixel_rgbb(0, 0x01, 0, base);
    buffer->d180.pixel[i] = pixel_rgbb(0x01, 0, 0, base);
    buffer->d270i.pixel[i] = pixel_rgbb(0, 0x01, 0, base);
#else
    buffer->d180i.pixel[i] = pixel_rgbb(0, 0x01, 0, base);
#endif
  }
}
