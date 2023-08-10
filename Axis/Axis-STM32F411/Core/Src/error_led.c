//
// Created by Joris on 07/08/2023.
//

#include "error_led.h"
#include <stdint.h>
#include "main.h"

uint16_t error_led_sequence[10];

extern TIM_HandleTypeDef htim3;

const uint8_t error_sequence_const[AXIS_ERROR_MAX_VALUE][8] = {
        /* NO_ERROR    */ {0,0,0,0,0,0,0,0},
        /* FATAL       */ {255,255,255,0,0,255,255,255},
        /* UNDEFINED   */ {255,255,0,128,128,128,128,128},
        /* NO_ROTATION */ {128,0,128,0,128,0,128,0},
};

void error_led_init(void) {
  error_led_sequence[8] = 0;
  error_led_sequence[9] = 0;
  error_led_set(AXIS_ERROR_UNDEFINED);
  HAL_TIM_PWM_Start_DMA(&hTIM_ERROR, TIM_ERROR_CHANNEL_LED, (uint32_t*)error_led_sequence, 10);
}

void error_led_set(axis_error_t error) {
  if (error >= AXIS_ERROR_MAX_VALUE) {
    error = AXIS_ERROR_FATAL;
  }
  for(size_t i = 0; i < 8; ++i) {
    uint16_t v = error_sequence_const[error][i];
    error_led_sequence[i] = v | (v << 8);
  }
}
