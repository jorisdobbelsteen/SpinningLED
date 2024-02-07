//
// Created by Joris on 07/08/2023.
//

#include "error_led.h"
#include <stdint.h>
#include "main.h"

static axis_error_t error_led_current;
static uint16_t error_led_sequence[10];

extern TIM_HandleTypeDef htim3;

const uint8_t error_sequence_const[AXIS_ERROR_MAX_VALUE][8] = {
        /* NO_ERROR    */ {0,0,0,0,0,0,0,0},
        /* FATAL_ERROR */ {255,255,255,0,0,255,255,255},
        /* FATAL_FAULT */ {255,255,255,255,255,255,255,255},
        /* INIT        */ {192,192,192,192,192, 192,192,192},
        /* NO_ROTATION */ {64,0,64,0,64,0,64,0},
        /* LATE        */ {128,128,0,0,128,128,0,0},
};

void error_led_init(void) {
  for(size_t i = 0; i != sizeof(error_led_sequence)/sizeof(*error_led_sequence); ++i) {
    error_led_sequence[i] = 0;
  }
  error_led_set(AXIS_ERROR_INIT);
  HAL_TIM_PWM_Start_DMA(&hTIM_ERROR, TIM_ERROR_CHANNEL_LED, (uint32_t*)error_led_sequence, 10);
}

void error_led_set(axis_error_t error) {
  if (error == error_led_current) {
    return;
  }
  if (error >= AXIS_ERROR_MAX_VALUE) {
    error = AXIS_ERROR_FATAL_ERROR;
  }
  for(size_t i = 0; i != sizeof(error_sequence_const[0])/sizeof(error_sequence_const[0][0]); ++i) {
    uint16_t v = error_sequence_const[error][i];
    error_led_sequence[i] = v | (v << 8);
  }
  error_led_current = error;
}
