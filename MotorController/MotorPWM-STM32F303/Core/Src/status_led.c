//
// Created by Joris on 07/08/2023.
//

#include "status_led.h"
#include <stdint.h>
#include "main.h"

extern TIM_HandleTypeDef StatusLedTimer;

// Red Yellow Green (256 is 1 second)
// Guidance:
// Red = Error
// Yellow = Stop
// Green = Running
const uint8_t mode_blink_const[STATUS_LED_MODE_MAX_VALUE][3] = {
        /* INIT              */ {96,96,96},
        /* FATAL             */ {192,0,0},
        /* ERROR             */ {255,0,0},
        /* STOPPED           */ {0,255,0},
        /* STOPPING          */ {0,128,0},
        /* STARTING          */ {0,0,128},
        /* STARTED           */ {0,0,255},
        /* THROTTLE_CALIB    */ {128,128,255},
};

void status_led_init(void) {
  _Static_assert(sizeof(mode_blink_const) / sizeof(*mode_blink_const) == STATUS_LED_MODE_MAX_VALUE, "mode_blink array not same size as enum");

  status_led_set(STATUS_LED_MODE_INIT);
  HAL_TIM_PWM_Start(&StatusLedTimer, STATUS_LED_RED_CHANNEL);
  HAL_TIM_PWM_Start(&StatusLedTimer, STATUS_LED_YELLOW_CHANNEL);
  HAL_TIM_PWM_Start(&StatusLedTimer, STATUS_LED_GREEN_CHANNEL);
}

void status_led_set(status_led_mode_t mode) {
  if (mode >= STATUS_LED_MODE_MAX_VALUE) {
    mode = STATUS_LED_MODE_ERROR;
  }
  __HAL_TIM_SET_COMPARE(&StatusLedTimer, STATUS_LED_RED_CHANNEL, mode_blink_const[mode][0]);
  __HAL_TIM_SET_COMPARE(&StatusLedTimer, STATUS_LED_YELLOW_CHANNEL, mode_blink_const[mode][1]);
  __HAL_TIM_SET_COMPARE(&StatusLedTimer, STATUS_LED_GREEN_CHANNEL, mode_blink_const[mode][2]);
}
