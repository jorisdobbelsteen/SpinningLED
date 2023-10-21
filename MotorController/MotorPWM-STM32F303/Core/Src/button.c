//
// Created by Joris on 07/08/2023.
//

#include "button.h"
#include "main.h"

extern TIM_HandleTypeDef ButtonTimer;

static const uint16_t debounce_mask = 0xffe0u; // each bit is a timer tick (10 ms with SysTick), so 50 ms now
struct button_state_t {
  uint16_t state;
  uint16_t mask;
  volatile uint8_t pressed;
};

struct button_state_t button_start;
struct button_state_t button_stop;

void buttons_start_pressed(void);
void buttons_stop_pressed(void);

void buttons_init(void) {
  button_start.state = 0;
  button_start.mask = debounce_mask;
  button_start.pressed = 0;
  button_stop.state = 0;
  button_stop.mask = debounce_mask;
  button_stop.pressed = 0;

  //HAL_TIM_Base_Start_IT(&ButtonTimer);
  // Is currently using  SysTick instead...
}

int buttons_both_pressed(void) {
  return HAL_GPIO_ReadPin(START_BTN_GPIO_Port, START_BTN_Pin) == GPIO_PIN_RESET
         && HAL_GPIO_ReadPin(STOP_BTN_GPIO_Port, STOP_BTN_Pin) == GPIO_PIN_RESET;
}

void buttons_check(void) {
  if (button_start.pressed) {
    button_start.pressed = 0;
    buttons_start_pressed();
  }

  if (button_stop.pressed) {
    button_stop.pressed = 0;
    buttons_stop_pressed();
  }
}

void buttons_reset(void) {
  button_start.pressed = 0;
  button_stop.pressed = 0;
}

static void IT_buttons_check_single(struct button_state_t* button_state, GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin) {
  button_state->mask = button_state->mask << 1 | debounce_mask |
                      (HAL_GPIO_ReadPin(GPIOx, GPIO_Pin) == GPIO_PIN_RESET ? 0 : 1);
  if (button_state->state == 0 && button_state->mask == debounce_mask) {
    // pressed
    button_state->pressed = 1;
    button_state->state = 1;
  } else if (button_state->state == 1 && button_state->mask == 0xffff) {
    // depressed
    button_state->state = 0;
  }
}

void IT_buttons_check(void) {
  IT_buttons_check_single(&button_start, START_BTN_GPIO_Port, START_BTN_Pin);
  IT_buttons_check_single(&button_stop, STOP_BTN_GPIO_Port, STOP_BTN_Pin);
}