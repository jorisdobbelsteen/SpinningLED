//
// Created by Joris on 07/08/2023.
//

#include "motor_control.h"
#include <stdint.h>
#include "main.h"

extern TIM_HandleTypeDef MotorTimer;

static volatile motor_control_mode_t motor_state;

int motor_control_init(void) {
  // Disable timer to ensure it is really reset. Just in case...
  HAL_TIM_PWM_Stop(&MotorTimer, MOTOR_CHANNEL);

  motor_state = MOTOR_CONTROL_STOP;

  return 0;
}

int motor_control_start(uint16_t pulse) {
  if (motor_state == MOTOR_CONTROL_BREAK)
    return 1;

  // Set pulse length on initialization
  motor_control_set_pulse(pulse);

  // Clear some interrupts and clear flags
  __HAL_TIM_DISABLE_IT(&MotorTimer, TIM_IT_UPDATE);

  // Early exit if it was running already, but disable update interrupt
  if (motor_state == MOTOR_CONTROL_RUN)
    return 0;

  __HAL_TIM_CLEAR_FLAG(&MotorTimer, TIM_FLAG_UPDATE);
  __HAL_TIM_CLEAR_FLAG(&MotorTimer, TIM_FLAG_BREAK);
  // This call also enables the master enable
  if (HAL_TIM_PWM_Start(&MotorTimer, MOTOR_CHANNEL) != HAL_OK) {
    // Force off, probably not needed...
    HAL_TIM_PWM_Stop(&MotorTimer, MOTOR_CHANNEL);
    return 1;
  }
  motor_state = MOTOR_CONTROL_RUN;

  // Enable break after setting state, since this will set the motor_state variable
  __HAL_TIM_ENABLE_IT(&MotorTimer, TIM_IT_BREAK);

  // There will be some delay for the break interrupt is triggered. Hence don't check
  // if variable changed already.
  return 0;
}

int motor_control_stop(void) {
  HAL_TIM_PWM_Stop(&MotorTimer, MOTOR_CHANNEL);
  __HAL_TIM_DISABLE_IT(&MotorTimer, TIM_IT_UPDATE);
  __HAL_TIM_DISABLE_IT(&MotorTimer, TIM_IT_BREAK);
  __HAL_TIM_CLEAR_FLAG(&MotorTimer, TIM_FLAG_UPDATE);
  __HAL_TIM_CLEAR_FLAG(&MotorTimer, TIM_FLAG_BREAK);
  motor_state = MOTOR_CONTROL_STOP;

  return 0;
}

int motor_control_start_update_interrupt(void) {
  if (motor_state == MOTOR_CONTROL_RUN) {
    __HAL_TIM_CLEAR_FLAG(&MotorTimer, TIM_FLAG_UPDATE);
    __HAL_TIM_ENABLE_IT(&MotorTimer, TIM_IT_UPDATE);
    return 0;
  } else {
    return 1;
  }
}

int motor_control_stop_update_interrupt(void) {
  if (motor_state == MOTOR_CONTROL_RUN) {
    __HAL_TIM_DISABLE_IT(&MotorTimer, TIM_IT_UPDATE);
    return 0;
  } else {
    return 1;
  }
}

motor_control_mode_t motor_control_get_state(void) {
  return motor_state;
}


void motor_control_set_pulse(uint16_t pulse) {
  if (pulse < MOTOR_PWM_MIN)
    pulse = MOTOR_PWM_MIN;
  else if (pulse > MOTOR_PWM_MAX)
    pulse = MOTOR_PWM_MAX;

  __HAL_TIM_SET_COMPARE(&MotorTimer, MOTOR_CHANNEL, pulse);
}

void motor_control_force_break(void) {
  // Force motor timer off
  __HAL_TIM_MOE_DISABLE_UNCONDITIONALLY(&MotorTimer);
}

void IT_motor_control_break(void) {
  // PWM off has already happened here.
  // Just set the state to reflect this change...
  __HAL_TIM_DISABLE_IT(&MotorTimer, TIM_IT_BREAK);
  __HAL_TIM_DISABLE_IT(&MotorTimer, TIM_IT_UPDATE);
  motor_state = MOTOR_CONTROL_BREAK;
}
