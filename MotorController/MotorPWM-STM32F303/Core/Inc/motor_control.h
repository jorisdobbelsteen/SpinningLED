//
// Created by Joris on 07/08/2023.
//

#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include <stdint.h>

#define MOTOR_PWM_MIN 1000   // microseconds pulse
#define MOTOR_PWM_MAX 2500   // microseconds pulse
#define MOTOR_PWM_LIMIT 1600 // microseconds pulse typical limit (going too fast if we hit this value)

typedef enum motor_control_mode_t {
  MOTOR_CONTROL_STOP,
  MOTOR_CONTROL_RUN,
  MOTOR_CONTROL_BREAK
} motor_control_mode_t;

int motor_control_init(void);

int motor_control_start(uint16_t pulse);
int motor_control_stop(void);
int motor_control_start_update_interrupt(void);
int motor_control_stop_update_interrupt(void);
void motor_control_set_pulse(uint16_t pulse);
motor_control_mode_t motor_control_get_state(void);

void motor_control_force_break(void);

// Interrupts
void IT_motor_control_break(void);

#endif //MOTOR_CONTROL_H
