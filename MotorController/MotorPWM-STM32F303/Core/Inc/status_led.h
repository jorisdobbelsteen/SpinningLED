//
// Created by Joris on 07/08/2023.
//

#ifndef MOTORPWM_STATUS_LED_H
#define MOTORPWM_STATUS_LED_H

typedef enum status_led_mode_t {
  STATUS_LED_MODE_INIT = 0,
  STATUS_LED_MODE_FATAL,
  STATUS_LED_MODE_ERROR,
  STATUS_LED_MODE_STOPPED,
  STATUS_LED_MODE_STOPPING,
  STATUS_LED_MODE_STARTING,
  STATUS_LED_MODE_STARTED,
  STATUS_LED_MODE_THROTTLE_CALIB,
  STATUS_LED_MODE_MAX_VALUE
} status_led_mode_t;

void status_led_init(void);
void status_led_set(status_led_mode_t mode);

#endif //MOTORPWM_STATUS_LED_H
