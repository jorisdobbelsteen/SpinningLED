//
// Created by Joris on 07/08/2023.
//

#include "rotation_sensor.h"
#include "main.h"

extern TIM_HandleTypeDef RotationCounterTimer;
/*
 * Requirements for RotationCaptureTimer in this code:
 * 1. Counter frequency is 1 MHz
 *    The timer prescaler should be configured correctly to: (Clock_Frequency / 1 MHz) - 1.
 * 2. 32-bit counter is used (to handle overflow)
 */
extern TIM_HandleTypeDef RotationCaptureTimer;

#define CHANNEL_TO_FLAG(CHANNEL) ((CHANNEL == TIM_CHANNEL_1) ? TIM_FLAG_CC1 : \
                                  ((CHANNEL == TIM_CHANNEL_2) ? TIM_FLAG_CC2 : \
                                  ((CHANNEL == TIM_CHANNEL_3) ? TIM_FLAG_CC3 : \
                                  ((CHANNEL == TIM_CHANNEL_4) ? TIM_FLAG_CC4 : \
                                  ((CHANNEL == TIM_CHANNEL_5) ? TIM_FLAG_CC5 : \
                                  TIM_FLAG_CC6                                    )))))
#define CHANNEL_TO_OVERFLAG(CHANNEL) ((CHANNEL == TIM_CHANNEL_1) ? TIM_FLAG_CC1OF : \
                                     ((CHANNEL == TIM_CHANNEL_2) ? TIM_FLAG_CC2OF : \
                                     ((CHANNEL == TIM_CHANNEL_3) ? TIM_FLAG_CC3OF : \
                                     TIM_FLAG_CC4OF                                  )))

int rotation_sensor_init(void) {
  _Static_assert(ROTATION_CAPTURE_HALF_CHANNEL == TIM_CHANNEL_1
                 || ROTATION_CAPTURE_HALF_CHANNEL == TIM_CHANNEL_2
                 || ROTATION_CAPTURE_HALF_CHANNEL == TIM_CHANNEL_3
                 || ROTATION_CAPTURE_HALF_CHANNEL == TIM_CHANNEL_4
                 || ROTATION_CAPTURE_HALF_CHANNEL == TIM_CHANNEL_5
                 || ROTATION_CAPTURE_HALF_CHANNEL == TIM_CHANNEL_6);
  _Static_assert(ROTATION_CAPTURE_FULL_CHANNEL == TIM_CHANNEL_1
                 || ROTATION_CAPTURE_FULL_CHANNEL == TIM_CHANNEL_2
                 || ROTATION_CAPTURE_FULL_CHANNEL == TIM_CHANNEL_3
                 || ROTATION_CAPTURE_FULL_CHANNEL == TIM_CHANNEL_4);

  if (HAL_TIM_Base_Start(&RotationCounterTimer) != HAL_OK)
    goto error1;
  if (HAL_TIM_IC_Start(&RotationCaptureTimer, ROTATION_CAPTURE_FULL_CHANNEL) != HAL_OK)
    goto error2;
  if (HAL_TIM_IC_Start(&RotationCaptureTimer, ROTATION_CAPTURE_HALF_CHANNEL) != HAL_OK)
    goto error3;
  return 0;

error3:
  HAL_TIM_IC_Stop(&RotationCaptureTimer, ROTATION_CAPTURE_FULL_CHANNEL);
error2:
  HAL_TIM_Base_Stop(&RotationCounterTimer);
error1:
  return 1;
}

void rotation_sensor_reset(void) {
  __HAL_TIM_CLEAR_FLAG(&RotationCaptureTimer, CHANNEL_TO_FLAG(ROTATION_CAPTURE_FULL_CHANNEL));
  __HAL_TIM_CLEAR_FLAG(&RotationCaptureTimer, CHANNEL_TO_OVERFLAG(ROTATION_CAPTURE_FULL_CHANNEL));
  __HAL_TIM_CLEAR_FLAG(&RotationCaptureTimer, CHANNEL_TO_FLAG(ROTATION_CAPTURE_HALF_CHANNEL));
}

uint_rotation_t rotation_sensor_get_counter(void) {
  return (uint_rotation_t)__HAL_TIM_GET_COUNTER(&RotationCounterTimer);
}

uint32_t rotation_sensor_get_millipulses_per_sec(void) {
  // A lot of state is derived from the Timer unit itself without state in the code. This is done so
  // we do not have to rely on periodic interrupts and this can run with lower power consumption. Also,
  // there is no acquisition delay to get the latest state.
  //
  // The following units for the counter are important:
  // * 32-bit counter value (us), continuously incrementing. May wrap around to zero.
  // * Full pulse capture value (us)
  //   Upon capture -> * counter value is reset to 0
  //                   * Its capture Flag is set
  //                   * Its overcapture flag is set on Second capture (= capture flag is already set)
  // * Half pulse capture value (us):
  //   Upon capture -> * Its capture Flag is set.
  //                   * Its overcapture flag is set on Second capture (= capture flag is already set)
  //                     depending on which channel is used.
  //
  // Is it important to have overcapture to determine rotation speed?
  //   If a capture happens right after the counter overflows, the the value can be wildly incorrect.
  //   This happens every 1 hour 11 minutes for only seconds.
  //   While practically irrelevant, this is still good to take into account.
  //   Resolution is to use overcapture flag, and ensure capture + overcapture is reset when counter
  //     reaches a high enough value (e.g. after the midpoint).
  //
  // Can we actually use the half capture?
  //   The half capture assumes that the signal has a 50% duty cycle (or close enough). So assuming
  //   a half capture occurred, then:
  //   * if no full capture (hence counter reset has occurred) -> we do know that sec/pulse > half capture value
  //     Probably no harm, but bad estimate...
  //   * if no full capture occurred, then we did have a reset.
  //     However, did the reset occur before the half capture or after?
  //     * If there is an overcapture on full cycle, we are guaranteed to have had a reset
  //     * Otherwise, time is from half -> full or full (hence zero) to half. Except immediately after a
  //       restart, no pulse was provided for a long time and the shortest duration is likely relevant.
  //       If the counter can be started with a high starting value, we can determine this.
  // All in all, the half capture is very minimally relevant.
  // Hence, let's ignore it for now...
  //
  // Hence we can derive that:
  // * No overcapture -> no rotation / extremely slow rotation / not determined yet
  // * Overcapture present
  //   Current rotation time = max(full_capture, counter)

  uint32_t pulse_us;

  uint32_t cnt = __HAL_TIM_GET_COUNTER(&RotationCaptureTimer);
  int full_updated = __HAL_TIM_GET_FLAG(&RotationCaptureTimer, CHANNEL_TO_OVERFLAG(ROTATION_CAPTURE_FULL_CHANNEL));
  if (full_updated) {
    uint32_t full = __HAL_TIM_GET_COMPARE(&RotationCaptureTimer, ROTATION_CAPTURE_FULL_CHANNEL);
    // pick maximum value
    pulse_us = full > cnt ? full : cnt;
  } else {
    pulse_us = UINT32_MAX;
  }

  // We do assume we execute this function somewhat periodically
  if (cnt >= UINT32_MAX / 2) {
    __HAL_TIM_CLEAR_FLAG(&RotationCaptureTimer, CHANNEL_TO_FLAG(ROTATION_CAPTURE_FULL_CHANNEL));
    __HAL_TIM_CLEAR_FLAG(&RotationCaptureTimer, CHANNEL_TO_OVERFLAG(ROTATION_CAPTURE_FULL_CHANNEL));
  }

  //pulse_us => us / tick
  // 1 / pulse_us => tick / us
  // 1000000 / pulse_us => tick / sec
  // 1000000000 / pulse_us => 1/1000 * tick / sec
  return 1000000000 / pulse_us;
}
