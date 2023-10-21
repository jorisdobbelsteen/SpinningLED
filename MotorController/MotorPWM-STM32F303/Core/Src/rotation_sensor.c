//
// Created by Joris on 07/08/2023.
//

#include "rotation_sensor.h"
#include "main.h"

extern TIM_HandleTypeDef RotationCounterTimer;
extern TIM_HandleTypeDef RotationCaptureTimer;

#define TIM_CHANNEL_TO_FLAG(CHANNEL) ((CHANNEL == TIM_CHANNEL_1) ? TIM_FLAG_CC1 : \
                                      ((CHANNEL == TIM_CHANNEL_2) ? TIM_FLAG_CC2 : \
                                      ((CHANNEL == TIM_CHANNEL_3) ? TIM_FLAG_CC3 : \
                                      ((CHANNEL == TIM_CHANNEL_4) ? TIM_FLAG_CC4 : \
                                      ((CHANNEL == TIM_CHANNEL_5) ? TIM_FLAG_CC5 : \
                                      TIM_FLAG_CC6                                    )))))

int rotation_sensor_init(void) {
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

uint_rotation_t rotation_sensor_get_counter(void) {
  return (uint_rotation_t)__HAL_TIM_GET_COUNTER(&RotationCounterTimer);
}

uint16_t rotation_sensor_get_rpm(void) {
  uint32_t half = __HAL_TIM_GET_COMPARE(&RotationCaptureTimer, ROTATION_CAPTURE_HALF_CHANNEL);
  uint32_t full = __HAL_TIM_GET_COMPARE(&RotationCaptureTimer, ROTATION_CAPTURE_FULL_CHANNEL);

  //__HAL_TIM_GET_FLAG()
  return full;
}
