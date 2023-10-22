//
// Created by Joris on 07/08/2023.
//

#ifndef ROTATION_SENSOR_H
#define ROTATION_SENSOR_H

#include <stdint.h>

typedef uint16_t uint_rotation_t;

int rotation_sensor_init(void);
void rotation_sensor_reset(void);

uint_rotation_t rotation_sensor_get_counter(void); // Retrieve rotation absolute counter (immediate)
uint32_t rotation_sensor_get_millipulses_per_sec(void);     // Retrieve pulses per millisecond values

#endif //ROTATION_SENSOR_H
