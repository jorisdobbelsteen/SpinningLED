//
// Created by Joris on 07/08/2023.
//

#ifndef ROTATION_SENSOR_H
#define ROTATION_SENSOR_H

#include <stdint.h>

typedef uint16_t uint_rotation_t;

int rotation_sensor_init(void);

uint_rotation_t rotation_sensor_get_counter(void);
uint16_t rotation_sensor_get_rpm(void);


#endif //ROTATION_SENSOR_H
