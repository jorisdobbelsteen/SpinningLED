//
// Created by Joris on 07/08/2023.
//

#ifndef AXIS_STM32F411_CONTROL_IF_H
#define AXIS_STM32F411_CONTROL_IF_H

void control_if_init(void);
void control_if_set_program(uint16_t program);
void control_if_process(void);

#endif //AXIS_STM32F411_CONTROL_IF_H
