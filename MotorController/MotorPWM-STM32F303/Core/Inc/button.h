//
// Created by Joris on 07/08/2023.
//

#ifndef BUTTON_H
#define BUTTON_H

void buttons_init();

int buttons_both_pressed(void);
void buttons_check(void);
void buttons_reset(void);

// Interrupts
void IT_buttons_check(void);

#endif //BUTTON_H
