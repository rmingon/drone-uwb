/*
 * led.h
 *
 *  Created on: Sep 26, 2023
 *      Author: roro
 */

#ifndef MAIN_LED_H_
#define MAIN_LED_H_

#include "led_strip.h"

extern led_strip_handle_t led_strip;

typedef struct {
    int r;
    int g;
    int b;
} Color;

led_strip_handle_t ledInit(void);

void setLed(Color color);


#endif /* MAIN_LED_H_ */
