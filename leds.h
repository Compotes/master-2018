#ifndef LEDS_H_
#define LEDS_H_

#include "hal.h"
#include "ch.h"

#define FALL_BLICK 1
#define STARTUP_BLICK 2
#define FIRST_BLICK 3
#define SECOND_BLICK 4
#define THIRD_BLICK 5
#define FOURTH_BLICK 6
#define FIRST_ON 7
#define SECOND_ON 8
#define THIRD_ON 9
#define FOURTH_ON 10
#define FIRST_OFF 11
#define SECOND_OFF 12
#define THIRD_OFF 13
#define FOURTH_OFF 14
#define UP_BLICK 15
#define STOP_BLICK 16

#define FIRST_LED_INDEX 10
#define NUMBER_OF_LEDS 3

#define LEDS_QUEUE 16

void leds_init(void);
void one_blink(uint8_t led);
void led_on(uint8_t led);
void led_off(uint8_t led);
void fall_blink(uint8_t times);
void up_blink(uint8_t times);
msg_t led_command(msg_t command);

#endif
