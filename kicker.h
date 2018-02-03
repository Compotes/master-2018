#ifndef KICKER_H_
#define KICKER_H_

#include "ch.h"
#include "hal.h"
#include "leds.h"

#define KICK_PIN 2
#define KICK_GPIO GPIOD

#define READY_PIN 12
#define READY_GPIO GPIOC

#define KICK_DURATION 100 // ms

#define BALL_DETECT_PIN 15
#define BALL_DETECT_GPIO GPIOD

void kick(void);
int8_t kicker_is_ready(void);
int8_t i_have_ball(void);

#endif

