#ifndef ULTRASONIC_H_
#define ULTRASONIC_H_

#include "hal.h"
#include "ch.h"
#include "leds.h"
#include "jetson.h"
#include "config.h"

#define MIN_PULSE_LENGTH 1
#define MAX_PULSE_LENGTH 3000

void icuwidthcb(ICUDriver *icup);

void icuoverflowcb(ICUDriver *icup);

void ultrasonic_init(void);

#endif

