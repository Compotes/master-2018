#ifndef _COMPASS_H_
#define _COMPASS_H_

#include "ch.h"
#include "hal.h"
#include "chprintf.h"
#include "leds.h"

#define COMPASS_SERIAL &SD1

#define LOAD_COMPASS_DEGREE -385

int16_t get_compass_degree(void);

int16_t get_azimuth(void);

void compass_init(void);

void read(uint8_t address, uint8_t length);

void write(uint8_t address, uint8_t value);

void start(void);

void get_compass_values(void);

void get_calibration_status(void);

#endif
