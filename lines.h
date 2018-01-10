#ifndef LINES_H_
#define LINES_H_

#include "hal.h"
#include "ch.h"
#include "chprintf.h"
#include "leds.h"
#include "motor.h"

#define NUMBER_OF_SENSORS 16
#define NO_LINE_DETECTED 1
#define LINE_DETECTED 2
#define LOAD_LINES_SPEEDS 3
#define LINE_CALIBRATION -1

#define LINE_REACTION_TIME 2000

#define LINE_CALIBRATION_SPEED 20
#define LINE_CALIBRATION_TIME 2000

#define INERTIA 0.95
#define PI 3.14159
#define SQRT3 1.7321

#define LINE_QUEUE 3



int32_t abs_int(int32_t x);
int32_t max(int32_t x, int32_t y);

void calculation_of_motor_speeds(void);


void determine_avoiding_direction(void) ;
void calibrate_lines(void);
uint8_t line_calibration(void);
void send_line_calibration(void);
msg_t check_line(void);


void line_init(void);

#endif
