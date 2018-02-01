#ifndef LINES_H_
#define LINES_H_

#include "hal.h"
#include "ch.h"
#include "chprintf.h"
#include "leds.h"
#include "motor.h"
#include "jetson.h"

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

#define CALIBRATION 5
#define LOAD_JETSON_CALIBRATION 3
#define CALIBRATION_VALUES 23

#define JETSON_SAVE_CALIBRATION 5
#define JETSON_LOAD_CALIBRATION 6
#define LINE_SAVE_CALIBRATION 7
#define LINE_LOAD_CALIBRATION 8

extern uint8_t line_calibration_values_in[NUMBER_OF_SENSORS];
extern uint8_t line_calibration_values_out[NUMBER_OF_SENSORS];

int32_t abs_int(int32_t x);
int32_t max(int32_t x, int32_t y);

void calculation_of_motor_speeds(void);


void determine_avoiding_direction(void) ;

void calibrate_lines(void);

uint8_t line_calibration(void);

void send_line_calibration(void);

msg_t check_line(void);

msg_t calibration_memory(msg_t command);

msg_t check_line_mailbox(void);

void send_to_line_mailbox(msg_t sending_command);


void line_init(void);

#endif
