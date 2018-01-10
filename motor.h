#ifndef MOTOR_H_
#define MOTOR_H_

#include "hal.h"
#include "ch.h"
#include "leds.h"


#define BITS_IN_CHAR 4
#define CHARS_IN_VALUE 4
#define COMMAND_CHAR_LENGTH 4
#define SPEED_VALUE_CHAR_LENGTH 3
#define NUMBER_OF_DRIBLERS 1
#define NUMBER_OF_MOTORS 3

#define MOVE_COMMAND 32123
#define TURN_ON_MOTOR_LED 16384
#define MOTOR_BLIKING_TIME 100

#define SAVE_MOTOR_SPEEDS 1
#define LOAD_MOTOR_SPEEDS 2

#define MOTOR_SERIAL &SD2

extern int16_t motors_speeds[NUMBER_OF_MOTORS];
extern int16_t driblers_speeds[NUMBER_OF_DRIBLERS];

void change_motors_speeds(void);

void copy_speeds(void);

void send_init_message(void);

void send_move_message(void);

void motor_blick(int8_t motor_id);

uint16_t motor_blick_interval(uint16_t loop);

void motors_init(void);

#endif
