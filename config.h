#ifndef CONFIG_H_
#define CONFIG_H_

#include "hal.h"
#include "ch.h"
#include "leds.h"

#define MAIN_QUEUE 4
#define ULTRASONIC_QUEUE 4

#define START_ULTRASONIC 5
#define STOP_ULTRASONIC 6

msg_t check_ultrasonic_mailbox(void);

void send_to_ultrasonic_mailbox(msg_t sending_command);

msg_t check_main_mailbox(void);

void send_to_main_mailbox(msg_t sending_command);

void board_init(void);

#endif
