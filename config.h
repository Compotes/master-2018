#ifndef CONFIG_H_
#define CONFIG_H_

#include "hal.h"
#include "ch.h"
#include "leds.h"

#define MAIN_QUEUE 4

msg_t check_main_mailbox(void);

void send_to_main_mailbox(msg_t sending_command);

void board_init(void);

#endif
