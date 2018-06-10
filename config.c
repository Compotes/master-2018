#include "config.h"

static msg_t main_commands_queue[MAIN_QUEUE];
static mailbox_t main_commands;
static msg_t ultrasonic_commands_queue[ULTRASONIC_QUEUE];
static mailbox_t ultrasonic_commands;

msg_t check_main_mailbox(void){
	msg_t ret = 0;
	chMBFetch(&main_commands, &ret, TIME_IMMEDIATE);
	return ret;
}

void send_to_main_mailbox(msg_t sending_command){
	chMBPost(&main_commands, sending_command, TIME_INFINITE);
}

msg_t check_ultrasonic_mailbox(void){
	msg_t ret = 0;
	chMBFetch(&ultrasonic_commands, &ret, TIME_IMMEDIATE);
	return ret;
}

void send_to_ultrasonic_mailbox(msg_t sending_command){
	chMBPost(&ultrasonic_commands, sending_command, TIME_INFINITE);
}

void set_pins(void)
{
	// START pin
	palSetPadMode(GPIOD, 14, PAL_MODE_INPUT_PULLDOWN);

	// DIP_SWITCH pins
	palSetPadMode(GPIOD, 9, PAL_MODE_INPUT_PULLDOWN);
	palSetPadMode(GPIOD, 8, PAL_MODE_INPUT_PULLDOWN);
	palSetPadMode(GPIOB, 15, PAL_MODE_INPUT_PULLDOWN);
	palSetPadMode(GPIOB, 14, PAL_MODE_INPUT_PULLDOWN);
	palSetPadMode(GPIOB, 13, PAL_MODE_INPUT_PULLDOWN);
	palSetPadMode(GPIOB, 12, PAL_MODE_INPUT_PULLDOWN);

	// LED pins
	palSetPadMode(GPIOD, 10, PAL_MODE_OUTPUT_PUSHPULL);
	palSetPadMode(GPIOD, 11, PAL_MODE_OUTPUT_PUSHPULL);
	palSetPadMode(GPIOD, 12, PAL_MODE_OUTPUT_PUSHPULL);
	palSetPadMode(GPIOD, 13, PAL_MODE_ALTERNATE(2));

	palClearPad(GPIOD, 10);
	palClearPad(GPIOD, 11);
	palClearPad(GPIOD, 12);
	//palClearPad(GPIOD, 13);

	// UART pins

	palSetPadMode(GPIOD, 5, PAL_MODE_ALTERNATE(7)); // USART_2 Motors
	palSetPadMode(GPIOD, 6, PAL_MODE_ALTERNATE(7));

	palSetPadMode(GPIOB, 10, PAL_MODE_OUTPUT_PUSHPULL); // USART_3 Debug
	//palSetPadMode(GPIOB, 11, PAL_MODE_ALTERNATE(1));

	palSetPadMode(GPIOD, 5, PAL_MODE_ALTERNATE(7)); // UART_4 Bluetooth
	palSetPadMode(GPIOD, 6, PAL_MODE_ALTERNATE(7));

	palSetPadMode(GPIOC, 6, PAL_MODE_ALTERNATE(8)); // USART_6 Jetson
	palSetPadMode(GPIOC, 7, PAL_MODE_ALTERNATE(8));

	palSetPadMode(GPIOA, 9, PAL_MODE_ALTERNATE(1));  // USART_1 Compass
	palSetPadMode(GPIOA, 10, PAL_MODE_OUTPUT_PUSHPULL);

	// COMPASS mode pins
	palSetPadMode(GPIOC, 8, PAL_MODE_OUTPUT_PUSHPULL); // PS0
	palSetPadMode(GPIOC, 9, PAL_MODE_OUTPUT_PUSHPULL); // PS1


	// KICKER pins
	palSetPadMode(GPIOD, 2, PAL_MODE_OUTPUT_PUSHPULL); // KICK
	palSetPadMode(GPIOC, 12, PAL_MODE_INPUT_PULLDOWN); // READY

	// BALL_DETECTON pin
	palSetPadMode(GPIOD, 15, PAL_MODE_INPUT_PULLDOWN); //BALL_DETECTON

	// ADC pins
	palSetPadMode(GPIOA, 0, PAL_MODE_INPUT_ANALOG);
	palSetPadMode(GPIOA, 1, PAL_MODE_INPUT_ANALOG);
	palSetPadMode(GPIOA, 2, PAL_MODE_INPUT_ANALOG);
	palSetPadMode(GPIOA, 3, PAL_MODE_INPUT_ANALOG);
	palSetPadMode(GPIOA, 4, PAL_MODE_INPUT_ANALOG);
	palSetPadMode(GPIOA, 5, PAL_MODE_INPUT_ANALOG);
	palSetPadMode(GPIOA, 6, PAL_MODE_INPUT_ANALOG);
	palSetPadMode(GPIOA, 7, PAL_MODE_INPUT_ANALOG);
	palSetPadMode(GPIOB, 0, PAL_MODE_INPUT_ANALOG);
	palSetPadMode(GPIOB, 1, PAL_MODE_INPUT_ANALOG);
	palSetPadMode(GPIOC, 0, PAL_MODE_INPUT_ANALOG);
	palSetPadMode(GPIOC, 1, PAL_MODE_INPUT_ANALOG);
	palSetPadMode(GPIOC, 2, PAL_MODE_INPUT_ANALOG);
	palSetPadMode(GPIOC, 3, PAL_MODE_INPUT_ANALOG);
	palSetPadMode(GPIOC, 4, PAL_MODE_INPUT_ANALOG);
	palSetPadMode(GPIOC, 5, PAL_MODE_INPUT_ANALOG);
}

static const SerialConfig usart_1_cfg = {
	115200,
	0,
	0,
	0
};

static const SerialConfig usart_2_cfg = {
	115200,
	0,
	0,
	0
};

static const SerialConfig usart_3_cfg = {
	115200,
	0,
	0,
	0
};

static const SerialConfig usart_4_cfg = {
	57600,
	0,
	0,
	0
};

static const SerialConfig usart_6_cfg = {
	115200,
	0,
	0,
	0
};

void init_drivers(void) {
	sdInit();

	//sdStart(&SD1, &usart_1_cfg);
	sdStart(&SD2, &usart_2_cfg);
	//sdStart(&SD3, &usart_3_cfg);
	sdStart(&SD4, &usart_4_cfg);
	sdStart(&SD6, &usart_6_cfg);

	adcStart(&ADCD1, NULL);
	adcStart(&ADCD2, NULL);
	adcStart(&ADCD3, NULL);
}

void board_init(void) {
	set_pins();
	init_drivers();

	chMBObjectInit(&ultrasonic_commands, ultrasonic_commands_queue, ULTRASONIC_QUEUE);
	chMBObjectInit(&main_commands, main_commands_queue, MAIN_QUEUE);
}
