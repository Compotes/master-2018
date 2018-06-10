#include "leds.h"

thread_t *leds_thread;

static msg_t leds_commands_queue[LEDS_QUEUE];
static mailbox_t leds_commands;

void one_blink(uint8_t led){
	palSetPad(GPIOD, FIRST_LED_INDEX+led);
	chThdSleepMilliseconds(50);
	palClearPad(GPIOD, FIRST_LED_INDEX+led);
}

void led_on(uint8_t led){
	palSetPad(GPIOD, FIRST_LED_INDEX+led);
}

void led_off(uint8_t led){
	palClearPad(GPIOD, FIRST_LED_INDEX+led);
}

void fall_blink(uint8_t times){
	uint8_t i, j;

	for(j = 0; j < times; j++){
		for(i = 0; i < NUMBER_OF_LEDS; i++){
			one_blink(i);
		}
	}
}

void up_blink(uint8_t times){
	uint8_t i, j;

	for(j = 0; j < times; j++){
		for(i = NUMBER_OF_LEDS; i > 0; i--){
			one_blink(i-1);
		}
	}
}


msg_t led_command(msg_t command){
	return 	chMBPost(&leds_commands, command, TIME_INFINITE);
	//return chMsgSend(leds_thread, command);
}

THD_WORKING_AREA(waLedsThread, 128);
THD_FUNCTION(LedsThread, arg) {
	(void)arg;

	//thread_t *master;
	msg_t msg;
	fall_blink(2);

	while(1) {
		chMBFetch(&leds_commands, &msg, TIME_INFINITE);
		//master = chMsgWait();
		//msg = chMsgGet(master);
		//chMsgRelease(master, MSG_OK);
		if(msg == FALL_BLICK) fall_blink(1);
		else if(msg == STARTUP_BLICK) fall_blink(2);
		else if(msg == UP_BLICK) up_blink(1);
		else if(msg == STOP_BLICK) up_blink(2);
		else if(msg == FIRST_BLICK) one_blink(0);
		else if(msg == SECOND_BLICK) one_blink(1);
		else if(msg == THIRD_BLICK) one_blink(2);
		//else if(msg == FOURTH_BLICK) one_blink(3);
		else if(msg == FIRST_ON) led_on(0);
		else if(msg == SECOND_ON) led_on(1);
		else if(msg == THIRD_ON) led_on(2);
		//else if(msg == FOURTH_ON) led_on(3);
		else if(msg == FIRST_OFF) led_off(0);
		else if(msg == SECOND_OFF) led_off(1);
		else if(msg == THIRD_OFF) led_off(2);
		//else if(msg == FOURTH_OFF) led_off(3);
	}

}

void leds_init(void) {
	chMBObjectInit(&leds_commands, leds_commands_queue, LEDS_QUEUE);
	leds_thread = chThdCreateStatic(waLedsThread, sizeof(waLedsThread), NORMALPRIO, LedsThread, NULL);
}
