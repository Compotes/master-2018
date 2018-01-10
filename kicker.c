#include "kicker.h"

int8_t kicker_is_ready(void){
	return palReadPad(READY_GPIO, READY_PIN);
}

int8_t i_have_ball(void){
	return palReadPad(BALL_DETECT_GPIO, BALL_DETECT_PIN);
}

void kick(void){
	palSetPad(KICK_GPIO, KICK_PIN);
	chThdSleepMilliseconds(KICK_DURATION);
	palClearPad(KICK_GPIO, KICK_PIN);
}
