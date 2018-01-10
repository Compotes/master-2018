#include "motor.h"

thread_t *motor_save_thread;

int16_t writing_motors_speeds[NUMBER_OF_MOTORS+NUMBER_OF_DRIBLERS];
int16_t motors_speeds[NUMBER_OF_MOTORS];
int16_t driblers_speeds[NUMBER_OF_DRIBLERS];

int16_t command = MOVE_COMMAND;

void change_motors_speeds(void) {
	chMsgSend(motor_save_thread, SAVE_MOTOR_SPEEDS);
}

void send_init_message(void) {
	int8_t i;
	sdPut(MOTOR_SERIAL, 'A');
	for(i = 0; i < NUMBER_OF_MOTORS+NUMBER_OF_DRIBLERS; i++) motor_blick(i);
	send_move_message();
}

void send_move_message(void) {
	int8_t i;
	sdPut(MOTOR_SERIAL, *((uint8_t*)&(command)+1));
	sdPut(MOTOR_SERIAL, *((uint8_t*)&(command)+0));

	for(i = 0; i < NUMBER_OF_MOTORS+NUMBER_OF_DRIBLERS; i++) {
		sdPut(MOTOR_SERIAL, *((uint8_t*)&(writing_motors_speeds[i])+1));
		sdPut(MOTOR_SERIAL, *((uint8_t*)&(writing_motors_speeds[i])+0));
	}
}

void motor_blick(int8_t motor_id){
	if(writing_motors_speeds[motor_id] < 0) writing_motors_speeds[motor_id] -= TURN_ON_MOTOR_LED;
	else writing_motors_speeds[motor_id] += TURN_ON_MOTOR_LED;
}

uint16_t motor_blick_interval(uint16_t ret){
	ret ++;
	ret %= (NUMBER_OF_MOTORS+NUMBER_OF_DRIBLERS)*MOTOR_BLIKING_TIME;
	return ret;
}


THD_WORKING_AREA(waMotorThread, 128);
THD_FUNCTION(MotorThread, arg) {
	(void)arg;

	int16_t loop = 0;
	send_init_message();


	while (1) {
		chMsgSend(motor_save_thread, LOAD_MOTOR_SPEEDS);

		motor_blick(loop/MOTOR_BLIKING_TIME);
		send_move_message();

		loop = motor_blick_interval(loop);
	}
}

THD_WORKING_AREA(waMotorSaveThread, 128);
THD_FUNCTION(MotorSaveThread, arg) {
		(void)arg;

		thread_t *master;
		msg_t command;
		int8_t i;
		int16_t motors_speeds_copy[NUMBER_OF_MOTORS+NUMBER_OF_DRIBLERS];

	while (1) {
		master = chMsgWait();
		command = chMsgGet(master);
		if(command == LOAD_MOTOR_SPEEDS){
			for(i = 0; i < NUMBER_OF_MOTORS+NUMBER_OF_DRIBLERS; i++){
				writing_motors_speeds[i] = motors_speeds_copy[i];
			}
		} else if(command == SAVE_MOTOR_SPEEDS){
			for(i = 0; i < NUMBER_OF_MOTORS; i++) {
				motors_speeds_copy[i] = motors_speeds[i];
			}
			for(i = 0; i < NUMBER_OF_DRIBLERS; i++) {
				motors_speeds_copy[NUMBER_OF_MOTORS+i] = driblers_speeds[i];
			}
		}
		chMsgRelease(master, MSG_OK);
	}
}

void motors_init(void) {
	chThdCreateStatic(waMotorThread, sizeof(waMotorThread), NORMALPRIO, MotorThread, NULL);
	motor_save_thread = chThdCreateStatic(waMotorSaveThread, sizeof(waMotorSaveThread), NORMALPRIO, MotorSaveThread, NULL);
}
