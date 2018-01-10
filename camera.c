#include "camera.h"

#define MAIN 2
#define READ 1
#define CAMERA_SERIAL &SD4

thread_t *camera_read_thread;
thread_t *camera_process_thread;

int16_t camera_read_values[NUMBER_OF_VALUES];
int16_t camera_actual_values[NUMBER_OF_VALUES];
int16_t camera_values[NUMBER_OF_VALUES];

char current_value;

THD_WORKING_AREA(waCameraReadThread, 128);
THD_FUNCTION(CameraReadThread, arg) {
	(void)arg;

	int8_t i;
	int16_t goal_degree = 420;
	int16_t goal_distance = 420;
	int16_t ball_degree = 420;
	int16_t ball_distance = 420;
	camera_read_values[4] = 0;

	while (1) {
		for(i = 0; i < 2; i++){
			current_value = sdGet(CAMERA_SERIAL);
			if(current_value == 'x') {
				goal_degree = process_value();
			} else if (current_value == 'y') {
				goal_distance = process_value();
			} else if (current_value == 'z') {
				ball_degree = process_value();
			} else if (current_value == 'i') {
				ball_distance = process_value();
			} else if (current_value == 'p') {
				camera_read_values[4] = 1;
			} else if (current_value == 's') {
				camera_read_values[4] = 0;
			} else if (current_value == 'k') {
				kick();
			} else if (current_value == 'a') {
				camera_read_values[5] = 1;
			} else if (current_value == 'b') {
				camera_read_values[5] = 0;
			}
		}

		camera_read_values[0] = goal_degree;
		camera_read_values[1] = goal_distance;
		camera_read_values[2] = ball_degree;
		camera_read_values[3] = ball_distance;

		chMsgSend(camera_process_thread, (msg_t)READ);
	}
}

THD_WORKING_AREA(waCameraProcessThread, 128);
THD_FUNCTION(CameraProcessThread, arg) {
	(void)arg;

	int8_t i;

	thread_t *master_thread;
	msg_t msg;

	while(1) {
		master_thread = chMsgWait();
		msg = chMsgGet(master_thread);
		if(msg == READ) {
			for(i = 0; i < NUMBER_OF_VALUES; i++) {
				camera_actual_values[i] = camera_read_values[i];
			}
		} else {
			for(i = 0; i < NUMBER_OF_VALUES; i++) {
				camera_values[i] = camera_actual_values[i];
			}
		}
		chMsgRelease(master_thread, MSG_OK);
	}
}

void get_camera_values(void) {
	chMsgSend(camera_process_thread, (msg_t)MAIN);
}

int16_t get_goal_degree(void) {
	return camera_values[0];
}

int16_t get_goal_distance(void) {
	return camera_values[1];
}

int16_t get_ball_degree(void) {
	return camera_values[2];
}

int16_t get_ball_distance(void) {
	return camera_values[3];
}

int16_t get_start(void) {
	return camera_values[4];
}

int16_t get_shooting_event(void) {
	return camera_values[5];
}

int16_t process_value(void) {
	int16_t val = 0;

	current_value = sdGet(&SD4);
	while(current_value != '+' && current_value != '-') {
		val += current_value - '0';
		val *= 10;
		current_value = sdGet(&SD4);
	}
	val /= 10;
	if(current_value == '-') return -val;
	return val;
}

void camera_init(void) {
	camera_read_thread = chThdCreateStatic(waCameraReadThread, sizeof(waCameraReadThread), NORMALPRIO, CameraReadThread, NULL);
	camera_process_thread = chThdCreateStatic(waCameraProcessThread, sizeof(waCameraProcessThread), NORMALPRIO, CameraProcessThread, NULL);
}
