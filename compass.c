#include "compass.h"
#include "leds.h"

thread_t *compass_communication_thread;
thread_t *compass_save_thread;

int16_t attack_degree = 0;

int16_t actual_angle = -1, final_degree = -1;
int8_t sys = 0, gyro = 0, accel = 0, mag = 0;
uint32_t error_count = 0;
int16_t degree[3];
int16_t compass_degree;

THD_WORKING_AREA(waCompassCommunicationThread, 128);
THD_FUNCTION(CompassCommunicationThread, arg) {
	(void)arg;

	while (1) {
		get_calibration_status();
		get_compass_values();
		compass_degree = ((degree[0] - attack_degree) + 360) % 360;
		if (compass_degree > 180) compass_degree -= 360;
		chMsgSend(compass_save_thread, compass_degree);
		//chThdSleepMilliseconds(100);
	}
}

THD_WORKING_AREA(waCompassSaveThread, 128);
THD_FUNCTION(CompassSaveThread, arg) {
		(void)arg;

		thread_t *master;
		msg_t command;
		int16_t compass_degree_copy = 0;

	while (1) {
		master = chMsgWait();
		command = chMsgGet(master);
		if(command != LOAD_COMPASS_DEGREE){
			compass_degree_copy = command;
		}
		chMsgRelease(master, compass_degree_copy);
	}
}

int16_t get_compass_degree(void) {
	return (int16_t)chMsgSend(compass_save_thread, LOAD_COMPASS_DEGREE);
}

void compass_init(void) {
	palClearPad(GPIOC, 8);
	palSetPad(GPIOC, 9);
	start();
	compass_communication_thread = chThdCreateStatic(waCompassCommunicationThread, sizeof(waCompassCommunicationThread), NORMALPRIO, CompassCommunicationThread, NULL);
	compass_save_thread = chThdCreateStatic(waCompassSaveThread, sizeof(waCompassSaveThread), NORMALPRIO, CompassSaveThread, NULL);
}

uint8_t answer[6];


void read(uint8_t address, uint8_t length){
	uint8_t i;
	sdPut(COMPASS_SERIAL, 0xAA);
	sdPut(COMPASS_SERIAL, 0x01);
	sdPut(COMPASS_SERIAL, address);
	sdPut(COMPASS_SERIAL, length);
	answer[0] = sdGet(COMPASS_SERIAL);
	answer[1] = sdGet(COMPASS_SERIAL);
	if(answer[0] == 0xBB){
		for(i = 0; i < length; i++){
			answer[i] = sdGet(COMPASS_SERIAL);
		}
	}/* else if(answer[0] == 0xEE) {
		chprintf((BaseSequentialStream*)&SD4, "error: %d\n", answer[1]);
	} else {
		chprintf((BaseSequentialStream*)&SD4, "sprostost: %d %d\n", answer[0], answer[1]);
	}*/
}

uint8_t response[2];

void write(uint8_t address, uint8_t value){
	response[0] = 0;
	response[1] = 0;
	while(response[0] != 0xEE && response[1] != 0x01){
		sdPut(COMPASS_SERIAL, 0xAA);
		sdPut(COMPASS_SERIAL, 0x00);
		sdPut(COMPASS_SERIAL, address);//0X3D); //set mode
		sdPut(COMPASS_SERIAL, 1);
		sdPut(COMPASS_SERIAL, value);//0x09); //mode type
		response[0] = sdGet(COMPASS_SERIAL);
		response[1] = sdGet(COMPASS_SERIAL);
	}
}


void start(void){
	chprintf((BaseSequentialStream*)&SD4, "init start\n");
	sdPut(COMPASS_SERIAL, 0xAA);
	sdPut(COMPASS_SERIAL, 0x00);
	sdPut(COMPASS_SERIAL, 0x07);
	sdPut(COMPASS_SERIAL, 1);
	sdPut(COMPASS_SERIAL, 0x00);//turn on
	sdGetTimeout(COMPASS_SERIAL, 50);
	sdGetTimeout(COMPASS_SERIAL, 50);
	write(0x3D, 0x00); //mode, setup_mode
	chThdSleepMilliseconds(30);
	write(0x07, 0x00); //turn on again
	//read(0x00, 0x01); //read bno055 id
	chprintf((BaseSequentialStream*)&SD4, "turn on\n");
	/*sdPut(COMPASS_SERIAL, 0xAA);
	sdPut(COMPASS_SERIAL, 0x00);
	sdPut(COMPASS_SERIAL, 0x3F);
	sdPut(COMPASS_SERIAL, 1);
	sdPut(COMPASS_SERIAL, 0x20); //reset
	sdGet(COMPASS_SERIAL);
	chThdSleepMilliseconds(1000);*/
	//write(0x3E, 0x00); //turn on normal powermode
	/*chprintf((BaseSequentialStream*)&SD4, "reset successful\n");
	write(0x3B, (0 << 7) | // Orientation = Android
				(0 << 4) | // Temperature = Celsius
				(0 << 2) | // Euler = Degrees
				(1 << 1) | // Gyro = Rads
				(0 << 0) );// Accelerometer = m/s^2
	chprintf((BaseSequentialStream*)&SD4, "setup units\n");
	write(0x3F, 0x00); //use internal oscilator
	write(0x3D, 0x0C); //mode, NDOF
	chThdSleepMilliseconds(30);

	chprintf((BaseSequentialStream*)&SD4, "init successful\n");
	chThdSleepMilliseconds(1000);
	*/
	//write(0x3D, 0x00); //mode, setup_mode
	//chThdSleepMilliseconds(30);
	write(0x3F, 0x80); //use external oscilator
	write(0x3D, 0x0C); //mode, NDOF
	chThdSleepMilliseconds(30);
	chprintf((BaseSequentialStream*)&SD4, "extern oscilator\n");
	write(0x07, 0x00); //turn on again
	write(0x3E, 0x00); //turn on normal powermode
	sdGetTimeout(COMPASS_SERIAL, 50);
	sdGetTimeout(COMPASS_SERIAL, 50);
	sdGetTimeout(COMPASS_SERIAL, 50);
	sdGetTimeout(COMPASS_SERIAL, 50);
	while((sys != 3 && mag != 3) || error_count > 10000){
		get_calibration_status();
	}
	if(error_count > 10000) start();
	chThdSleepMilliseconds(100);
	get_compass_values();
	get_compass_values();
	attack_degree = degree[0];
	//read(0x39, 0x01);
	//chprintf((BaseSequentialStream*)&SD4, "%d\n", answer[0]);
}

void get_compass_values(void){
	int8_t i;
	sdPut(COMPASS_SERIAL, 0xAA);
	sdPut(COMPASS_SERIAL, 0x01);
	sdPut(COMPASS_SERIAL, 0x1A);
	sdPut(COMPASS_SERIAL, 6);
	answer[0] = sdGet(COMPASS_SERIAL);
	answer[1] = sdGet(COMPASS_SERIAL);
	if(answer[0] == 0xBB){
		for(i = 0; i < 3; i++){
			degree[i] = sdGet(COMPASS_SERIAL) | (sdGet(COMPASS_SERIAL) << 8);
			degree[i] /= 16;
			chprintf((BaseSequentialStream*)&SD4, "%d ", degree[i]);
		}
		sdPut(&SD4, '\n');
	} /*else if(answer[0] == 0xEE) {
		chprintf((BaseSequentialStream*)&SD4, "error: %d\n", answer[1]);
	} else {
		chprintf((BaseSequentialStream*)&SD4, "sprostost: %d %d\n", answer[0], answer[1]);
	}*/
}

void get_calibration_status(void){
	sdPut(COMPASS_SERIAL, 0xAA);
	sdPut(COMPASS_SERIAL, 0x01);
	sdPut(COMPASS_SERIAL, 0x35);
	sdPut(COMPASS_SERIAL, 1);
	answer[0] = sdGet(COMPASS_SERIAL);
	answer[1] = sdGet(COMPASS_SERIAL);
	if(answer[0] == 0xBB){
		answer[0] = sdGet(COMPASS_SERIAL);
		sys = (answer[0] >> 6) & 0x03;
		gyro = (answer[0] >> 4) & 0x03;
		accel = (answer[0] >> 2) & 0x03;
		mag = answer[0] & 0x03;
		chprintf((BaseSequentialStream*)&SD4, "%d %d %d %d ", sys, gyro, accel, mag);
	} else if(answer[0] == 0xEE) {
		error_count ++;
	}
}




