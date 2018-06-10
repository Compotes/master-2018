#include "jetson.h"

/**
 * @addtogroup Jetcom
 * @{
 */

thread_t *jetson_write_thread;
thread_t *jetson_save_thread;

uint16_t ball_degree;
int16_t robot_speed;
int16_t robot_azimuth;

uint8_t received_command;/**< command received from jetson*/

uint16_t jetson_degree;
int16_t jetson_speed;
int16_t jetson_azimuth;


msg_t send_jetson(msg_t command){
	return chMsgSend(jetson_write_thread, command);
}

static THD_WORKING_AREA(waJetsonWriteThread, 128);
/**
 * @brief
 * ## Jetson write thread
 * Waits for the command, which then correctly sends to Jetson.
 * @param JetsonWriteThread
 * @param arg
 * @returns
 *
 *
 */
static THD_FUNCTION(JetsonWriteThread, arg) {
	(void)arg;

	thread_t *master;
	int16_t command;
	uint16_t i = 0;

	sdPut(JETSON_SERIAL, INIT_COMMAND);
	while (1) {
		master = chMsgWait();
		command = chMsgGet(master);
		chMsgRelease(master, MSG_OK);
		if(command == CALIBRATION_VALUES){
			calibration_memory(JETSON_LOAD_CALIBRATION);
			sdPut(JETSON_SERIAL, LINE_CALIBRATION_COMMAND);
			for(i = 0; i < NUMBER_OF_SENSORS; i++){
				sdPut(JETSON_SERIAL, line_calibration_values_out[i]);
				//sdPut(JETSON_SERIAL, *((uint8_t*)&(line_calibration_values_out[i])+1));
				//sdPut(JETSON_SERIAL, *((uint8_t*)&(line_calibration_values_out[i])+0));
			}
		} else {
			sdPut(JETSON_SERIAL, command);
		}
	}
}


static THD_WORKING_AREA(waJetsonReadThread, 128);
/**
 * @brief
 * ## Jetson read thread
 * Receives all commands from Jetson and sends them for further processing.
 * @param JetsonReadThread
 * @param arg
 * @returns
 *
 *
 */
static THD_FUNCTION(JetsonReadThread, arg) {
	(void)arg;

	int16_t i;

    while (1) {
		received_command = sdGet (JETSON_SERIAL);
		//sdPut(&SD3, received_command);

		if(received_command == JETSON_MOVE_COMMAND){
			jetson_degree = sdGet(JETSON_SERIAL);
			//sdPut(&SD3, jetson_degree);
			jetson_speed = (int16_t)sdGet(JETSON_SERIAL) - 100;
			//sdPut(&SD3, jetson_speed);
			jetson_azimuth = (int16_t)sdGet(JETSON_SERIAL) - 100;
			//sdPut(&SD3, jetson_azimuth);
			if(jetson_speed < 0){
				jetson_degree += 180;
				jetson_speed = -jetson_speed;
				jetson_degree %= 360;
			}
			chMsgSend(jetson_save_thread, SAVE_JETSON_VALUES);
		} else if(received_command == LINE_CALIBRATION_COMMAND){
			send_to_line_mailbox(CALIBRATION);
			//chprintf((BaseSequentialStream *)&SD4, "jetson: som tu\n");
		} else if(received_command == INIT_COMMAND){
			//led_command(FOURTH_ON);
			for(i = 0; i < NUMBER_OF_SENSORS; i++){
				line_calibration_values_in[i] = sdGet(JETSON_SERIAL);
				//line_calibration_values_in[i] = (line_calibration_values_in[i] << 8) | sdGet (JETSON_SERIAL);
				//chprintf((BaseSequentialStream *)&SD4, "jetson: nacital som %d\n", line_calibration_values_in[i]);
			}
			calibration_memory(JETSON_SAVE_CALIBRATION);
			send_to_line_mailbox(LOAD_JETSON_CALIBRATION);
		} else if(received_command == START_COMMAND){
			send_to_main_mailbox(START);
		}	else if(received_command == STOP_COMMAND){
			send_to_main_mailbox(STOP);
		} else if(received_command == DRIBLER_COMMAND){
			send_to_main_mailbox(DRIBLER_ON_OFF+sdGet(JETSON_SERIAL));
		} else if(received_command == KICK_COMMAND){
			send_to_main_mailbox(KICK);
			//chprintf((BaseSequentialStream *)&SD4, "jetson: kick\n");
		} else if(received_command == START_ULTRASONIC_COMMAND){
			//led_command(FIRST_ON);
			send_to_ultrasonic_mailbox(START_ULTRASONIC);
		} else if(received_command == STOP_ULTRASONIC_COMMAND){
			send_to_ultrasonic_mailbox(STOP_ULTRASONIC);
		}
	}
}


msg_t get_jetson_values(void){
	return chMsgSend(jetson_save_thread, LOAD_JETSON_VALUES);
}

static THD_WORKING_AREA(waJetsonSaveThread, 128);
/**
 * @brief
 * ## Jetson save thread
 * Makes copies of movement values from [Jetson read thread]
 * (@ref THD_FUNCTION(JetsonReadThread, arg)) and makes them available
 * from outside with function @ref get_jetson_values.
 * @param JetsonSaveThread
 * @param arg
 * @returns
 *
 *
 */
static THD_FUNCTION(JetsonSaveThread, arg) {
	(void)arg;

	thread_t *master;
	msg_t command;
	uint16_t copy_jetson_degree = 0;
	int16_t copy_jetson_speed = 0;
	int16_t copy_jetson_azimuth = 0;

	while (1) {
		master = chMsgWait();
		command = chMsgGet(master);
		if(command == SAVE_JETSON_VALUES){
			copy_jetson_degree = jetson_degree;
			copy_jetson_speed = jetson_speed;
			copy_jetson_azimuth = jetson_azimuth;
		} else if(command == LOAD_JETSON_VALUES){
			ball_degree = copy_jetson_degree;
			robot_speed = copy_jetson_speed;
			robot_azimuth = copy_jetson_azimuth;
		}
		chMsgRelease(master, MSG_OK);
	}
}

static THD_WORKING_AREA(waButtonGoThread, 128);
/**
 * @brief
 * ## Button go thread
 * check go button on top of robot
 * @param ButtonGoThread
 * @param arg
 * @returns
 *
 *
 */
static THD_FUNCTION(ButtonGoThread, arg) {
	(void)arg;

	int8_t old_state = palReadPad(GPIOD, 14);
	int8_t new_state;
	while (1) {
		if(palReadPad(GPIOD, 14)){
			new_state = 100;
		}
		if((old_state == 0 && new_state > 0) || (old_state > 0 && new_state == 0)){
			old_state = new_state;
			send_to_main_mailbox(START_STOP);
		}
		chThdSleepMilliseconds(1);
		new_state--;
	}
}

void jetson_init(void) {
	jetson_write_thread = chThdCreateStatic(waJetsonWriteThread, sizeof(waJetsonWriteThread), NORMALPRIO, JetsonWriteThread, NULL);
	jetson_save_thread = chThdCreateStatic(waJetsonSaveThread, sizeof(waJetsonSaveThread), NORMALPRIO, JetsonSaveThread, NULL);
	chThdCreateStatic(waJetsonReadThread, sizeof(waJetsonReadThread), NORMALPRIO, JetsonReadThread, NULL);
	//chThdCreateStatic(waButtonGoThread, sizeof(waButtonGoThread), NORMALPRIO, ButtonGoThread, NULL);
}

/**
 * @}
 */
