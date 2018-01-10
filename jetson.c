#include "jetson.h"

thread_t *jetson_write_thread;

int16_t received_command;


THD_WORKING_AREA(waJetsonWriteThread, 128);
THD_FUNCTION(JetsonWriteThread, arg) {
		(void)arg;

		thread_t *master;
		int16_t command;

	while (1) {
		master = chMsgWait();
		command = chMsgGet(master);
		chMsgRelease(master, MSG_OK);

		sdPut (&SD1, *((uint8_t*)&(command)+1));
		sdPut (&SD1, *((uint8_t*)&(command)+0));
	}
}

int16_t receive_command(void){
	int16_t ret = 'A';
	while(ret != START_JETSON_COMMAND){
		ret = sdGet (JETSON_SERIAL);
	}
	ret = (ret << 8) | sdGet (JETSON_SERIAL);
	return ret;
}

void jetson_communication_init(void){
	uint8_t init_command = sdGet (JETSON_SERIAL);
    while(init_command != 'A'){
		init_command = sdGet (JETSON_SERIAL);
	}
}

THD_WORKING_AREA(waJetsonReadThread, 128);
THD_FUNCTION(JetsonReadThread, arg) {
	(void)arg;

	jetson_communication_init();

    while (1) {
		received_command = receive_command();
		if(received_command == LINE_CALIBRATION_COMMAND){
			send_line_calibration();
		}
	}
}

void jetson_init(void) {
	jetson_write_thread = chThdCreateStatic(waJetsonWriteThread, sizeof(waJetsonWriteThread), NORMALPRIO, JetsonWriteThread, NULL);
	chThdCreateStatic(waJetsonReadThread, sizeof(waJetsonReadThread), NORMALPRIO, JetsonReadThread, NULL);
}
