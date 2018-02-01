#include "lines.h"



#define PI 3.14159
#define SQRT3 1.7321

thread_t *line_save_thread;
thread_t *line_calibration_save_thread;
msg_t msg;


static msg_t line_commands_queue[LINE_QUEUE];
static mailbox_t line_commands;

adcsample_t line_sensors_values[NUMBER_OF_SENSORS];
uint16_t min_line_sensors_values[NUMBER_OF_SENSORS];
uint16_t max_line_sensors_values[NUMBER_OF_SENSORS];
uint16_t average_line_sensors_values[NUMBER_OF_SENSORS];
uint8_t line_calibration_values_out[NUMBER_OF_SENSORS];
uint8_t line_calibration_values_in[NUMBER_OF_SENSORS];

int32_t line_motor_speeds[NUMBER_OF_MOTORS];

static int16_t dx[16] = {83, 56, 20, -20, -56, -83, -98, -98, -83, -56, -20,  20,  56,  83,  98, 98};
static int16_t dy[16] = {56, 83, 98,  98,  83,  56,  20, -20, -56, -83, -98, -98, -83, -56, -20, 20};
static int16_t line_result;
static int32_t res_x, res_y, res_max, old_x, old_y, old_max;
uint32_t line_timer = LINE_REACTION_TIME;
uint32_t line_calibration_timer = LINE_CALIBRATION_TIME;
int8_t line_calibration_state = 0;

static const ADCConversionGroup line_sensors_cfg1 = {
	TRUE,//FALSE,
	6,
	NULL,
	NULL,
	0,
	ADC_CR2_SWSTART,
	0,
	ADC_SMPR2_SMP_AN1(ADC_SAMPLE_3) | ADC_SMPR2_SMP_AN2(ADC_SAMPLE_3) |
	ADC_SMPR2_SMP_AN3(ADC_SAMPLE_3) | ADC_SMPR2_SMP_AN4(ADC_SAMPLE_3) |
	ADC_SMPR2_SMP_AN5(ADC_SAMPLE_3) | ADC_SMPR2_SMP_AN6(ADC_SAMPLE_3),
	ADC_SQR1_NUM_CH(6),
	0,
	ADC_SQR3_SQ1_N(ADC_CHANNEL_IN1) | ADC_SQR3_SQ2_N(ADC_CHANNEL_IN2) |
	ADC_SQR3_SQ3_N(ADC_CHANNEL_IN3) | ADC_SQR3_SQ4_N(ADC_CHANNEL_IN4) |
	ADC_SQR3_SQ5_N(ADC_CHANNEL_IN5) | ADC_SQR3_SQ6_N(ADC_CHANNEL_IN6)
};

static const ADCConversionGroup line_sensors_cfg2 = {
	TRUE,//FALSE,
	5,
	NULL,
	NULL,
	0,
	ADC_CR2_SWSTART,
	ADC_SMPR1_SMP_AN14(ADC_SAMPLE_3) | ADC_SMPR1_SMP_AN15(ADC_SAMPLE_3),
	ADC_SMPR2_SMP_AN7(ADC_SAMPLE_3) | ADC_SMPR2_SMP_AN8(ADC_SAMPLE_3) |
	ADC_SMPR2_SMP_AN9(ADC_SAMPLE_3),
	ADC_SQR1_NUM_CH(5),
	0,
	ADC_SQR3_SQ1_N(ADC_CHANNEL_IN7) | ADC_SQR3_SQ2_N(ADC_CHANNEL_IN14) |
	ADC_SQR3_SQ3_N(ADC_CHANNEL_IN15) | ADC_SQR3_SQ4_N(ADC_CHANNEL_IN8) |
	ADC_SQR3_SQ5_N(ADC_CHANNEL_IN9)
};

static const ADCConversionGroup line_sensors_cfg3 = {
	TRUE,//FALSE,
	5,
	NULL,
	NULL,
	0,
	ADC_CR2_SWSTART,
	ADC_SMPR1_SMP_AN10(ADC_SAMPLE_3) | ADC_SMPR1_SMP_AN11(ADC_SAMPLE_3) |
	ADC_SMPR1_SMP_AN12(ADC_SAMPLE_3) | ADC_SMPR1_SMP_AN13(ADC_SAMPLE_3),
	ADC_SMPR2_SMP_AN0(ADC_SAMPLE_3),
	ADC_SQR1_NUM_CH(5),
	0,
	ADC_SQR3_SQ1_N(ADC_CHANNEL_IN10) | ADC_SQR3_SQ2_N(ADC_CHANNEL_IN11) |
	ADC_SQR3_SQ3_N(ADC_CHANNEL_IN12) | ADC_SQR3_SQ4_N(ADC_CHANNEL_IN13) |
	ADC_SQR3_SQ5_N(ADC_CHANNEL_IN0)
};

static const ADCConversionGroup line_sensors_cfg = {
	FALSE,
	NUMBER_OF_SENSORS,
	NULL,
	NULL,
	0,
	ADC_CR2_SWSTART,
	ADC_SMPR1_SMP_AN10(ADC_SAMPLE_3) | ADC_SMPR1_SMP_AN11(ADC_SAMPLE_3) |
	ADC_SMPR1_SMP_AN12(ADC_SAMPLE_3) | ADC_SMPR1_SMP_AN13(ADC_SAMPLE_3) |
	ADC_SMPR1_SMP_AN14(ADC_SAMPLE_3) | ADC_SMPR1_SMP_AN15(ADC_SAMPLE_3),
	ADC_SMPR2_SMP_AN0(ADC_SAMPLE_3) | ADC_SMPR2_SMP_AN1(ADC_SAMPLE_3) |
	ADC_SMPR2_SMP_AN2(ADC_SAMPLE_3) | ADC_SMPR2_SMP_AN3(ADC_SAMPLE_3) |
	ADC_SMPR2_SMP_AN4(ADC_SAMPLE_3) | ADC_SMPR2_SMP_AN5(ADC_SAMPLE_3) |
	ADC_SMPR2_SMP_AN6(ADC_SAMPLE_3) | ADC_SMPR2_SMP_AN7(ADC_SAMPLE_3) |
	ADC_SMPR2_SMP_AN8(ADC_SAMPLE_3) | ADC_SMPR2_SMP_AN9(ADC_SAMPLE_3),
	ADC_SQR1_SQ13_N(ADC_CHANNEL_IN11) | ADC_SQR1_SQ14_N(ADC_CHANNEL_IN12) |
	ADC_SQR1_SQ15_N(ADC_CHANNEL_IN13) | ADC_SQR1_SQ16_N(ADC_CHANNEL_IN0) |
	ADC_SQR1_NUM_CH(NUMBER_OF_SENSORS),
	ADC_SQR2_SQ7_N(ADC_CHANNEL_IN7) | ADC_SQR2_SQ8_N(ADC_CHANNEL_IN14) |
	ADC_SQR2_SQ9_N(ADC_CHANNEL_IN15) | ADC_SQR2_SQ10_N(ADC_CHANNEL_IN8) |
	ADC_SQR2_SQ11_N(ADC_CHANNEL_IN9) | ADC_SQR2_SQ12_N(ADC_CHANNEL_IN10),
	ADC_SQR3_SQ1_N(ADC_CHANNEL_IN1) | ADC_SQR3_SQ2_N(ADC_CHANNEL_IN2) |
	ADC_SQR3_SQ3_N(ADC_CHANNEL_IN3) | ADC_SQR3_SQ4_N(ADC_CHANNEL_IN4) |
	ADC_SQR3_SQ5_N(ADC_CHANNEL_IN5) | ADC_SQR3_SQ6_N(ADC_CHANNEL_IN6)
};


int32_t abs_int(int32_t x){
	if(x<0)x*=-1;
	return x;
}

int32_t max(int32_t x, int32_t y){
	if(x<y)return y;
	return x;
}

void calculation_of_motor_speeds(void){
	int8_t i;
	int32_t maximum;

	res_y *= 500*SQRT3;
	res_x *= 1000;
	line_motor_speeds[2] = -res_x;
	line_motor_speeds[0] = res_x/2 - res_y;
	line_motor_speeds[1] = res_x/2 + res_y;
	maximum = max(abs_int(line_motor_speeds[2]), max(abs_int(line_motor_speeds[0]), abs_int(line_motor_speeds[1])));
	for(i = 0; i < NUMBER_OF_MOTORS; i++){
		line_motor_speeds[i] = line_motor_speeds[i]*1000/maximum;
	}
}



void determine_avoiding_direction(void) {
	int8_t i;
	res_x = 0, res_y = 0, res_max = 0;

	/*for(i = 0; i < NUMBER_OF_SENSORS; i++) {
		round_data_values[i] *= INERTIA;
		if(line_sensors_values[i] < 500){
			round_data_values[i] += (1.000 - INERTIA);
		}
		if (round_data_values[i] > 0.99 - INERTIA) {
			res_x += dx[i];
			res_y += dy[i];
		}
	}*/
	for(i = 0; i < NUMBER_OF_SENSORS; i++) {
		if(line_sensors_values[i] < average_line_sensors_values[i]){
			res_x += dx[i];
			res_y += dy[i];
		}
	}
	if(abs_int(res_x)>abs_int(res_y)) res_max = res_x;
	else res_max = res_y;

	if(line_timer == LINE_REACTION_TIME && res_max == 0){
		line_result = NO_LINE_DETECTED;
	} else {
		if(res_max == 0){
			res_x = old_x;
			res_y = old_y;
			line_timer++;
		} else {
			line_timer = 0;
		}
		if((old_max < 0 && res_max > 0) || (old_max > 0 && res_max < 0)){
			res_x = old_x;
			res_y = old_y;
		}
		if(line_result == NO_LINE_DETECTED){
			old_x = res_x;
			old_y = res_y;
			old_max = res_max;
		}
		line_result = LINE_DETECTED;
		calculation_of_motor_speeds();
	}

	//chprintf(&SD4, "calculate_lines: %d %d\n", res_x, res_y);
}

void calibrate_lines(void) {
	int8_t i;
	line_result = LINE_CALIBRATION;
	for(i = 0; i < NUMBER_OF_MOTORS; i++){
		line_motor_speeds[i] = 0;
	} for(i = 0; i < NUMBER_OF_SENSORS; i++){
		if(line_sensors_values[i] < min_line_sensors_values[i]){
			min_line_sensors_values[i] = line_sensors_values[i];
		} else if(line_sensors_values[i] > max_line_sensors_values[i]){
			max_line_sensors_values[i] = line_sensors_values[i];
		}
	}

}

void end_line_calibration(void) {
	int8_t i;
	for(i = 0; i < NUMBER_OF_SENSORS; i++){
		average_line_sensors_values[i] = (max_line_sensors_values[i]+min_line_sensors_values[i])/2;
		chprintf((BaseSequentialStream *)&SD4, "%d ", average_line_sensors_values[i]);
	}
	chprintf((BaseSequentialStream *)&SD4, "\n");
	calibration_memory(LINE_SAVE_CALIBRATION);
	chprintf((BaseSequentialStream *)&SD4, "lines: ulozil som\n");
	send_jetson(CALIBRATION_VALUES);
}

msg_t check_line_mailbox(void){
	msg_t ret = 0;
	chMBFetch(&line_commands, &ret, TIME_IMMEDIATE);
	//chThdSleepMilliseconds(10);
	return ret;
}

void send_to_line_mailbox(msg_t sending_command){
	chMBPost(&line_commands, sending_command, TIME_INFINITE);
}

THD_WORKING_AREA(waLineThread, 128);
THD_FUNCTION(LineThread, arg) {
		(void)arg;


		int16_t l = 0;
		msg_t line_calibration_command;
	while (1) {
		adcConvert(&ADCD1, &line_sensors_cfg, line_sensors_values, 1);
		line_calibration_command = check_line_mailbox();
		if(line_calibration_command == CALIBRATION){
			//line_calibration_timer = 0;
			if(line_calibration_state == 0){
				line_calibration_state = 1;
			} else {
				line_calibration_state = 0;
				end_line_calibration();
			}
		} else if(line_calibration_command == LOAD_JETSON_CALIBRATION){
			calibration_memory(LINE_LOAD_CALIBRATION);
		}
		if(line_calibration_state == 1/*line_calibration_timer < LINE_CALIBRATION_TIME*/){
			calibrate_lines();
			//line_calibration_timer++;
		} else {
			determine_avoiding_direction();
		}
		chMsgSend(line_save_thread, line_result);
		if(l == 0) led_command(SECOND_BLICK);
		l++;
		l%=2000;
	}
}

msg_t check_line(void){
	return chMsgSend(line_save_thread, LOAD_LINES_SPEEDS);
}

msg_t calibration_memory(msg_t command){
	return chMsgSend(line_calibration_save_thread, command);
}

THD_WORKING_AREA(waLineSaveThread, 128);
THD_FUNCTION(LineSaveThread, arg) {
		(void)arg;

		thread_t *master;
		msg_t command;
		int8_t i, line_state = NO_LINE_DETECTED;
		int16_t line_motor_speeds_copy[NUMBER_OF_MOTORS];

	while (1) {
		master = chMsgWait();
		command = chMsgGet(master);
		if(command == NO_LINE_DETECTED || command == LINE_CALIBRATION){
			line_state = command;
		} else if(command == LOAD_LINES_SPEEDS && line_state != NO_LINE_DETECTED && command != LINE_CALIBRATION){
			for(i = 0; i < NUMBER_OF_MOTORS; i++){
				motors_speeds[i] = line_motor_speeds_copy[i];
			}
		} else if(command == LINE_DETECTED){
			line_state = LINE_DETECTED;
			for(i = 0; i < NUMBER_OF_MOTORS; i++){
				line_motor_speeds_copy[i] = line_motor_speeds[i];
			}
		}
		chMsgRelease(master, line_state);
	}
}

THD_WORKING_AREA(waLineCalibrationSaveThread, 128);
THD_FUNCTION(LineCalibrationSaveThread, arg) {
		(void)arg;

		thread_t *master;
		msg_t command;
		int8_t i;
		uint16_t copy_line_calibration_values[NUMBER_OF_SENSORS];

	while (1) {
		master = chMsgWait();
		command = chMsgGet(master);
		if(command == JETSON_SAVE_CALIBRATION){
			for(i = 0; i < NUMBER_OF_SENSORS; i++){
				copy_line_calibration_values[i] = (line_calibration_values_in[i])*20;
			}
		} else if(command == JETSON_LOAD_CALIBRATION){
			for(i = 0; i < NUMBER_OF_SENSORS; i++){
				line_calibration_values_out[i] = copy_line_calibration_values[i]/20;
			}
		} else if(command == LINE_SAVE_CALIBRATION){
			for(i = 0; i < NUMBER_OF_SENSORS; i++){
				copy_line_calibration_values[i] = average_line_sensors_values[i];
			}
		} else if(command == LINE_LOAD_CALIBRATION){
			for(i = 0; i < NUMBER_OF_SENSORS; i++){
				average_line_sensors_values[i] = copy_line_calibration_values[i];
			}
		}
		chMsgRelease(master, MSG_OK);
	}
}

void line_init(void) {
	//adcStartConversion(&ADCD1, &line_sensors_cfg1, &line_sensors_values[0],1);
	//adcStartConversion(&ADCD2, &line_sensors_cfg2, &line_sensors_values[6],1);
	//adcStartConversion(&ADCD3, &line_sensors_cfg3, &line_sensors_values[11],1);

	chMBObjectInit(&line_commands, line_commands_queue, LINE_QUEUE);

	chThdCreateStatic(waLineThread, sizeof(waLineThread), NORMALPRIO, LineThread, NULL);
	line_save_thread = chThdCreateStatic(waLineSaveThread, sizeof(waLineSaveThread), NORMALPRIO, LineSaveThread, NULL);
	line_calibration_save_thread = chThdCreateStatic(waLineCalibrationSaveThread, sizeof(waLineCalibrationSaveThread), NORMALPRIO, LineCalibrationSaveThread, NULL);
}
