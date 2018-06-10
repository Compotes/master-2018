#include "ultrasonic.h"


void icuwidthcb(ICUDriver *icup) {
    int32_t value;

    value = icuGetWidthX(icup);

    //if (value < MIN_PULSE_LENGTH || value > MAX_PULSE_LENGTH) {
    //    value = MAX_PULSE_LENGTH;
    //}

    if(value > 3000){
		value = 0;
	} else if(value > 1800){
		value = 1;
	} else {
		value = 2;
	}
	//led_command(FIRST_BLICK);
    if (icup == &ICUD4) {
		if(value == 1){
			send_jetson(LEFT_TRUE);
		} else if(value == 2) {
			send_jetson(LEFT_CLOSE);
		} else {
			send_jetson(LEFT_FALSE);
		}
    } else if (icup == &ICUD1) {
		if(value == 1){
			send_jetson(RIGHT_TRUE);
		} else if(value == 2) {
			send_jetson(RIGHT_CLOSE);
		} else {
			send_jetson(RIGHT_FALSE);
		}
    }
}

void icuoverflowcb(ICUDriver *icup) {
	(void)icup;
}

static ICUConfig icucfg_right = {
	ICU_INPUT_ACTIVE_HIGH,
	2000000,
	icuwidthcb,
	NULL,
	icuoverflowcb,
	ICU_CHANNEL_2,
	0
};

static ICUConfig icucfg_left = {
	ICU_INPUT_ACTIVE_HIGH,
	2000000,
	icuwidthcb,
	NULL,
	icuoverflowcb,
	ICU_CHANNEL_2,
	0
};

THD_WORKING_AREA(waUltrasonicTrigerThread, 128);
THD_FUNCTION(UltrasonicTrigerThread, arg) {
	(void)arg;

	uint8_t start = FALSE;
	uint8_t ultrasonic_command;
	uint8_t loop = 0;

	while (1) {
		ultrasonic_command = check_ultrasonic_mailbox();
		if(ultrasonic_command == START_ULTRASONIC){
			start = TRUE;
			chThdSleepMilliseconds(150);
			led_command(FIRST_ON);
		} else if(ultrasonic_command == STOP_ULTRASONIC){
			start = FALSE;
			chThdSleepMilliseconds(20);
		}
		if(start){
			if(loop == 0){
				icuStop(&ICUD4);
				icuStopCapture(&ICUD4);
				icuDisableNotifications(&ICUD4);
				icuStop(&ICUD1);
				icuStopCapture(&ICUD1);
				icuDisableNotifications(&ICUD1);
				palSetPadMode(GPIOD, 13, PAL_MODE_OUTPUT_PUSHPULL);
				palSetPadMode(GPIOA, 9, PAL_MODE_OUTPUT_PUSHPULL);
				palSetPad(GPIOD, 13);
				palSetPad(GPIOA, 9);
				chThdSleepMilliseconds(10);
				palClearPad(GPIOD, 13);
				palClearPad(GPIOA, 9);
				icuStart(&ICUD4, &icucfg_left);
				icuStartCapture(&ICUD4);
				icuEnableNotifications(&ICUD4);
				palSetPadMode(GPIOD, 13, PAL_MODE_ALTERNATE(2));


				icuStart(&ICUD1, &icucfg_right);
				icuStartCapture(&ICUD1);
				icuEnableNotifications(&ICUD1);
				palSetPadMode(GPIOA, 9, PAL_MODE_ALTERNATE(1));
			}
			palSetPad(GPIOB, 10);
			chThdSleepMicroseconds(10);
			palClearPad(GPIOB, 10);
			chThdSleepMilliseconds(10);
			palSetPad(GPIOA, 10);
			chThdSleepMicroseconds(10);
			palClearPad(GPIOA, 10);
			chThdSleepMilliseconds(10);
			loop++;
			loop%=20;
		}
		chThdSleepMilliseconds(10);
	}
}

void ultrasonic_init(void){
	icuStart(&ICUD4, &icucfg_left);
	icuStartCapture(&ICUD4);
	icuEnableNotifications(&ICUD4);
	palSetPadMode(GPIOD, 13, PAL_MODE_ALTERNATE(2));


	icuStart(&ICUD1, &icucfg_right);
	icuStartCapture(&ICUD1);
	icuEnableNotifications(&ICUD1);
	palSetPadMode(GPIOA, 9, PAL_MODE_ALTERNATE(1));

	chThdCreateStatic(waUltrasonicTrigerThread, sizeof(waUltrasonicTrigerThread), NORMALPRIO, UltrasonicTrigerThread, NULL);
}

