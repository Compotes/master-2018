#include "ch.h"
#include "hal.h"
#include "test.h"
#include "camera.h"
#include "config.h"
#include "compass.h"
#include "lines.h"
#include "motor.h"
#include "kicker.h"
#include "leds.h"
#include "dip_switch.h"
#include "jetson.h"
#include "chprintf.h"
#include "ultrasonic.h"
#include <math.h>

#define MAX_SPEED 923
#define MAX_ROBOT_POWER 15
#define MIN_ROBOT_POWER 40
#define FAR_POINT 0
#define NEER_POINT 400
#define ALIGN_POWER 50

#define DRIBLER_SPEED -1900

#define PI 3.14159

#define ALIGN_TOLLERANCE 5

#define SHOOTING_RANGE 5
#define HAVING_BALL_RANGE 5

#define OBJECT_NOT_FOUND 420
#define SEARCH_SPEED 20

#define DRIBLER_REACTION_TIME 600

#define CAMERA_CENTER 4
#define AROUND_BALL_ALIGN 0
#define BASIC_ALIGN 1
#define CLASSIC_ALIGN 2
#define NOT_ALIGN 3
#define LINE_CALIBRATION_ALIGN 4


//int16_t true_speeds[101] = {0, 1118, 1224, 1291, 1340, 1380, 1413, 1442, 1468, 1490, 1511, 1530, 1548, 1564, 1579, 1593, 1607, 1620, 1632, 1644, 1655, 1665, 1675, 1685, 1695, 1704, 1712, 1721, 1729, 1737, 1745, 1752, 1760, 1767, 1774, 1780, 1787, 1793, 1800, 1806, 1812, 1818, 1823, 1829, 1835, 1840, 1845, 1850, 1856, 1861, 1866, 1870, 1875, 1880, 1884, 1889, 1893, 1898, 1902, 1906, 1911, 1915, 1919, 1923, 1927, 1931, 1935, 1938, 1942, 1946, 1950, 1953, 1957, 1960, 1964, 1967, 1971, 1974, 1977, 1981, 1984, 1987, 1990, 1994, 1997, 2000, 2003, 2006, 2009, 2012, 2015, 2018, 2021, 2023, 2026, 2029, 2032, 2035, 2037, 2040, 2042};

int16_t movement_degree = 90;
int16_t old_line = 0;

int16_t dribler_timer = 0;


int8_t have_ball = 0;

int8_t using_align = CLASSIC_ALIGN;

int16_t loop = 0;
int32_t move_time = 600;
int32_t stop_time = 24000;

int16_t sinus[91] = {0, 17, 34, 52, 69, 87, 104, 121, 139, 156, 173, 190, 207, 224, 241, 258, 275, 292, 309, 325, 342, 358, 374, 390, 406, 422, 438, 453, 469, 484, 499, 515, 529, 544, 559, 573, 587, 601, 615, 629, 642, 656, 669, 681, 694, 707, 719, 731, 743, 754, 766, 777, 788, 798, 809, 819, 829, 838, 848, 857, 866, 874, 882, 891, 898, 906, 913, 920, 927, 933, 939, 945, 951, 956, 961, 965, 970, 974, 978, 981, 984, 987, 990, 992, 994, 996, 997, 998, 999, 999, 1000};


int16_t sinn(int16_t degree){
	degree += 360;
	degree %= 360;
	if(degree < 90) return sinus[degree];
	if(degree < 180) return sinus[90-degree%90];
	if(degree < 270) return -sinus[degree%90];
	return -sinus[90-degree%90];
}

/*int16_t robot_power(void){
	//return 50;
	//return ((-ball_distance+650)/4 < 100)? 100 : (-ball_distance+650)/4;
	int16_t a = MAX_ROBOT_POWER - (ball_distance - FAR_POINT)*(MAX_ROBOT_POWER - MIN_ROBOT_POWER)/(NEER_POINT-FAR_POINT); // rychlost_rastu = 4, celkova_rychlost = 650
	if(a > MAX_ROBOT_POWER) a = MAX_ROBOT_POWER;
	if(a < 0) a = 0;
	if(dribler_timer > 0  && dribler_timer < 6) a = 100;
	if(ball_degree == OBJECT_NOT_FOUND) a = MAX_ROBOT_POWER;
	a = MAX_ROBOT_POWER;
	return a;
}*/

int16_t abs_value_int(int16_t a) {
	if (a < 0) a *= -1;
	return a;
}

void motors_off(void) {
	int8_t p;
	for (p = 0; p < NUMBER_OF_MOTORS; p++) {
		motors_speeds[p] = 0;
	}
}

int16_t set_motor_speed(int16_t relative_speed) {
	return relative_speed * MAX_SPEED / 100;
}

void set_dribler_speed(int32_t dribler_speed){
	int8_t p;
	for (p = 0; p < NUMBER_OF_DRIBLERS; p++) {
		driblers_speeds[p] = set_motor_speed(dribler_speed);

		if(p%2==1) driblers_speeds[p] *= -1;

		if (driblers_speeds[p] > 0) {
			driblers_speeds[p] += 1125;
		} else if (driblers_speeds[p] < 0) {
			driblers_speeds[p] -= 1125;
		}
	}
}

void set_movement(int16_t degree) {
	int8_t p;
	for (p = 0; p < NUMBER_OF_MOTORS; p++) {
		motors_speeds[p] = sinn(degree - 90*(NUMBER_OF_MOTORS - 2 + (4 * p))/ NUMBER_OF_MOTORS);
	}
}


void correct_motors_speeds(int8_t align_type, int16_t speed, int16_t align_speed) {
	int8_t p;
	int32_t max = 0;
	if(align_type == NOT_ALIGN){
		align_speed = 0;
	}

	for (p = 0; p < NUMBER_OF_MOTORS; p++) {
		if (abs_value_int(motors_speeds[p]) > max) {
			max = abs_value_int(motors_speeds[p]);
		}
	}

	for (p = 0; p < NUMBER_OF_MOTORS; p++) {
		//if(motors_speeds[p] == max) motors_speeds[p] *= 1.5;
		motors_speeds[p] = motors_speeds[p] * set_motor_speed((100 - abs_value_int(align_speed))*speed/100) / max;//(max*1.5);
	}
	//chprintf((BaseSequentialStream *)&SD4, "%d %d %d %d\n", robot_azimuth, motors_speeds[3], motors_speeds[1], motors_speeds[2]);
	for (p = 0; p < NUMBER_OF_MOTORS; p++) {
		motors_speeds[p] += set_motor_speed((align_speed)*speed/100);
	}

	if(align_type == AROUND_BALL_ALIGN) {
		motors_speeds[0] = 0;
		motors_speeds[1] = 0;
		motors_speeds[2] *= 3;
	}

	for (p = 0; p < NUMBER_OF_MOTORS; p ++) {
		//if(motors_speeds[p] > 0) motors_speeds[p] = 846.8312145313 * pow((double)motors_speeds[p], 0.1249204791);
		//else motors_speeds[p] = -(846.8312145313 * pow(-(double)motors_speeds[p], 0.1249204791));
		if (motors_speeds[p] > 0) {
			motors_speeds[p] += 1125;
		} else if (motors_speeds[p] < 0) {
			motors_speeds[p] -= 1125;
		}
	}
}

/*
void left_right(int8_t degree){
	loop %= 2*move_time + 2*stop_time;
	if(loop < move_time/2){
		set_movement(degree);
		correct_motors_speeds(using_align, 2*loop*MAX_ROBOT_POWER/move_time);
	}else if(loop < move_time){
		set_movement(degree);
		correct_motors_speeds(using_align, (move_time-2*(loop - move_time/2))*MAX_ROBOT_POWER/move_time);
	}
	else if(loop < move_time + stop_time) motors_off();
	else if(loop < 1.5*move_time + stop_time){
		set_movement((degree+180)%360);
		correct_motors_speeds(using_align, MAX_ROBOT_POWER/2);//2*(loop - move_time - stop_time)*MAX_ROBOT_POWER/move_time);
	}else if(loop < 2*move_time + stop_time){
		set_movement((degree+180)%360);
		correct_motors_speeds(using_align, MAX_ROBOT_POWER/2);//(move_time-2*(loop - 1.5*move_time - stop_time))*MAX_ROBOT_POWER/move_time);
	}
	else motors_off();
}

void circle(int32_t pseudo_diameter){
	loop %= 36000/4;
	set_movement(((loop)/25)%360);
	correct_motors_speeds(using_align, MAX_ROBOT_POWER/2);
}

void eight(int32_t pseudo_diameter){
	loop %= 360*2/pseudo_diameter;
	if(loop < 360/pseudo_diameter/2){
		set_movement(45);
		correct_motors_speeds(using_align, MAX_ROBOT_POWER/5);
	} else if(loop < 360/pseudo_diameter){
		set_movement((loop*pseudo_diameter)%360);
		correct_motors_speeds(using_align, MAX_ROBOT_POWER/2);
	} else if(loop < 360/pseudo_diameter*4/2){
		set_movement(225);
		correct_motors_speeds(using_align, MAX_ROBOT_POWER/5);
	} else if(loop < 360/pseudo_diameter*3){
		set_movement((-(loop-360/pseudo_diameter)*pseudo_diameter)%360);
		correct_motors_speeds(using_align, MAX_ROBOT_POWER/2);
	} else {
		set_movement(45);
		correct_motors_speeds(using_align, MAX_ROBOT_POWER/5);
	}
	correct_motors_speeds(using_align, MAX_ROBOT_POWER/2);
}*/


int main(void) {
	halInit();
	chSysInit();

	board_init();
	leds_init();
	jetson_init();
	motors_init();
	//camera_init();
	line_init();
	ultrasonic_init();
	//compass_init();

	int16_t line_state;
	int32_t l = 0;
	uint8_t main_command;
	uint8_t go = FALSE;
	uint8_t i = 0;
	int16_t dribler_speed = 0;
	int16_t button;
	while (1) {
		line_state = check_line();
		get_jetson_values();//actualize ball_degree, robot_speed, robot_azimuth;
		//chprintf(&SD3, "%d %d %d\r\n", ball_degree, robot_speed, robot_azimuth);
		main_command = check_main_mailbox();
		/*if(main_command == START){
			send_jetson(START_COMMAND);
			go = TRUE;
			led_command(FIRST_ON);
			chThdSleepMilliseconds(10);
		} else if(main_command == STOP){
			send_jetson(STOP_COMMAND);
			go = FALSE;
			led_command(FIRST_OFF);
			chThdSleepMilliseconds(10);
		} else if(main_command == START_STOP){
			if(go){
				send_jetson(STOP_COMMAND);
				go = FALSE;
				led_command(FIRST_OFF);
				chThdSleepMilliseconds(10);
			} else {
				send_jetson(START_COMMAND);
				go = TRUE;
				led_command(FIRST_ON);
				chThdSleepMilliseconds(10);
			}
		} else*/ if(main_command >= DRIBLER_ON_OFF){
			dribler_speed = main_command - DRIBLER_ON_OFF - 100;
			chThdSleepMilliseconds(10);
		}

		if(palReadPad(GPIOD, 14)){
			button = 10;
		}

		if(!go && button > 0){
			go = TRUE;
			led_command(FIRST_ON);
			send_jetson(START_COMMAND);
		} else if(go && button == 0){
			go = FALSE;
			led_command(FIRST_OFF);
			send_jetson(STOP_COMMAND);
		}
		//chprintf((BaseSequentialStream*)&SD4, "%d\n", get_compass_degree());
		//get_camera_values();
		//chprintf(&SD4, "%d\n", get_dip_switch());

		if(go || get_start()){
			if(line_state == NO_LINE_DETECTED){
				set_movement(ball_degree);
				//for(i = 0; i < NUMBER_OF_MOTORS; i++){
				//	chprintf(&SD4, "%d ", motors_speeds[i]);
				//}
				//chprintf(&SD4, "\n");
				correct_motors_speeds(using_align, robot_speed, robot_azimuth);
			} else if(line_state == LINE_CALIBRATION){
				using_align = LINE_CALIBRATION_ALIGN;
				correct_motors_speeds(using_align, robot_speed, robot_azimuth);
				//chprintf(&SD4, "main: som tu\n");
			} else {
				correct_motors_speeds(using_align, 60, robot_azimuth);
			}
			//loop++;
		} else {
			motors_off();
		}

		set_dribler_speed(dribler_speed);

		if(i_have_ball() || main_command == KICK || dribler_timer > 0){
			set_dribler_speed(0);
			if(dribler_timer > 0 ){
				dribler_timer--;
			} else {
				dribler_timer = DRIBLER_REACTION_TIME;
			}
			//change_motors_speeds(0, 0, 0, 0, 0, MASTER_THREAD_ID);
			if(dribler_timer == 1) {
				//chThdSleepMilliseconds(200);
				kick();
			}
		} else {
			dribler_timer = 0;
			//dribler_on();
		}
		//dribler_on();
		//loop++;
		//left_right(20);
		//dribler_off();
		//motors_off();
		//if(loop > 30) motors_off();
		//motors_speeds[0] = -1500;
		//motors_off();
		//chprintf(&SD4,"%d\r\n", check_line());
		//dribler_on();

		//correct_motors_speeds(using_align, robot_speed, robot_azimuth);
		//motors_off();

		//motors_speeds[1]=DRIBLER_SPEED;
		change_motors_speeds();
		if(l == 0) {
			led_command(THIRD_BLICK);
			//kick();
		}
		l++;
		l%=2000;
		if(button > 0){
			button--;
		}
		//chThdSleepMilliseconds(10);//musim si odichnut
	}
}
