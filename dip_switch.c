#include "dip_switch.h"

uint8_t get_dip_switch(void){
	uint8_t ret;
	ret = palReadPad(GPIOD, 9);
	ret = (ret << 1) | palReadPad(GPIOD, 8);
	ret = (ret << 1) | palReadPad(GPIOB, 15);
	ret = (ret << 1) | palReadPad(GPIOB, 14);
	ret = (ret << 1) | palReadPad(GPIOB, 13);
	ret = (ret << 1) | palReadPad(GPIOB, 12);
	return ret;
}
