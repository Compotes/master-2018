/*
    ChibiOS/RT - Copyright (C) 2006-2013 Giovanni Di Sirio

    Licensed under the Apache License, Version 2.0 (the "License");
    you may not use this file except in compliance with the License.
    You may obtain a copy of the License at

        http://www.apache.org/licenses/LICENSE-2.0

    Unless required by applicable law or agreed to in writing, software
    distributed under the License is distributed on an "AS IS" BASIS,
    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
    See the License for the specific language governing permissions and
    limitations under the License.
*/

/*
 Edited from ST_STM3220G_EVAL by Mario Hodas and Lukas Pariza
*/

#ifndef _BOARD_H_
#define _BOARD_H_

/*
 * Setup for board.
 */

/*
 * Board identifier.
 */
#define BOARD_NITE4_V1_0
#define BOARD_NAME  "Nite 4 V1.0"

#define STM32_VDD 330

/*
 * Board frequencies.
 * NOTE: The HSE crystal is not fitted by default on the board.
 */
#define STM32_LSECLK            0
#define STM32_HSECLK            0

/*
 * MCU type as defined in the ST header file stm32f2xx.h.
 */
#define STM32F205xx

/*
 * IO pins assignments.
 */

// ADC - PHOTOTRANSISTORS
#define GPIOC_PT1       5   //PC5
#define GPIOC_PT2       4   //PC4
#define GPIOA_PT3       6   //PA6
#define GPIOA_PT4       5   //PA5
#define GPIOA_PT5       1   //PA1
#define GPIOC_PT6       3   //PC3
#define GPIOC_PT7       1   //PC1
#define GPIOC_PT8       0   //PC0

// ADC - TSL
#define GPIOA_TSL1      7   //PA7
#define GPIOA_TSL2      4   //PA4
#define GPIOA_TSL3      2   //PA2
#define GPIOC_TSL4      2   //PC2

// GPIO
#define GPIOD_GP1       5   //PD5
#define GPIOD_GP2       3   //PD3
#define GPIOD_GP3       4   //PD4
#define GPIOD_GP4       2   //PD2

// LED
#define GPIOE_LED1      15  //PE15
#define GPIOE_LED2      14  //PE14

// IR LED (DISTANCE)
#define GPIOE_DIS_IR1   12  //PE12
#define GPIOE_DIS_IR2   7   //PE7
#define GPIOE_DIS_IR3   6   //PE6
#define GPIOE_DIS_IR4   4   //PE4

// IR LED (LINE)
#define GPIOE_LIN_IR1   11  //PE11
#define GPIOE_LIN_IR2   10  //PE10
#define GPIOE_LIN_IR3   9   //PE9
#define GPIOE_LIN_IR4   8   //PE8
#define GPIOE_LIN_IR5   5   //PE5
#define GPIOE_LIN_IR6   4   //PE4
#define GPIOE_LIN_IR7   2   //PE2

// BUTTON
#define GPIOB_BTN       8   //PB8

// I2C
#define GPIOB_I2C2_SCL  10  //PB10
#define GPIOB_I2C2_SDA  11  //PB11

// UART
#define GPIOC_USART3_TX  10 //PC10
#define GPIOC_USART3_RX  11 //PC11
#define BAUD_RATE        115200

// GPT & PWM
#define GPIOA_TIM1_CH1  8   //PA8
#define GPIOA_TIM1_CH2  9   //PA9
#define GPIOA_TIM2_CH1  15  //PA15
#define GPIOB_TIM2_CH2  3   //PB3
#define GPIOB_TIM3_CH1  4   //PB4
#define GPIOB_TIM3_CH2  5   //PB5
#define GPIOD_TIM4_CH1  12  //PD12
#define GPIOD_TIM4_CH2  13  //PD13

// SWD
#define GPIOA_SWDIO     13  //PA13
#define GPIOA_SWCLK     14  //PA14

// ADC inputs assignment
#define ADC_PT1         15  //ADC_IN15
#define ADC_PT2         14  //ADC_IN14
#define ADC_PT3         6   //ADC_IN6
#define ADC_PT4         5   //ADC_IN5
#define ADC_PT5         1   //ADC_IN1
#define ADC_PT6         13  //ADC_IN13
#define ADC_PT7         11  //ADC_IN11
#define ADC_PT8         10  //ADC_IN10
#define ADC_TSL1        7   //ADC_IN7
#define ADC_TSL2        4   //ADC_IN4
#define ADC_TSL3        2   //ADC_IN2
#define ADC_TSL4        12  //ADC_IN12



/*
 * I/O ports initial setup, this configuration is established soon after reset
 * in the initialization code.
 * Please refer to the STM32 Reference Manual for details.
 */
#define PIN_MODE_INPUT(n)           (0 << ((n) * 2))
#define PIN_MODE_OUTPUT(n)          (1 << ((n) * 2))
#define PIN_MODE_ALTERNATE(n)       (2 << ((n) * 2))
#define PIN_MODE_ANALOG(n)          (3 << ((n) * 2))
#define PIN_OTYPE_PUSHPULL(n)       (0 << (n))
#define PIN_OTYPE_OPENDRAIN(n)      (1 << (n))
#define PIN_OSPEED_2M(n)            (0 << ((n) * 2))
#define PIN_OSPEED_25M(n)           (1 << ((n) * 2))
#define PIN_OSPEED_50M(n)           (2 << ((n) * 2))
#define PIN_OSPEED_100M(n)          (3 << ((n) * 2))
#define PIN_PUDR_FLOATING(n)        (0 << ((n) * 2))
#define PIN_PUDR_PULLUP(n)          (1 << ((n) * 2))
#define PIN_PUDR_PULLDOWN(n)        (2 << ((n) * 2))
#define PIN_AFIO_AF(n, v)           ((v##U) << ((n % 8) * 4))

/*
 * Port A setup.
 * All input with pull-up except:
 * PA5  - SPI1_SCK      (alternate 5).
 * PA6  - SPI1_MISO     (alternate 5).
 * PA7  - SPI1_MOSI     (alternate 5).
 * PA9  - USART1_TX     (alternate 7).
 * PA10 - USART1_RX     (alternate 7).
 */
#define VAL_GPIOA_MODER		( \
				PIN_MODE_ALTERNATE(GPIOA_SWDIO) | \
				PIN_MODE_ALTERNATE(GPIOA_SWCLK) \
                )
#define VAL_GPIOA_OTYPER	0x00000000
#define VAL_GPIOA_OSPEEDR	0x55555555
#define VAL_GPIOA_PUPDR     0x55555555
#define VAL_GPIOA_ODR		0xFFFFF0FF
#define VAL_GPIOA_AFRL		0x55500000
#define VAL_GPIOA_AFRH		0x00000770

/*
 * Port B setup.
 * All input with pull-up except:
 * PB3  - SPI3_SCK      (alternate 6).
 * PB4  - SPI3_MISO     (alternate 6).
 * PB5  - SPI3_MOSI     (alternate 6).
 * PB8  - I2C1_SCL      (alternate 4).
 * PB9  - I2C1_SDA      (alternate 4).
 * PB10 - USART3_TX     (alternate 7).
 * PB11 - USART3_RX     (alternate 7).
 * PB13 - SPI2_SCK      (alternate 5).
 * PB14 - SPI2_MISO     (alternate 5).
 * PB15 - SPI2_MOSI     (alternate 5).
 */
#define VAL_GPIOB_MODER     ( \
                PIN_MODE_ALTERNATE(GPIOB_I2C2_SCL) | \
                PIN_MODE_ALTERNATE(GPIOB_I2C2_SDA) \
                )
#define VAL_GPIOB_OTYPER	( \
                PIN_OTYPE_OPENDRAIN(GPIOB_I2C2_SCL) | \
                PIN_OTYPE_OPENDRAIN(GPIOB_I2C2_SDA) \
                ) /* 0x00000000 */
#define VAL_GPIOB_OSPEEDR	0x55555555
#define VAL_GPIOB_PUPDR     ( \
                PIN_PUDR_FLOATING(GPIOB_I2C2_SCL) | \
                PIN_PUDR_FLOATING(GPIOB_I2C2_SDA) \
                ) /* 0x55555555 */
#define VAL_GPIOB_ODR		0xFFFF0FFF
#define VAL_GPIOB_AFRL		0x00666000
/* 0x00000000 */
#define VAL_GPIOB_AFRH      ( \
                PIN_AFIO_AF(GPIOB_I2C2_SCL, 4) | \
                PIN_AFIO_AF(GPIOB_I2C2_SDA, 4))

/*
 * Port C setup.
 * All input with pull-up except:
 * PC6  - USART6_TX     (alternate 8).
 * PC7  - USART6_RX     (alternate 8).
 * PC14 - OSC32_INT     (input floating).
 * PC15 - OSC32_OUT     (input floating).
 */
#define VAL_GPIOC_MODER		0x0//( 
//				PIN_MODE_OUTPUT(4) | 
//				PIN_MODE_OUTPUT(5) | 
//				PIN_MODE_ALTERNATE(6) | 
//				PIN_MODE_ALTERNATE(7) | 
///				PIN_MODE_OUTPUT(8) | 
//				PIN_MODE_OUTPUT(9) | 
//				PIN_MODE_OUTPUT(10) | 
//				PIN_MODE_OUTPUT(11) | 
//				PIN_MODE_OUTPUT(12) 
//				)

#define VAL_GPIOC_OTYPER	0x00000000
#define VAL_GPIOC_OSPEEDR	0x55555555 // 25 MHz
#define VAL_GPIOC_PUPDR         (( \
				PIN_PUDR_PULLUP(0)    | \
				PIN_PUDR_PULLUP(1)    | \
				PIN_PUDR_PULLUP(2)    | \
				PIN_PUDR_PULLUP(3)    | \
				PIN_PUDR_PULLUP(13)    | \
                                PIN_PUDR_FLOATING(14)  | \
                                PIN_PUDR_FLOATING(15) \
				))
#define VAL_GPIOC_ODR		0xFFFF08FF
#define VAL_GPIOC_AFRL		0x88000000
#define VAL_GPIOC_AFRH		0x00000000
				
/*
 * Port D setup.
 * All input with pull-up.
 * PD8  - USART3_TX     (alternate 7).
 * PD9  - USART3_RX     (alternate 7).
 * PD12 - TIM4_CH1      (alternate 2).
 * PC13 - TIM4_CH2      (alternate 2).
 */

#ifndef _DEBUG
#define VAL_GPIOD_MODER		0x0//( 
//				PIN_MODE_OUTPUT(1) | 
//				PIN_MODE_OUTPUT(2) | 
//				PIN_MODE_OUTPUT(4) | 
//				PIN_MODE_OUTPUT(5) | 
//				PIN_MODE_OUTPUT(6) | 
//				PIN_MODE_OUTPUT(7) | 
//				PIN_MODE_ALTERNATE(8) | 
//				PIN_MODE_ALTERNATE(9) | 
//				PIN_MODE_ALTERNATE(12) | 
//				PIN_MODE_ALTERNATE(13) 
//				)
#else
#define VAL_GPIOD_MODER		0x0//( 
//				PIN_MODE_OUTPUT(1) | 
//				PIN_MODE_OUTPUT(2) | 
//				PIN_MODE_OUTPUT(4) | 
//				PIN_MODE_OUTPUT(5) | 
//				PIN_MODE_OUTPUT(6) | 
//				PIN_MODE_OUTPUT(7) | 
//				PIN_MODE_ALTERNATE(12) | 
//				PIN_MODE_ALTERNATE(13) 
//				)
#endif
#define VAL_GPIOD_OTYPER	0x00000000
#define VAL_GPIOD_OSPEEDR	0x55555555 // 25 MHz
#define VAL_GPIOD_PUPDR     0x55555555
#define VAL_GPIOD_ODR		0xFFFFFF00
#define VAL_GPIOD_AFRL		0x00000000
#define VAL_GPIOD_AFRH		0x00220077

/*
 * Port E setup.
 * All input with pull-up.
 */
#define VAL_GPIOE_MODER		( \
				PIN_MODE_OUTPUT(14) | \
				PIN_MODE_OUTPUT(15) \
				)
#define VAL_GPIOE_OTYPER	0x00000000
#define VAL_GPIOE_OSPEEDR	0x55555555
#define VAL_GPIOE_PUPDR     0x55555555 // all pull-up
#define VAL_GPIOE_ODR		0xFFFF0200
#define VAL_GPIOE_AFRL		0x00000000
#define VAL_GPIOE_AFRH		0x00000000

/*
 * Port F setup.
 * All input with pull-up.
 */
#define VAL_GPIOF_MODER		0x00000000
#define VAL_GPIOF_OTYPER	0x00000000
#define VAL_GPIOF_OSPEEDR	0xFFFFFFFF
#define VAL_GPIOF_PUPDR		0xFFFFFFFF
#define VAL_GPIOF_ODR		0xFFFFFFFF
#define VAL_GPIOF_AFRL		0x00000000
#define VAL_GPIOF_AFRH		0x00000000

/*
 * Port G setup.
 * All input with pull-up.
 */
#define VAL_GPIOG_MODER		0x00000000
#define VAL_GPIOG_OTYPER	0x00000000
#define VAL_GPIOG_OSPEEDR	0xFFFFFFFF
#define VAL_GPIOG_PUPDR		0xFFFFFFFF
#define VAL_GPIOG_ODR		0xFFFFFFFF
#define VAL_GPIOG_AFRL		0x00000000
#define VAL_GPIOG_AFRH		0x00000000

/*
 * Port H setup.
 * All input with pull-up.
 */
#define VAL_GPIOH_MODER		0x00000000
#define VAL_GPIOH_OTYPER	0x00000000
#define VAL_GPIOH_OSPEEDR	0xFFFFFFFF
#define VAL_GPIOH_PUPDR		0xFFFFFFFF
#define VAL_GPIOH_ODR		0xFFFFFFFF
#define VAL_GPIOH_AFRL		0x00000000
#define VAL_GPIOH_AFRH		0x00000000

/*
 * Port I setup.
 * All input with pull-up.
 */
#define VAL_GPIOI_MODER		0x00000000
#define VAL_GPIOI_OTYPER	0x00000000
#define VAL_GPIOI_OSPEEDR	0xFFFFFFFF
#define VAL_GPIOI_PUPDR		0xFFFFFFFF
#define VAL_GPIOI_ODR		0xFFFFFFFF
#define VAL_GPIOI_AFRL		0x00000000
#define VAL_GPIOI_AFRH		0x00000000

#if !defined(_FROM_ASM_)
#ifdef __cplusplus
extern "C" {
#endif
  void boardInit(void);
#ifdef __cplusplus
}
#endif
#endif /* _FROM_ASM_ */

#endif /* _BOARD_H_ */
