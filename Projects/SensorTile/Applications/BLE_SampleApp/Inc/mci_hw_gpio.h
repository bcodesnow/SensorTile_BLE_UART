#ifndef ___hw_gpio_
#define ___hw_gpio_

#include "stm32l4xx_hal.h"

#define GPIO_SET_PC0 						GPIOC->BSRR = GPIO_PIN_0		
#define GPIO_CLEAR_PCO  					GPIOC->BRR = GPIO_PIN_0	
#define GPIO_TOGGLE_PC0			    GPIOC->ODR ^= GPIO_PIN_0


#define GPIO_SET_PC1_PB8_PB9			GPIOC->BSRR = GPIO_PIN_1; GPIOB->BSRR = GPIO_PIN_8; GPIOB->BSRR = GPIO_PIN_9		
#define GPIO_CLEAR_PC1_PB8_PB9		GPIOC->BRR = GPIO_PIN_1; GPIOB->BRR = GPIO_PIN_8; GPIOB->BRR = GPIO_PIN_9

#define GPIO_SET_PC1							GPIOC->BSRR = GPIO_PIN_1	
#define GPIO_CLEAR_PC1						GPIOC->BRR = GPIO_PIN_1
#define GPIO_TOGGLE_PC1			    GPIOC->ODR ^= GPIO_PIN_1

#define GPIO_SET_PB8							GPIOC->BSRR = GPIO_PIN_8	
#define GPIO_CLEAR_PB8						GPIOC->BRR = GPIO_PIN_8

#define GPIO_SET_ORANGE_LED 			GPIOG->BSRR = GPIO_PIN_12
#define GPIO_CLEAR_ORANGE_LED 		GPIOG->BRR = GPIO_PIN_12
#define GPIO_TOGGLE_ORANGE_LED 		GPIOG->ODR ^= GPIO_PIN_12

#define GPIO_SET_SWD_LED					GPIOA->BRR = GPIO_PIN_14
#define GPIO_CLEAR_SWD_LED 				GPIOA->BSRR = GPIO_PIN_14
#define GPIO_TOGGLE_SWD_LED 			GPIOA->ODR ^= GPIO_PIN_14

#define READ_BUTTON_STATE (GPIOC->IDR & 0x01) // READ BUTTON STATE

extern void init_gpio_pc0_as_output(void);
extern void init_gpio_pc1_as_output(void);
extern void init_gpio_pc0_as_input(uint8_t withInterrupt);
extern void init_gpio_pc1_as_input(uint8_t withInterrupt);
extern void init_gpio_leds(void);

#endif


