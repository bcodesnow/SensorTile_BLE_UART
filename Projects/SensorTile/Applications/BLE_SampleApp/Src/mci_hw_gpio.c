#include "mci_hw_gpio.h"

void init_gpio_pc0_as_output(void)
{
	// Configure as Output PCO
	// GPIOC is already enabled, but .. better twice than never
	RCC->AHB2ENR |= RCC_AHB2ENR_GPIOCEN;
	// set pc0 as output
	GPIOC->MODER &= ~GPIO_MODER_MODE0; /* Delete Flags in MODER0 */ /* Input mode */
	GPIOC->MODER |= GPIO_MODER_MODE0_0; /* General purpose output mode */
	// delete the flags to get push pull on the output
	GPIOC->OTYPER &= ~GPIO_OTYPER_OT_0;
	// make sure pullup pulldown is disabled
  GPIOC->PUPDR &= ~GPIO_PUPDR_PUPDR0;
	// make it very highspeed
	GPIOC->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR0;
}

void init_gpio_pc1_as_output(void)
{
	// Configure as Output PC1
	// GPIOC is already enabled, but .. better twice than never
	RCC->AHB2ENR |= RCC_AHB2ENR_GPIOCEN;
	// set pc1 as output
	GPIOC->MODER &= ~GPIO_MODER_MODE1; /* Delete Flags in MODER1 */ /* Input mode */
	GPIOC->MODER |= GPIO_MODER_MODE1_0; /* General purpose output mode */
	// set the flag to get open drain on the output
	GPIOC->OTYPER |= GPIO_OTYPER_OT_1;
	// make sure pullup is enabled
  GPIOC->PUPDR &= ~GPIO_PUPDR_PUPDR1;
	GPIOC->PUPDR |= GPIO_PUPDR_PUPDR1_0;
	// make it very highspeed
	GPIOC->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR1;
}

void init_gpio_leds(void) 
{
	// ORANGE GPIOG14
	// GPIOG is also already enabled, but .. better twice than never
	RCC->AHB2ENR |= RCC_AHB2ENR_GPIOGEN;
	// set g12
	GPIOG->MODER &= ~(0x3 << (12*2)); /* Delete Flags in MODER12 */
	GPIOG->MODER |=  GPIO_MODER_MODE12_0;
	// delete the flags to get push pull on the output
	GPIOG->OTYPER &= ~GPIO_OTYPER_OT_12;
	// make sure pullup pulldown is disabled
  GPIOG->PUPDR &= ~(GPIO_PUPDR_PUPDR12);

	//SWD GPIOA14
	// GPIOA is also already enabled, but .. better twice than never
	RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN;
	// set g14
	GPIOA->MODER &= ~(0x3 << (14*2)); /* Delete Flags in MODER14 */
	GPIOA->MODER |=  GPIO_MODER_MODE14_0;
	// delete the flags to get push pull on the output
	GPIOA->OTYPER &= ~GPIO_OTYPER_OT_14;
	// make sure pullup pulldown is disabled
  GPIOA->PUPDR &= ~(GPIO_PUPDR_PUPDR14);
}

/***********************************
*	inits pc0 as input with pullup	 *
* optionally with interrupt on     *
* falling edge										 *
***********************************/
void init_gpio_pc0_as_input(uint8_t withInterrupt)
{
	// GPIOC is already enabled, but .. better twice than never
	RCC->AHB2ENR |= RCC_AHB2ENR_GPIOCEN;
	// set pc0 as input
	GPIOC->MODER &= ~GPIO_MODER_MODE0; /* Delete Flags in MODER0 & MODER1, this also sets input mode 0 */
	GPIOC->PUPDR &= ~GPIO_PUPDR_PUPDR0; // clear pullup / pulldown
	GPIOC->PUPDR |= GPIO_PUPDR_PUPDR0_0; // turn on pullup 
	
	if ( withInterrupt )
	{
		RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;
		SYSCFG->EXTICR[0] |= SYSCFG_EXTICR1_EXTI0_PC; //Connect the portC pin0 to external interrupt line0
		EXTI->IMR1 |= (1<<0); //mask register to trigger interrupt  0 = marked, 1 = not masked (enabled)
		EXTI->FTSR1 |= (1<<0); //selecting falling edge
		__enable_irq(); // just to make sure
		NVIC_SetPriority(EXTI0_IRQn,1); 
		NVIC_ClearPendingIRQ(EXTI0_IRQn);
		NVIC_EnableIRQ(EXTI0_IRQn);
	}
}

void init_gpio_pc1_as_input(uint8_t withInterrupt)
{
	// Configure as Input PC1
	// GPIOC is already enabled, but .. better twice than never
	RCC->AHB2ENR |= RCC_AHB2ENR_GPIOCEN;
	// set pc1 as input
	GPIOC->MODER &= ~GPIO_MODER_MODE1; /* Delete Flags in MODER1 */ /* Input mode */
	// make sure pullup enabled
  GPIOC->PUPDR &= ~GPIO_PUPDR_PUPDR1;
	GPIOC->PUPDR |= GPIO_PUPDR_PUPDR1_0;
	
	if ( withInterrupt )
	{
		RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;
		SYSCFG->EXTICR[0] |= SYSCFG_EXTICR1_EXTI1_PC; //Connect the portC pin1 to external interrupt line1
		EXTI->IMR1 |= (1<<1); //mask register to trigger interrupt  0 = marked, 1 = not masked (enabled)
		EXTI->FTSR1 |= (1<<1); //selecting falling edge
		__enable_irq(); // just to make sure
		NVIC_SetPriority(EXTI1_IRQn,1);
		NVIC_ClearPendingIRQ(EXTI1_IRQn);
		NVIC_EnableIRQ(EXTI1_IRQn);
	}
	
	// Assumption: PC1&PB8&PB9 are all connected to the same GPIO on the STEVAL-STLCS01V1 so we have to set two of them high impedant and use only one of them..
	// GPIOB is also already enabled, but .. better twice than never
	RCC->AHB2ENR |= RCC_AHB2ENR_GPIOBEN;
	// set pb8 as output and pb9 as input
	GPIOB->MODER &= ~GPIO_MODER_MODE8; /* Delete Flags in MODER8 */ /* Input mode */
	GPIOB->MODER |= GPIO_MODER_MODE8_0; /* General purpose output mode */
	GPIOB->MODER &= ~GPIO_MODER_MODE9; /* Delete Flags in MODER9 */ /* Input mode */
	// set the flag to get open drain on the output
	GPIOB->OTYPER |= GPIO_OTYPER_OT_8;
	// make sure pullup is enabled
  GPIOB->PUPDR &= ~(GPIO_PUPDR_PUPDR8 | GPIO_PUPDR_PUPDR9);
  GPIOB->PUPDR |= ~(GPIO_PUPDR_PUPDR8_0 | GPIO_PUPDR_PUPDR9_0);
}
