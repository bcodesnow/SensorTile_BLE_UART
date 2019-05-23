#include "mci_hw_timer.h"


// 1ms
void init_tim2(void)
{
	RCC->APB1ENR1 |= RCC_APB1ENR1_TIM2EN;    /* enable clock for TIM2    */

  TIM2->PSC   = 4;                         /* set prescaler   				 */
  TIM2->ARR   = 16000;                     /* set auto-reload  				 */
  TIM2->RCR   =  0;                        /* set repetition counter   */

  TIM2->DIER = TIM_DIER_UIE;               /* Update Interrupt enable  */
  NVIC_EnableIRQ(TIM2_IRQn);               /* TIM4   Interrupt enable  */
  TIM2->CR1  |= TIM_CR1_CEN;               /* timer enable             */
}

// 10ms
void init_tim5(void)
{
	RCC->APB1ENR1 |= RCC_APB1ENR1_TIM5EN;    /* enable clock for TIM5    */

  TIM5->PSC   = 256;  
  TIM5->ARR   = 3125;                      /* set auto-reload 				 */
  TIM5->RCR   =  0;                        /* set repetition counter   */

  TIM5->DIER = TIM_DIER_UIE;               /* Update Interrupt enable  */
  NVIC_EnableIRQ(TIM5_IRQn);               /* TIM4   Interrupt enable  */
  TIM5->CR1  |= TIM_CR1_CEN;               /* timer enable             */
}

// 1ms
void init_tim4(void)
{
	RCC->APB1ENR1 |= RCC_APB1ENR1_TIM4EN;    /* enable clock for TIM4    */

  TIM4->PSC   = 4;                         /* set prescaler   				 */
  TIM4->ARR   = 16000;                     /* set auto-reload  				 */
  TIM4->RCR   =  0;                        /* set repetition counter   */

  TIM4->DIER = TIM_DIER_UIE;               /* Update Interrupt enable  */
  NVIC_EnableIRQ(TIM4_IRQn);               /* TIM4   Interrupt enable  */
  TIM4->CR1  |= TIM_CR1_CEN;               /* timer enable             */
}

// 100ms
void init_tim3(void)
{
	RCC->APB1ENR1 |= RCC_APB1ENR1_TIM3EN;    /* enable clock for TIM3    */

  TIM3->PSC   = 1024;                      /* set prescaler  					 */
  TIM3->ARR   = 6250;                      /* set auto-reload	- 512ms=32000 | 100ms=6250 */
  TIM3->RCR   =  0;                        /* set repetition counter   */

  TIM3->DIER = TIM_DIER_UIE;               /* Update Interrupt enable  */
  NVIC_EnableIRQ(TIM3_IRQn);               /* TIM4   Interrupt enable  */
  TIM3->CR1  |= TIM_CR1_CEN;               /* timer enable             */
}

inline void stop_timer_2(void) 
{
			TIM2->CR1 &= ~TIM_CR1_CEN;
			NVIC_DisableIRQ(TIM2_IRQn);
}

inline void start_timer_2(void)
{
			TIM2->SR = ~TIM_FLAG_UPDATE;
			TIM2->CNT = 0;
			TIM2->CR1 |= TIM_CR1_CEN;
			NVIC_EnableIRQ(TIM2_IRQn);
}

inline void stop_timer_4(void) 
{			
			TIM4->CR1 &= ~TIM_CR1_CEN;
			NVIC_DisableIRQ(TIM4_IRQn);
}

inline void start_timer_4(void)
{			
			TIM4->SR = ~TIM_FLAG_UPDATE;
			TIM4->CNT = 0;
			TIM4->CR1 |= TIM_CR1_CEN;
			NVIC_EnableIRQ(TIM4_IRQn);
}

inline void stop_timer_5(void) 
{
			TIM5->CR1 &= ~TIM_CR1_CEN;
			NVIC_DisableIRQ(TIM5_IRQn);
}

inline void start_timer_5(void)
{
			TIM5->SR = ~TIM_FLAG_UPDATE;
			TIM5->CNT = 0;
			TIM5->CR1 |= TIM_CR1_CEN;
			NVIC_EnableIRQ(TIM5_IRQn);
}
