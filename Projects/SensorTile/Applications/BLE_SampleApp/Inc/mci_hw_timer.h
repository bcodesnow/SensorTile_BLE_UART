#ifndef ___hw_timer_
#define ___hw_timer_

#include "stm32l4xx_hal.h"

/* Init the hw timers, 512ms, 100hz, 1khz, setup priorities, yes one 1ms timer could also doit, but lets try this way */
// todo: stop, reset routines

extern void init_tim2(void);
extern void init_tim3(void);
extern void init_tim4(void);
extern void init_tim5(void);

extern inline void stop_timer_2(void);
extern inline void start_timer_2(void);
extern inline void stop_timer_4(void);
extern inline void start_timer_4(void);
extern inline void stop_timer_5(void);
extern inline void start_timer_5(void);

/* The Preemption Priority allows an ISR to be preempted (interrupted) by another interrupt of higher priority.	*/ 
/* When the higher-priority interrupt is completed, the lower-priority interrupt continues from where it left off. */
/* Subpriority, on the other hand, has nothing to do with preemption. Say that you have two interrupts of the same */
/*	which are both pending. The interrupt handler will choose which one to service first, based on their subpriority. */
/* Once the first one is completed, the second one will begin. It may seem unlikely that two interrupts can happen at the */
/* same time. One common situation where this could happen is if you have an interrupt service routine that takes a long */
/* time. During this routine, multiple other interrupt triggers may have taken place. If so, the subpriority will determine */
/* which one is handled next. */
/* Finally, keep in mind that as the actual priority increases, the priority value decreases. That is, Priority=1 will be serviced before Priority=2. */


#endif /*___hw_timer_*/
