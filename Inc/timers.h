#ifndef SYS_TICK_H_
#define SYS_TICK_H_

#include <stm32f4xx.h>


void SysTickDelayMs(int delay);

void NBdelay_ms(uint32_t ms);

extern volatile uint32_t msTicks;

extern unsigned int countWakeUp;
extern int flagmsTicks;  // extern


#endif /* SYS_TICK_H_ */


void SysTick_Init(void);
// void SysTick_Handler(void); // defined in CMSIS
