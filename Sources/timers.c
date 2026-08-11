#include "stm32f446xx.h"
#include "stm32f4xx.h"
#include "timers.h"


unsigned int countWakeUp = 0;  // extern
int flagmsTicks = 0;  // extern


// Records the starting tick count and waits until the required nb of ms has passed. It’s non-blocking, the CPU can still handle interrupts while waiting
void NBdelay_ms(uint32_t ms)
{
    uint32_t start = msTicks;
    while ((msTicks - start) < ms) {}
}

