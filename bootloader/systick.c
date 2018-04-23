#include <samd21.h>
#include "systick.h"

volatile uint32_t msTicks = 0;
volatile int32_t msDelay = 0;

void systick_init()
{
    SystemCoreClockUpdate();
    msTicks = 0;
    if (SysTick_Config(SystemCoreClock / 1000)) for(;;);
}

void systick_delay_ms(uint32_t delay_ms)
{
    msDelay = delay_ms;
    while(msDelay != 0);
}

uint32_t systick_now_ms()
{
    return msTicks;
}
    
void SysTick_Handler(void)
{   
    if (msDelay != 0x00)
        msDelay --;
    msTicks++;
}
    
__attribute__ ( ( section ( ".ramfunc" ) ) ) void active_delay_us(uint32_t delay_us)
{       
    __asm(
            "mydelay0: \n"
            " mov  r1, #15    \n"  // 1 cycle
	        "mydelay1: \n"
	        " sub  r1, r1, #1 \n"  // 1 cycle
	        " bne  mydelay1    \n" // 2 if taken, 1 otherwise
	        " sub  r0, r0, #1 \n"  // 1 cycle
	        " bne  mydelay0    \n"  // 2 if taken, 1 otherwise
	);
}

