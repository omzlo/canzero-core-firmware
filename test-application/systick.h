#ifndef _SYSTICK_H_
#define _SYSTICK_H_
    
void systick_init(void);

void systick_delay_ms(uint32_t delay_ms);

uint32_t systick_now_ms(void);

void active_delay_us(uint32_t delay_us);

#endif

