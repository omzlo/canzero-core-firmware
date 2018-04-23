#include <samd21.h>
#include "serial.h"
#include "system_samd21.h"

/* 
 * BETA:
 * SERIAL (SERCOM 0) on PA10 and PA11 
 *
 * CANZERO_MKR:
 * SERIAL (SERCOM 5 ALT) on PB22 and PB23
 */

int serial_open(uint32_t baud)
{
    /* In arithmetic mode:
     *                                  BaudRate
     * BAUD = 65536 * (1 - SampleRate * ---------- )
     *                                  fCoreClock
     * 
     * 65536 * ( 1 - 16 * 115200/48000000) = 63019.41
     */

    uint32_t baud_val;
    
    switch (baud) {
        case 115200:
            baud_val = 63019;
            break;
        case 19200:
            baud_val = 65116;
        case 38400:
            baud_val = 64697;
            break;
        case 9600:
            baud_val = 65326;
            break;
        default:
            return -1;
    }

#ifdef CANZERO_MKR
    // PB23 is D13, aka RX, aka SERCOM5.PAD3
    PORT->Group[1].PINCFG[(PIN_PB23&31)].bit.PMUXEN = 1;
    // PMUXO = PMUX odd 4bits 4..7 
    PORT->Group[1].PMUX[(PIN_PB23&31)/2].bit.PMUXO = PORT_PMUX_PMUXE_D;

    // PB22 is D14, aka TX, aka SERCOM5.PAD2
    PORT->Group[1].PINCFG[(PIN_PB22&31)].bit.PMUXEN = 1;
    // PMUXE = MUX even 4bits 0..3
    PORT->Group[1].PMUX[(PIN_PB22&31)/2].bit.PMUXE = PORT_PMUX_PMUXE_D;

    /* Enable clock for BOOT_USART_MODULE */
    PM->APBCMASK.reg |= PM_APBCMASK_SERCOM5;

    /* Set GCLK_GEN0 as source for GCLK_ID_SERCOMx_CORE */
    GCLK->CLKCTRL.reg = GCLK_CLKCTRL_ID( GCLK_CLKCTRL_ID_SERCOM5_CORE_Val ) | // Generic Clock 0 (SERCOMx)
        GCLK_CLKCTRL_GEN_GCLK0 | // Generic Clock Generator 0 is source
        GCLK_CLKCTRL_CLKEN ;

    /* Wait until ready ?needed? */
    while ( GCLK->STATUS.reg & GCLK_STATUS_SYNCBUSY )
    {
        /* Wait for synchronization */
    }

    /* Wait for synchronization */
    while(SERCOM5->USART.SYNCBUSY.bit.ENABLE);

    /* Disable the SERCOM UART module */
    SERCOM5->USART.CTRLA.bit.ENABLE = 0;
    /* Wait for synchronization */
    while(SERCOM5->USART.SYNCBUSY.bit.SWRST);

    /* Perform a software reset */
    SERCOM5->USART.CTRLA.bit.SWRST = 1;
    /* Wait for synchronization */
    while(SERCOM5->USART.CTRLA.bit.SWRST);
    /* Wait for synchronization */
    while(SERCOM5->USART.SYNCBUSY.bit.SWRST || SERCOM5->USART.SYNCBUSY.bit.ENABLE);

    /* Update the UART pad settings, mode and data order settings 
     * SERCOM_USART_CTRLA_MODE(1) -> USART with internal clock.
     * pad_conf -> TXPO and RXPO selects pads
     * DORD -> Data order 0:MSB first, 1:LSB first
     */
    SERCOM5->USART.CTRLA.reg = SERCOM_USART_CTRLA_RXPO(3) 
        | SERCOM_USART_CTRLA_TXPO(1) 
        | SERCOM_USART_CTRLA_MODE(1) 
        | SERCOM_USART_CTRLA_DORD;
    /* Wait for synchronization */
    while(SERCOM5->USART.SYNCBUSY.bit.CTRLB);

    /* Enable transmit and receive and set data size to 8 bits */
    SERCOM5->USART.CTRLB.reg = SERCOM_USART_CTRLB_RXEN | SERCOM_USART_CTRLB_TXEN | SERCOM_USART_CTRLB_CHSIZE(0);
    /* Load the baud value */
    SERCOM5->USART.BAUD.reg = baud_val;
    /* Wait for synchronization */
    while(SERCOM5->USART.SYNCBUSY.bit.ENABLE);
    /* Enable SERCOM UART */
    SERCOM5->USART.CTRLA.bit.ENABLE = 1;

#else
    // PA11 is D0, aka RX, aka SERCOM0.PAD3
    PORT->Group[0].PINCFG[PIN_PA11].bit.PMUXEN = 1;
    // PMUXO = PMUX odd 4bits 4..7 
    PORT->Group[0].PMUX[PIN_PA11/2].bit.PMUXO = PORT_PMUX_PMUXE_C;

    // PA10 is D1, aka TX, aka SERCOM0.PAD2
    PORT->Group[0].PINCFG[PIN_PA10].bit.PMUXEN = 1;
    // PMUXE = MUX even 4bits 0..3
    PORT->Group[0].PMUX[PIN_PA10/2].bit.PMUXE = PORT_PMUX_PMUXE_C;

    /* Enable clock for BOOT_USART_MODULE */
    PM->APBCMASK.reg |= PM_APBCMASK_SERCOM0;

    /* Set GCLK_GEN0 as source for GCLK_ID_SERCOMx_CORE */
    GCLK->CLKCTRL.reg = GCLK_CLKCTRL_ID( GCLK_CLKCTRL_ID_SERCOM0_CORE_Val ) | // Generic Clock 0 (SERCOMx)
        GCLK_CLKCTRL_GEN_GCLK0 | // Generic Clock Generator 0 is source
        GCLK_CLKCTRL_CLKEN ;

    /* Wait until ready ?needed? */
    while ( GCLK->STATUS.reg & GCLK_STATUS_SYNCBUSY )
    {
        /* Wait for synchronization */
    }

    /* Wait for synchronization */
    while(SERCOM0->USART.SYNCBUSY.bit.ENABLE);

    /* Disable the SERCOM UART module */
    SERCOM0->USART.CTRLA.bit.ENABLE = 0;
    /* Wait for synchronization */
    while(SERCOM0->USART.SYNCBUSY.bit.SWRST);

    /* Perform a software reset */
    SERCOM0->USART.CTRLA.bit.SWRST = 1;
    /* Wait for synchronization */
    while(SERCOM0->USART.CTRLA.bit.SWRST);
    /* Wait for synchronization */
    while(SERCOM0->USART.SYNCBUSY.bit.SWRST || SERCOM0->USART.SYNCBUSY.bit.ENABLE);

    /* Update the UART pad settings, mode and data order settings 
     * SERCOM_USART_CTRLA_MODE(1) -> USART with internal clock.
     * pad_conf -> TXPO and RXPO selects pads
     * DORD -> Data order 0:MSB first, 1:LSB first
     */
    SERCOM0->USART.CTRLA.reg = SERCOM_USART_CTRLA_RXPO(3) 
        | SERCOM_USART_CTRLA_TXPO(1) 
        | SERCOM_USART_CTRLA_MODE(1) 
        | SERCOM_USART_CTRLA_DORD;
    /* Wait for synchronization */
    while(SERCOM0->USART.SYNCBUSY.bit.CTRLB);

    /* Enable transmit and receive and set data size to 8 bits */
    SERCOM0->USART.CTRLB.reg = SERCOM_USART_CTRLB_RXEN | SERCOM_USART_CTRLB_TXEN | SERCOM_USART_CTRLB_CHSIZE(0);
    /* Load the baud value */
    SERCOM0->USART.BAUD.reg = baud_val;
    /* Wait for synchronization */
    while(SERCOM0->USART.SYNCBUSY.bit.ENABLE);
    /* Enable SERCOM UART */
    SERCOM0->USART.CTRLA.bit.ENABLE = 1;
#endif
    return 0;
}

int serial_close()
{
#ifdef CANZERO_MKR
    /* Wait for synchronization */
	while(SERCOM5->USART.SYNCBUSY.bit.ENABLE);
	/* Disable SERCOM UART */
	SERCOM5->USART.CTRLA.bit.ENABLE = 0;
#else
    /* Wait for synchronization */
	while(SERCOM0->USART.SYNCBUSY.bit.ENABLE);
	/* Disable SERCOM UART */
	SERCOM0->USART.CTRLA.bit.ENABLE = 0;
#endif
    return 0;
}

int serial_putc(int c)
{
#ifdef CANZERO_MKR
    while (SERCOM5->USART.INTFLAG.bit.DRE==0) {}
    SERCOM5->USART.DATA.reg = c;
#else
    while (SERCOM0->USART.INTFLAG.bit.DRE==0) {}
    SERCOM0->USART.DATA.reg = c;
#endif
    return c;
}

int serial_getc()
{
#ifdef CANZERO_MKR
    while(!SERCOM5->USART.INTFLAG.bit.RXC);
    /* We should check for errors here in 
     * SERCOM0->USART.STATUS.bit.PERR || SERCOM0->USART.STATUS.bit.FERR || SERCOM0->USART.STATUS.bit.BUFOVF
     */
    return (int)(SERCOM5->USART.DATA.reg&0xFF);
#else
    while(!SERCOM0->USART.INTFLAG.bit.RXC);
    /* We should check for errors here in 
     * SERCOM0->USART.STATUS.bit.PERR || SERCOM0->USART.STATUS.bit.FERR || SERCOM0->USART.STATUS.bit.BUFOVF
     */
    return (int)(SERCOM0->USART.DATA.reg&0xFF);
#endif
}

int serial_available(void)
{
#ifdef CANZERO_MKR
    return SERCOM5->USART.INTFLAG.bit.RXC;
#else
    return SERCOM0->USART.INTFLAG.bit.RXC;
#endif
}

#ifndef NULL
#define NULL ((void *)0)
#endif

const char digits[]="0123456789abcdef";

static void _process_uint(unsigned u)
{
    unsigned d,c;

    if (u==0) {
        serial_putc('0');
        return;
    }

    d=1000000000; // 32 bits means max uint = 4,294,967,295
    while (d>u) d/=10;
    while (d)
    {
        c = u/d;
        serial_putc(digits[c]);
        u %= d;
        d /= 10;
    }
    return;
}

static void _process_string(const char *s)
{
    while (*s) serial_putc(*s++);
}

static void _process_hex(unsigned u)
{
    unsigned su,c;

    if (u==0) 
    {
        serial_putc('0');
        serial_putc('0');
        return;
    }         
    su = 24;
    while ((u>>su)==0) su-=8;
    su+=4;
    for (;;) {
        c = (u>>su)&0xF;
        serial_putc(digits[c]);
        if (su==0) break;
        su-=4;
    }
}


int serial_vprintf(const char *format, va_list ap)
{
    int i;
    unsigned u;
    const char *s;

    while (*format) {
        if (*format=='%')
        {
            format++;
            switch (*format) {
                case '%':
                    format++;
                    serial_putc(*format++);
                    break;
                case 'i':
                    format++;
                    i = va_arg(ap, int);
                    if (i<0)
                    {
                        serial_putc('-');
                        i = -i;
                    }
                    _process_uint((unsigned)i);
                    break;
                case 'u':
                    format++;
                    u = va_arg(ap, unsigned);
                    _process_uint(u);
                    break;
                case 'x':
                case 'X':
                    format++;
                    u = va_arg(ap, unsigned);
                    _process_hex(u);
                    break;
                case 's':
                    format++;
                    s = va_arg(ap, const char *);
                    _process_string(s);    
                    break;
                case 'c':
                    format++;
                    i = va_arg(ap, int);
                    serial_putc(i);
                    break;
                case 'p':
                    format++;
                    _process_string("0x");
                    u = va_arg(ap, unsigned);
                    _process_hex(u);
                    break;
                default:
                    format++;
                    _process_string("<?format?>");
                    return -1;
            }
        }
        else
        {
            if (*format=='\n')
                serial_putc('\r');
            serial_putc(*format++);
        }
    }

    return 0;
}
    
            
int serial_printf(const char *format, ...)
{
    va_list ap;
    int retval;

    va_start(ap, format);
    retval = serial_vprintf(format, ap);
    va_end(ap);
    return retval;
}

unsigned serial_debug_enable = 0;

int serial_debug_printf(const char *format, ...)
{
    if (serial_debug_enable)
    {
        va_list ap;
        int retval;

        va_start(ap, format);
        retval = serial_vprintf(format, ap);
        va_end(ap);
        return retval;
    }
    return 0;
}

