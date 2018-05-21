#ifdef ARDUINO
    #include "sam.h"
#else
    #include <samd21.h>
#endif
#include "spi.h"



/*
 * SPI on SERCOM 2 (SERCOM)
 * MOSI -> PA12, PAD0
 * SCK  -> PA13, PAD1
 * MISO -> PA15, PAD3
 * _SS  -> PA14, PAD2
 */

#define GPIO_PMUXEN(group, pin, mux) \
    do { \
        PORT->Group[group].PINCFG[(pin&31)].bit.PMUXEN = 1; \
        if (pin&1) \
          PORT->Group[group].PMUX[(pin&31)>>1].bit.PMUXO = mux; \
        else \
          PORT->Group[group].PMUX[(pin&31)>>1].bit.PMUXE = mux; \
    } while (0)

#define GPIO_A 0
#define GPIO_B 1
#define GPIO_C 2

int spi_init(uint32_t baud)
{
    uint32_t rbaud = 48000000/(baud*2) - 1;

    // PA12
    GPIO_PMUXEN(GPIO_A, PIN_PA12, PORT_PMUX_PMUXE_C);
    // PA13
    GPIO_PMUXEN(GPIO_A, PIN_PA13, PORT_PMUX_PMUXE_C);
    // PA15
    GPIO_PMUXEN(GPIO_A, PIN_PA15, PORT_PMUX_PMUXE_C);
    // PA14, _SS
    // GPIO_PMUXEN(GPIO_A, PIN_PA14, PORT_PMUX_PMUXE_C);

    // PA14, _SS as out
    PORT->Group[GPIO_A].DIRSET.reg = PORT_PA14; // Probably not needed
    PORT->Group[GPIO_A].OUTSET.reg = PORT_PA14; // SET high
    
    // Enable bus clock for SERCOM2
    PM->APBCMASK.reg |= PM_APBCMASK_SERCOM2;

    // Select clock source for SERCOM2
    GCLK->CLKCTRL.reg = 
        GCLK_CLKCTRL_ID(SERCOM2_GCLK_ID_CORE) |
        GCLK_CLKCTRL_GEN_GCLK0 |    // Generic clock generator 0 is source
        GCLK_CLKCTRL_CLKEN;         // enable

    // Do software reset of SPI on SERCOM2
    SERCOM2->SPI.CTRLA.reg = SERCOM_SPI_CTRLA_SWRST;
    while (SERCOM2->SPI.CTRLA.reg & SERCOM_SPI_CTRLA_SWRST);


    // Configure pads, set as master
    SERCOM2->SPI.CTRLA.reg = 
        SERCOM_SPI_CTRLA_DOPO(0) |  // Data Out PinOut, 0 => DO:PAD0, SCK:PAD1, SS:PAD2
        SERCOM_SPI_CTRLA_DIPO(3) |  // Data In Pinouti, 3 => DI:PAD3
        SERCOM_SPI_CTRLA_MODE_SPI_MASTER;

    // set baud rate
    SERCOM2->SPI.BAUD.reg = rbaud;

    SERCOM2->SPI.CTRLB.reg = SERCOM_SPI_CTRLB_RXEN; // Receive enable
        // We do not set hardware controlled _SS since it drives _SS high between each byte.

    // Enable SPI
    SERCOM2->SPI.CTRLA.reg |= SERCOM_SPI_CTRLA_ENABLE;

    return 0;
}

void spi_begin(void) 
{
    PORT->Group[GPIO_A].OUTCLR.reg = PORT_PA14;     // Select slave (SS = 0)
}

void spi_end(void) 
{
    /* Notes: 
       - You MUST wait for SERCOMx->SPI.INTFLAG.bit.TXC before unselecting slave.
     */

    while (SERCOM2->SPI.INTFLAG.bit.TXC==0);       // Wait for transmit to finish
    PORT->Group[GPIO_A].OUTSET.reg = PORT_PA14;     // Deselect slave (SS = 1)
}

uint8_t spi_transfer(uint8_t b)
{
    SERCOM2->SPI.DATA.reg = b;
    while (SERCOM2->SPI.INTFLAG.bit.RXC==0);
    return SERCOM2->SPI.DATA.reg;
}
