#ifdef ARDUINO
    #include "sam.h"
#else
    #include <samd21.h>
#endif
#include "spi.h"



/*
 * SPI on SERCOM 4 (SERCOM-ALT)
 *  
 * MOSI -> PB10, PAD2
 * SCK  -> PB11, PAD3
 * MISO -> PA12, PAD0
 * _SS  -> PA13, PAD1
 *
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

#ifdef CANZERO_MKR
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

#else
    // PB10
    GPIO_PMUXEN(GPIO_B, PIN_PB10, PORT_PMUX_PMUXE_D);
    // PB11
    GPIO_PMUXEN(GPIO_B, PIN_PB11, PORT_PMUX_PMUXE_D);
    // PA12
    GPIO_PMUXEN(GPIO_A, PIN_PA12, PORT_PMUX_PMUXE_D);
    // PA13, _SS
    // GPIO_PMUXEN(GPIO_A, PIN_PA13, PORT_PMUX_PMUXE_D);

    // PA13, _SS as out
    PORT->Group[GPIO_A].DIRSET.reg = PORT_PA13; // Probably not needed
    //PORT->Group[GPIO_B].DIRSET.reg = PORT_PB10;
    //PORT->Group[GPIO_B].DIRSET.reg = PORT_PB11;
    // set _SS as high
    PORT->Group[GPIO_A].OUTSET.reg = PORT_PA13;
    
    // Enable bus clock for SERCOM4
    PM->APBCMASK.reg |= PM_APBCMASK_SERCOM4;

    // Select clock source for SERCOM4
    GCLK->CLKCTRL.reg = 
        GCLK_CLKCTRL_ID(SERCOM4_GCLK_ID_CORE) |
        GCLK_CLKCTRL_GEN_GCLK0 |    // Generic clock generator 0 is source
        GCLK_CLKCTRL_CLKEN;         // enable

    // Do software reset of SPI on SERCOM4
    SERCOM4->SPI.CTRLA.reg = SERCOM_SPI_CTRLA_SWRST;
    while (SERCOM4->SPI.CTRLA.reg & SERCOM_SPI_CTRLA_SWRST);


    // Configure pads, set as master
    SERCOM4->SPI.CTRLA.reg = 
        SERCOM_SPI_CTRLA_DOPO(1) |  // Data Out PinOut, 1 => DO:PAD2, SCK:PAD3, SS:PAD1
        SERCOM_SPI_CTRLA_DIPO(0) |  // Data In Pinout
        SERCOM_SPI_CTRLA_MODE_SPI_MASTER;

    // set baud rate
    SERCOM4->SPI.BAUD.reg = rbaud;

    SERCOM4->SPI.CTRLB.reg = SERCOM_SPI_CTRLB_RXEN; // Receive enable
        // We do not set hardware controlled _SS since it drives _SS high between each byte.

    // Enable SPI
    SERCOM4->SPI.CTRLA.reg |= SERCOM_SPI_CTRLA_ENABLE;
#endif
    return 0;
}

void spi_begin(void) 
{
#ifdef CANZERO_MKR
    PORT->Group[GPIO_A].OUTCLR.reg = PORT_PA14;     // Select slave (SS = 0)
#else
    PORT->Group[GPIO_A].OUTCLR.reg = PORT_PA13;     // Select slave (SS = 0)
#endif
}

void spi_end(void) 
{
    /* Notes: 
       - You MUST wait for SERCOMx->SPI.INTFLAG.bit.TXC before unselecting slave.
     */

#ifdef CANZERO_MKR
    while (SERCOM2->SPI.INTFLAG.bit.TXC==0);       // Wait for transmit to finish
    PORT->Group[GPIO_A].OUTSET.reg = PORT_PA14;     // Deselect slave (SS = 1)
#else
     while (SERCOM4->SPI.INTFLAG.bit.TXC==0);       // Wait for transmit to finish
    // SERCOM4->SPI.INTFLAG.bit.TXC = 1;             // clear Int flag
    PORT->Group[GPIO_A].OUTSET.reg = PORT_PA13;     // Deselect slave (SS = 1)
#endif
}

uint8_t spi_transfer(uint8_t b)
{
#ifdef CANZERO_MKR
    SERCOM2->SPI.DATA.reg = b;
    while (SERCOM2->SPI.INTFLAG.bit.RXC==0);
    return SERCOM2->SPI.DATA.reg;
#else
    SERCOM4->SPI.DATA.reg = b;
    while (SERCOM4->SPI.INTFLAG.bit.RXC==0);
    return SERCOM4->SPI.DATA.reg;
#endif
}
