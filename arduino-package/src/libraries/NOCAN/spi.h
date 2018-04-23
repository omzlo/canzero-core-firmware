#ifndef _SPI_H_
#define _SPI_H_

#ifdef __cplusplus
extern "C" {
#endif


#include <stdint.h>

int spi_init(uint32_t baud);

void spi_begin(void);

void spi_end(void);

uint8_t spi_transfer(uint8_t b);

#ifdef __cplusplus
}
#endif

#endif
