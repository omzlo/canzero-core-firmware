#ifndef _SERIAL_H_
#define _SERIAL_H_

#include <stdarg.h>

int serial_open(uint32_t baud);
int serial_close(void);
int serial_putc(int c);
int serial_getc(void);
int serial_available(void);

int serial_vprintf(const char *format, va_list ap);
int serial_printf(const char *format, ...);

extern unsigned serial_debug_enable;
int serial_debug_printf(const char *format, ...);

int serial_reset(void);

#endif
