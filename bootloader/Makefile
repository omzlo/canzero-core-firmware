# Tools

LDSCRIPT = samd21g18a_flash.ld
STARTUP = startup_samd21.o system_samd21.o
PTYPE=__SAMD21G18A__

CC=arm-none-eabi-gcc
LD=arm-none-eabi-gcc
AR=arm-none-eabi-ar
AS=arm-none-eabi-as
OBJCOPY=arm-none-eabi-objcopy
SIZEUP=arm-none-eabi-size

ELF=$(notdir $(CURDIR)).elf
HEX=$(notdir $(CURDIR)).hex

ASF_ROOT=../../../Src/xdk-asf-3.33.0

INCLUDES= \
		  sam0/utils/cmsis/samd21/include \
		  sam0/utils/cmsis/samd21/source \
		  thirdparty/CMSIS/Include \
		  thirdparty/CMSIS/Lib/GCC


#OBJS=$(STARTUP) main.o $(patsubst %.c,$(ASF_ROOT)/%.o,$(SOURCES))
OBJS = $(STARTUP) main.o systick.o serial.o ../omzlo-zero-lib/spi.o ../omzlo-zero-lib/nocan_ll.o

LDFLAGS+= -T$(LDSCRIPT) -mthumb -mcpu=cortex-m0 -Wl,--gc-sections 
CFLAGS+= -mcpu=cortex-m0 -mthumb -g -Os
CFLAGS+= $(INCLUDES:%=-I $(ASF_ROOT)/%) -I . -I ../omzlo-zero-lib/
CFLAGS+= -D$(PTYPE)
CFLAGS+= -DDEBUG
CFLAGS+= -DCANZERO_MKR
CFLAGS+=-pipe -Wall -Werror-implicit-function-declaration -Wpointer-arith -fno-strict-aliasing -ffunction-sections -fdata-sections -Wchar-subscripts -Wcomment -Wformat=2 -Wmain -Wparentheses -Wsequence-point -Wreturn-type -Wswitch -Wtrigraphs -Wunused -Wuninitialized -Wunknown-pragmas -Wfloat-equal -Wundef -Wshadow  -Wwrite-strings -Wsign-compare -Waggregate-return -Wmissing-declarations -Wformat -Wmissing-format-attribute -Wno-deprecated-declarations -Wpacked -Wredundant-decls -Wlong-long -Wunreachable-code -Wcast-align --param max-inline-insns-single=500

$(ELF):		$(OBJS)
			$(LD) $(LDFLAGS) -o $@ $(OBJS) $(LDLIBS)
			$(OBJCOPY) -O ihex $(ELF) $(HEX)
			$(SIZEUP) $@

# compile and generate dependency info

%.o:	%.c
		$(CC) -c $(CFLAGS) $< -o $@
		$(CC) -MM $(CFLAGS) $< > $*.d

%.o:	%.s
		$(AS) $< -o $@

info:		
		@echo CFLAGS=$(CFLAGS)
		@echo OBJS=$(OBJS)

clean:
		rm -f $(OBJS) $(OBJS:.o=.d) $(ELF) $(HEX) $(CLEANOTHER)

debug:	$(ELF)
		arm-none-eabi-gdb -iex "target extended-remote localhost:3333" $(ELF)
# pull in dependencies

-include	$(OBJS:.o=.d)
