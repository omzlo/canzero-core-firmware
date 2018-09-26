# Tools

LDSCRIPT = samd21g18a_flash_with_bootloader.ld
STARTUP = startup_samd21.o system_samd21.o
PTYPE=__SAMD21G18A__

CC=arm-none-eabi-gcc
CXX=arm-none-eabi-g++
LD=arm-none-eabi-gcc
AR=arm-none-eabi-ar
AS=arm-none-eabi-as
OBJCOPY=arm-none-eabi-objcopy
SIZEUP=arm-none-eabi-size

ELF=$(notdir $(CURDIR)).elf
HEX=$(notdir $(CURDIR)).hex
BOOTLOADER=../bootloader/bootloader.hex

ASF_ROOT=../../../../Src/xdk-asf-3.33.0

INCLUDES= \
		  sam0/utils/cmsis/samd21/include \
		  sam0/utils/cmsis/samd21/source \
		  thirdparty/CMSIS/Include \
		  thirdparty/CMSIS/Lib/GCC


#OBJS=$(STARTUP) main.o $(patsubst %.c,$(ASF_ROOT)/%.o,$(SOURCES))
OBJS = $(STARTUP) main.o systick.o serial.o ../lib/spi.o ../lib/nocan_ll.o ../lib/nocan.o

LDFLAGS+= -T$(LDSCRIPT) -mthumb -mcpu=cortex-m0 -Wl,--gc-sections 
CFLAGS+= -mcpu=cortex-m0 -mthumb -g -Os
CFLAGS+= $(INCLUDES:%=-I $(ASF_ROOT)/%) -I . -I ../lib/
CFLAGS+= -D$(PTYPE)
# CFLAGS+= -DDEBUG
CFLAGS+=-pipe -Wall -Werror-implicit-function-declaration -Wpointer-arith -fno-strict-aliasing -ffunction-sections -fdata-sections -Wchar-subscripts -Wcomment -Wformat=2 -Wmain -Wparentheses -Wsequence-point -Wreturn-type -Wswitch -Wtrigraphs -Wunused -Wuninitialized -Wunknown-pragmas -Wfloat-equal -Wundef -Wshadow  -Wwrite-strings -Wsign-compare -Waggregate-return -Wmissing-declarations -Wformat -Wmissing-format-attribute -Wno-deprecated-declarations -Wpacked -Wredundant-decls -Wlong-long -Wunreachable-code -Wcast-align --param max-inline-insns-single=500

$(ELF):		$(OBJS)
			$(LD) $(LDFLAGS) -o $@ $(OBJS) $(LDLIBS)
			$(SIZEUP) $@

$(HEX):		$(ELF)
			$(OBJCOPY) -O ihex $(ELF) $(HEX)

combined:	$(HEX) $(BOOTLOADER)
			srec_cat $(BOOTLOADER) -Intel $(HEX) -Intel -Output combined.hex -Intel			
			arm-none-eabi-gdb -iex "target extended-remote localhost:3333" combined.hex

# compile and generate dependency info

%.o:	%.c
		$(CC) -c $(CFLAGS) $< -o $@
		$(CC) -MM $(CFLAGS) $< > $*.d

%.o:	%.cpp
		$(CXX) -c $(CFLAGS) $< -o $@

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
