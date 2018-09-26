# CANZERO core firmware for SAMD21G18

This repository contains all the source code for the SAMD21G18 ARM Cortex
MCU on the [CANZERO][1]

## Directories

`arduino-package` contains the source code for the Arduino package needed to user NoCAN in the arduino IDE.

`bootloader` contains the SAMD21G18 bootloader for NoCAN.

`default-application` contains the default application that ships with a new CANZERO board on the SAMD21G18.

`lib` contains common libraries (SPI communications with the STM#@ driver IC).

## Compiling

Compilation relies on the the `arm-none-eabi-gcc` toolchain and `make`.

Compiling the firmware requires the very small subset of CMSIS libraries that come with [ATMEL ASF 3][2]. The code itself does not use ASF but relies on direct programming of SAMD21G18 registers.

To compile the code, you will need to edit the `ASF_ROOT` variable in the Makefile to point to the location of your copy of the ASF.

## License

This code is licensed under the MIT licence, as described in the LICENSE file.

[1]: http://omzlo.com/canzero
[2]: http://www.microchip.com/mplab/avr-support/advanced-software-framework
