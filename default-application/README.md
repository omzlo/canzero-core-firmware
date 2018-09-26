# Flashing the bootloader + default application with Atmel ICE and openocd

**Step 1:** run `make combined`

**Step 2:** Exit debugger with `^C`

**Step 3:** Generate bin firmware with `arm-none-eabi-objcopy -I ihex -O binary combined.hex combined.bin`

**Step 4:** run `openocd -f openocd-flash.cfg`
