# put your *.o targets here, make should handle the rest!
CMSIS_PATH 		?= STM32Cube/Drivers/CMSIS
CMSIS_DEV_PATH  ?= $(CMSIS_PATH)/Device/ST/STM32F0xx
HAL_PATH   		?= STM32Cube/Drivers/STM32F0xx_HAL_Driver

MAC_ADDR ?= 0xDEBE10BB

CC      := arm-none-eabi-gcc
OBJCOPY := arm-none-eabi-objcopy
OBJDUMP := arm-none-eabi-objdump
SIZE    := arm-none-eabi-size

#CFLAGS  = -Wall -Wpedantic -Wstrict-aliasing -g -std=gnu11 -Os  
CFLAGS  = -Wall -Wpedantic -Wstrict-aliasing -g -std=gnu11 -O1
CFLAGS += -mlittle-endian -mcpu=cortex-m0 -march=armv6-m -mthumb
CFLAGS += -ffunction-sections -fdata-sections -Wl,--gc-sections 
CFLAGS += -Wl,-Map=main.map

# Technically we're using an STM32F030F4, but apart from the TSSOP20 package that one is largely identical to the
# STM32F030*6 and there is no separate device header provided for it, so we're faking a *6 device here. This is
# even documented in stm32f0xx.h. Thanks ST!
CFLAGS += -DSTM32F030x6 -DHSE_VALUE=25000000 -DMAC_ADDR=$(MAC_ADDR)

CFLAGS += -Tstm32_flash.ld
CFLAGS += -I$(CMSIS_DEV_PATH)/Include -I$(CMSIS_PATH)/Include -I$(HAL_PATH)/Inc -Iconfig
CFLAGS += -L$(CMSIS_PATH)/Lib/GCC -larm_cortexM0l_math

###################################################

.PHONY: program clean

all: main.elf

cmsis_exports.c: $(CMSIS_DEV_PATH)/Include/stm32f030x6.h $(CMSIS_PATH)/Include/core_cm0.h
	python3 gen_cmsis_exports.py $^ > $@

sources.tar.xz: main.c serial.h global.h serial.c adc.h adc.c Makefile
	tar -cf $@ $^

# don't ask...
sources.tar.xz.zip: sources.tar.xz
	zip $@ $^

sources.c: sources.tar.xz.zip
	xxd -i $< | head -n -1 | sed 's/=/__attribute__((section(".source_tarball"))) =/' > $@

main.elf: main.c adc.c serial.c startup_stm32f030x6.s system_stm32f0xx.c $(HAL_PATH)/Src/stm32f0xx_ll_utils.c cmsis_exports.c sources.c
	$(CC) $(CFLAGS) -o $@ $^
	$(OBJCOPY) -O ihex $@ $(@:.elf=.hex)
	$(OBJCOPY) -O binary $@ $(@:.elf=.bin)
	$(OBJDUMP) -St $@ >$(@:.elf=.lst)
	$(SIZE) $@
	
program: main.elf openocd.cfg
	openocd -f openocd.cfg -c "program $< verify reset exit"

clean:
	rm -f *.o
	rm -f main.elf main.hex main.bin main.map main.lst
	rm -f sources.tar.xz sources.tar.xz.zip

