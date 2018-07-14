
OPENCM3_PATH         ?= ../../libopencm3
OPENCM3_LIBRARY_PATH ?= $(OPENCM3_PATH)/lib
OPENCM3_INCLUDE_PATH ?= $(OPENCM3_PATH)/include

CC      := arm-none-eabi-gcc
OBJCOPY := arm-none-eabi-objcopy
OBJDUMP := arm-none-eabi-objdump
SIZE    := arm-none-eabi-size


# CFLAGS  = -Wall -g -std=gnu11 -Os  
CFLAGS  = -Wall -g -std=gnu11 -O1  
CFLAGS += -mlittle-endian -mthumb
CFLAGS += -mcpu=cortex-m3
CFLAGS += -mfix-cortex-m3-ldrd
CFLAGS += -msoft-float
#CFLAGS += -ffunction-sections -fdata-sections -Wl,--gc-sections 
CFLAGS += -Wl,-Map=main.map


CFLAGS += -Tstm32f103c8t6.ld


CFLAGS += -I$(OPENCM3_INCLUDE_PATH)
CFLAGS += -L$(OPENCM3_LIBRARY_PATH)

CFLAGS += -DSTM32F1

CFLAGS += --static -nostartfiles



LDFLAGS += -Wl,--start-group -lc -lgcc -lnosys -Wl,--end-group
LDFLAGS += -lopencm3_stm32f1 -lm

AS = arm-none-eabi-as

all: main.elf


cr4_fft_1024_stm32.o: cr4_fft_1024_stm32.s
	$(AS) -o cr4_fft_1024_stm32.o cr4_fft_1024_stm32.s

%.o: %.c
	$(CC) $(CFLAGS) -c -o $@ $<


main.elf: main.o usb.o
	$(CC) $(CFLAGS) -o main.elf main.o usb.o $(LDFLAGS)

	$(OBJCOPY) -O ihex $@ $(@:.elf=.hex)
	$(OBJCOPY) -O binary $@ $(@:.elf=.bin)
	$(OBJDUMP) -St $@ >$(@:.elf=.lst)
	$(SIZE) $@



flash: main.elf
	openocd -f openocd.cfg -c "program $< verify reset exit"


clean:
	rm -f *.o
	rm -f *.hex
	rm -f *.lst
	rm -f *.map
	rm -f *.elf
	rm -f *.bin

	


.PHONY: flash clean

