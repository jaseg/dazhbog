
OPENCM3_PATH         ?= $(HOME)/resource/libopencm3

CC      := arm-none-eabi-gcc
OBJCOPY := arm-none-eabi-objcopy
OBJDUMP := arm-none-eabi-objdump
SIZE    := arm-none-eabi-size
AS = arm-none-eabi-as

CFLAGS  = -Wall -Wextra -g -std=gnu11 -O1  
CFLAGS += -mlittle-endian -mthumb -mcpu=cortex-m3 -mfix-cortex-m3-ldrd -msoft-float
#CFLAGS += -ffunction-sections -fdata-sections -Wl,--gc-sections 
#CFLAGS += -Wl,-Map=main.map
CFLAGS += -Tstm32f103c8t6.ld --static -nostartfiles
CFLAGS += -I$(OPENCM3_PATH)/include -L$(OPENCM3_PATH)/lib
CFLAGS += -DSTM32F1

LDFLAGS += -Wl,--start-group -lc -lgcc -lnosys -Wl,--end-group
LDFLAGS += -lopencm3_stm32f1 -lm

all: main.elf

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
	rm -f $(addprefix main,.o .hex .lst .map .elf .bin)

.PHONY: flash clean

