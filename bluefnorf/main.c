/*
 * ADC test: Try to get as many samples as possible
 * using fast interleave mode and DMA
 */

#include <stdlib.h>
#include <string.h>
#include <stdio.h>

#include <libopencm3/cm3/nvic.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/adc.h>
#include <libopencm3/stm32/dma.h>

#include "usb.h"

int main(void) {
    rcc_clock_setup_in_hse_8mhz_out_72mhz();
    usb_serial_init();

    while (42) {
    }
}

void hard_fault_handler(void) {
    while (23);
}
