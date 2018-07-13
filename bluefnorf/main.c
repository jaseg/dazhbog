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

int main(void)
{
	int i = 0;
    // const char* line;

    // Clock Setup
    rcc_clock_setup_in_hse_8mhz_out_72mhz();

    // Initialize USB
	usb_serial_init();

	while (1) {
        /*
        if( i % 100000  == 0 ) {
            // Read ADC
            printf("Fnord 42 :: %d %d\r\n", _adc_samples[0], _adc_samples[1]);
        }
        */

        i++;
    }
}
