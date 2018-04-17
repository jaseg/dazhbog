/* Megumin LED display firmware
 * Copyright (C) 2018 Sebastian Götte <code@jaseg.net>
 * 
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 * 
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <stm32f0xx.h>
#include <stdint.h>
#include <system_stm32f0xx.h>
#include <stm32f0xx_ll_utils.h>
#include <math.h>

#include "adc.h"

volatile int16_t adc_vcc_mv = 0;
volatile int16_t adc_temp_celsius = 0;

static volatile uint16_t adc_buf[2];

void adc_init(void) {
    /* The ADC is used for temperature measurement. To compute the temperature from an ADC reading of the internal
     * temperature sensor, the supply voltage must also be measured. Thus we are using two channels.
     *
     * The ADC is triggered by compare channel 4 of timer 1. The trigger is set to falling edge to trigger on compare
     * match, not overflow.
     */
    ADC1->CFGR1 = ADC_CFGR1_DMAEN | ADC_CFGR1_DMACFG | (2<<ADC_CFGR1_EXTEN_Pos) | (1<<ADC_CFGR1_EXTSEL_Pos);
    /* Clock from PCLK/4 instead of the internal exclusive high-speed RC oscillator. */
    ADC1->CFGR2 = (2<<ADC_CFGR2_CKMODE_Pos);
    /* Use the slowest available sample rate */
    ADC1->SMPR  = (7<<ADC_SMPR_SMP_Pos);
    /* Internal VCC and temperature sensor channels */
    ADC1->CHSELR = ADC_CHSELR_CHSEL16 | ADC_CHSELR_CHSEL17;
    /* Enable internal voltage reference and temperature sensor */
    ADC->CCR = ADC_CCR_TSEN | ADC_CCR_VREFEN;
    /* Perform ADC calibration */
    ADC1->CR |= ADC_CR_ADCAL;
    while (ADC1->CR & ADC_CR_ADCAL)
        ;
    /* Enable ADC */
    ADC1->CR |= ADC_CR_ADEN;
    ADC1->CR |= ADC_CR_ADSTART;

    /* Configure DMA 1 Channel 1 to get rid of all the data */
    DMA1_Channel1->CPAR = (unsigned int)&ADC1->DR;
    DMA1_Channel1->CMAR = (unsigned int)&adc_buf;
    DMA1_Channel1->CNDTR = sizeof(adc_buf)/sizeof(adc_buf[0]);
    DMA1_Channel1->CCR = (0<<DMA_CCR_PL_Pos);
    DMA1_Channel1->CCR |=
          DMA_CCR_CIRC /* circular mode so we can leave it running indefinitely */
        | (1<<DMA_CCR_MSIZE_Pos) /* 16 bit */
        | (1<<DMA_CCR_PSIZE_Pos) /* 16 bit */
        | DMA_CCR_MINC
        | DMA_CCR_TCIE; /* Enable transfer complete interrupt. */
    DMA1_Channel1->CCR |= DMA_CCR_EN; /* Enable channel */

    /* triggered on transfer completion. We use this to process the ADC data */
    NVIC_EnableIRQ(DMA1_Channel1_IRQn);
    NVIC_SetPriority(DMA1_Channel1_IRQn, 3);
}

void DMA1_Channel1_IRQHandler(void) {
    /* This interrupt takes either 1.2us or 13us. It can be pre-empted by the more timing-critical UART and LED timer
     * interrupts. */
    static int count = 0; /* oversampling accumulator sample count */
    static uint32_t adc_aggregate[2] = {0, 0}; /* oversampling accumulator */

    /* Clear the interrupt flag */
    DMA1->IFCR |= DMA_IFCR_CGIF1;

    adc_aggregate[0] += adc_buf[0];
    adc_aggregate[1] += adc_buf[1];

    if (++count == (1<<ADC_OVERSAMPLING)) {
        /* This has been copied from the code examples to section 12.9 ADC>"Temperature sensor and internal reference
         * voltage" in the reference manual with the extension that we actually measure the supply voltage instead of
         * hardcoding it. This is not strictly necessary since we're running off a bored little LDO but it's free and
         * the current supply voltage is a nice health value.
         */
        adc_vcc_mv = (3300 * VREFINT_CAL)/(adc_aggregate[0]>>ADC_OVERSAMPLING);
        int32_t temperature = (((uint32_t)TS_CAL1) - ((adc_aggregate[1]>>ADC_OVERSAMPLING) * adc_vcc_mv / 3300)) * 1000;
        temperature = (temperature/5336) + 30;
        adc_temp_celsius = temperature;

        count = 0;
        adc_aggregate[0] = 0;
        adc_aggregate[1] = 0;
    }
}

