/* OpenStep 2
 * Copyright (C) 2017 Sebastian Götte <code@jaseg.net>
 * 
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Affero General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 * 
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Affero General Public License for more details.
 * 
 * You should have received a copy of the GNU Affero General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

/* Preliminary remarks.
 *
 * This code is intended to run on an ARM Cortex-M0 microcontroller made by ST, part number STM32F030F4C6
 *
 * Some terminology:
 * 
 * * The term "raw channel" refers to a single output of the 32 outputs provided by the driver board. It corresponds to
 *   a single color sub-channel of one RGBW output. One RGBW output consists of four raw channels.
 *
 * * The term "logical channel" refers to one RGBW output of four individual colors handled by a group of four raw
 *   channels.
 */

#include <stm32f0xx.h>
#include <stdint.h>
#include <system_stm32f0xx.h>
#include <stm32f0xx_ll_utils.h>
#include <math.h>

#include "global.h"
#include "serial.h"
#include "adc.h"

/* Bit count of this device. Note that to change this you will also have to adapt the per-bit timer period lookup table
 * below.
 */
#define NBITS 14

/* Maximum bit count supported by serial command protocol. The brightness data is assumed to be of this bit width, but
 * only the uppermost NBITS bits are used. */
#define MAX_BITS 16

void do_transpose(void);

/* Bit-golfed modulation data generated from the above values by the main loop, ready to be sent out to the shift
 * registers.
 */
volatile uint32_t brightness_by_bit[NBITS] = { 0 };

/* Global systick timing variables */
uint32_t sys_time = 0;
uint32_t sys_time_seconds = 0;

int main(void) {
    /* Get all the good clocks and PLLs on this thing up and running. We're running from an external 25MHz crystal,
     * which we're first dividing down by 5 to get 5 MHz, then PLL'ing up by 6 to get 30 MHz as our main system clock.
     *
     * The busses are all run directly from these 30 MHz because why not.
     *
     * Be careful in mucking around with this code since you can kind of semi-brick the chip if you do it wrong.
     */
    RCC->CR |= RCC_CR_HSEON;
    while (!(RCC->CR&RCC_CR_HSERDY));

    // HSE ready, let's configure the PLL
    RCC->CFGR &= ~RCC_CFGR_PLLMUL_Msk & ~RCC_CFGR_SW_Msk & ~RCC_CFGR_PPRE_Msk & ~RCC_CFGR_HPRE_Msk;

    // PLLMUL: 6x (0b0100)
    RCC->CFGR |= (0b0100<<RCC_CFGR_PLLMUL_Pos) | RCC_CFGR_PLLSRC_HSE_PREDIV;

    // PREDIV:
    // HSE / PREDIV = PLL SRC
    RCC->CFGR2 &= ~RCC_CFGR2_PREDIV_Msk;
    RCC->CFGR2 |= RCC_CFGR2_PREDIV_DIV5; /* prediv :10 -> 5 MHz */

    RCC->CR |= RCC_CR_PLLON;
    while (!(RCC->CR&RCC_CR_PLLRDY));

    RCC->CFGR |= (2<<RCC_CFGR_SW_Pos);

    SystemCoreClockUpdate();
    SysTick_Config(SystemCoreClock/1000); /* 1ms interval */


    /* Enable all the periphery we need */
    RCC->AHBENR  |= RCC_AHBENR_GPIOAEN | RCC_AHBENR_GPIOBEN;
    RCC->APB2ENR |= RCC_APB2ENR_SPI1EN | RCC_APB2ENR_TIM1EN | RCC_APB2ENR_USART1EN | RCC_APB2ENR_ADCEN;
    RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;

    /* Configure all the GPIOs */
    GPIOA->MODER |=
          (3<<GPIO_MODER_MODER0_Pos)  /* PA0  - Current measurement analog input */
        | (1<<GPIO_MODER_MODER1_Pos)  /* PA1  - RS485 TX enable */
        | (2<<GPIO_MODER_MODER2_Pos)  /* PA2  - RS485 TX */
        | (2<<GPIO_MODER_MODER3_Pos)  /* PA3  - RS485 RX */
        /* PA4 reserved because */
        | (2<<GPIO_MODER_MODER5_Pos)  /* PA5  - Shift register clk/SCLK */
        | (1<<GPIO_MODER_MODER6_Pos)  /* PA6  - LED2 open-drain output */
        | (2<<GPIO_MODER_MODER7_Pos)  /* PA7  - Shift register data/MOSI */
        | (2<<GPIO_MODER_MODER9_Pos)  /* FIXME PA9  - Shift register clear (TIM1_CH2) */
        | (2<<GPIO_MODER_MODER10_Pos);/* PA10 - Shift register strobe (TIM1_CH3) */
    GPIOB->MODER |=
          (2<<GPIO_MODER_MODER1_Pos); /* PB1  - Shift register clear (TIM1_CH3N) */


    GPIOA->OTYPER |= GPIO_OTYPER_OT_6; /* LED outputs -> open drain */

    /* Set shift register IO GPIO output speed */
    GPIOA->OSPEEDR |=
          (3<<GPIO_OSPEEDR_OSPEEDR5_Pos)  /* SCLK   */
        | (3<<GPIO_OSPEEDR_OSPEEDR6_Pos)  /* LED1   */
        | (3<<GPIO_OSPEEDR_OSPEEDR7_Pos)  /* MOSI   */
        | (3<<GPIO_OSPEEDR_OSPEEDR10_Pos);/* Strobe */
    GPIOB->OSPEEDR |=
          (3<<GPIO_OSPEEDR_OSPEEDR1_Pos); /* Clear  */

    /* Alternate function settings */
    GPIOA->AFR[0] |=
          (1<<GPIO_AFRL_AFRL2_Pos)   /* USART1_TX */
        | (1<<GPIO_AFRL_AFRL3_Pos)   /* USART1_RX */
        | (0<<GPIO_AFRL_AFRL5_Pos)   /* SPI1_SCK  */
        | (0<<GPIO_AFRL_AFRL7_Pos);  /* SPI1_MOSI */
    GPIOA->AFR[1] |=
          (2<<GPIO_AFRH_AFRH2_Pos);  /* TIM1_CH3  */
    GPIOB->AFR[0] |=
          (2<<GPIO_AFRL_AFRL1_Pos);  /* TIM1_CH3N */

    /* Configure SPI controller */
    /* CPOL=0, CPHA=0, prescaler=2 -> 16MBd */
    SPI1->CR1 = SPI_CR1_BIDIMODE | SPI_CR1_BIDIOE | SPI_CR1_SSM | SPI_CR1_SSI | SPI_CR1_SPE | (0<<SPI_CR1_BR_Pos) | SPI_CR1_MSTR;
    SPI1->CR2 = (0xf<<SPI_CR2_DS_Pos);

    /* Configure TIM1 for display strobe generation */
    TIM1->CR1 = TIM_CR1_ARPE;

    TIM1->PSC = 1; /* Prescale by 2, resulting in a 16MHz timer frequency and 62.5ns timer step size. */
    /* CH2 - clear/!MR, CH3 - strobe/STCP */
    TIM1->CCMR2 = (6<<TIM_CCMR2_OC3M_Pos) | TIM_CCMR2_OC3PE;
    TIM1->CCER |= TIM_CCER_CC3E | TIM_CCER_CC3NE | TIM_CCER_CC3P | TIM_CCER_CC3NP;
    TIM1->BDTR = TIM_BDTR_MOE | (1<<TIM_BDTR_DTG_Pos); /* really short dead time */
    TIM1->DIER = TIM_DIER_UIE; /* Enable update (overrun) interrupt */
    TIM1->ARR = 1;
    TIM1->CR1 |= TIM_CR1_CEN;

    /* Configure Timer 1 update (overrun) interrupt on NVIC.  Used only for update (overrun) for strobe timing. */
    NVIC_EnableIRQ(TIM1_BRK_UP_TRG_COM_IRQn);
    NVIC_SetPriority(TIM1_BRK_UP_TRG_COM_IRQn, 1);

    /* Pre-load initial values, kick of first interrupt */
    TIM1->EGR |= TIM_EGR_UG;

    /* Configure TIM3 for USART timeout handing */
    TIM3->CR1 = TIM_CR1_OPM;
    TIM3->DIER = TIM_DIER_UIE;
    TIM3->PSC = 30;
    TIM3->ARR = 1000;

    /* Configure Timer 3 update (overrun) interrupt on NVIC.  Used only for update (overrun) for USART timeout handling. */
    NVIC_EnableIRQ(TIM3_IRQn);
    NVIC_SetPriority(TIM3_IRQn, 2);

    /* Pre-load initial values */
    TIM3->EGR |= TIM_EGR_UG;

    /* Configure UART for RS485 comm */
    /* 8N1, 1MBd */
    USART1->CR1 = /* 8-bit -> M1, M0 clear */
        /* RTOIE clear */
          (8 << USART_CR1_DEAT_Pos) /* 8 sample cycles/1 bit DE assertion time */
        | (8 << USART_CR1_DEDT_Pos) /* 8 sample cycles/1 bit DE assertion time */
        /* CMIF clear */
        /* WAKE clear */
        /* PCE, PS clear */
        | USART_CR1_RXNEIE
        /* other interrupts clear */
        | USART_CR1_TE
        | USART_CR1_RE;
    USART1->CR3 = USART_CR3_DEM; /* RS485 DE enable (output on RTS) */
    // USART1->BRR = 30;
    USART1->BRR = 40; // 750000
    USART1->CR1 |= USART_CR1_UE;

    /* Configure USART1 interrupt on NVIC. Used only for RX. */
    NVIC_EnableIRQ(USART1_IRQn);
    NVIC_SetPriority(USART1_IRQn, 2);

    adc_init();

    /* Idly loop around, occassionally disfiguring some integers. */
    while (42) {
        /* Debug output on LED. */
        GPIOA->ODR ^= GPIO_ODR_6;

        if (framebuf_out_of_sync != 0) {
            /* This logic is very slightly racy, but that should not matter since we're updating the frame buffer often
             * enough so you don't notice one miss every billion frames. */
            framebuf_out_of_sync = 0;
            /* Bit-mangle the integer framebuf data to produce raw modulation data */
            do_transpose();
        }
    }
}

/* Modulation data bit golfing routine */
void do_transpose(void) {
    /* For each bit value */
    for (uint32_t i=0; i<NBITS; i++) {
        uint32_t mask = 1<<i<<(MAX_BITS-NBITS); /* Bit mask for this bit value. */
        uint32_t bv = 0; /* accumulator thing */
        for (uint32_t j=0; j<32; j++) {
            if (rx_buf.set_fb_rq.framebuf[j] & mask)
            bv |= 1<<j;
        }
        brightness_by_bit[i] = bv;
    }
}

/* Bit timing base value. This is the lowes bit interval used */
#define PERIOD_BASE 4

/* This value is a constant offset added to every bit period to allow for the timer IRQ handler to execute. This is set
 * empirically using a debugger and a logic analyzer. */
#define TIMER_CYCLES_FOR_SPI_TRANSMISSIONS 120

/* This is the same as above, but for the reset cycle of the bit period. */
#define RESET_PERIOD_LENGTH 40

/* Defines for brevity */
#define A TIMER_CYCLES_FOR_SPI_TRANSMISSIONS
#define B PERIOD_BASE

/* This is a constant offset containing some empirically determined correction values */
#define C (1 /* reset pulse comp */ - 3 /* analog snafu comp */)

/* This lookup table maps bit positions to timer period values. This is a lookup table to allow for the compensation for
 * non-linear effects of ringing at lower bit durations.
 */
static uint16_t timer_period_lookup[NBITS] = {
    /* LSB here */
    A - C + (B<< 0),
    A - C + (B<< 1),
    A - C + (B<< 2),
    A - C + (B<< 3),
    A - C + (B<< 4),
    A - C + (B<< 5),
    A - C + (B<< 6),
    A - C + (B<< 7),
    A - C + (B<< 8),
    A - C + (B<< 9),
    A - C + (B<<10),
    A - C + (B<<11),
    A - C + (B<<12),
    A - C + (B<<13),
    /* MSB here */
};

/* Don't pollute the global namespace */
#undef A
#undef B
#undef C

/* Timer 1 main IRQ handler. This is used only for overflow ("update" or UP event in ST's terminology). */
void TIM1_BRK_UP_TRG_COM_IRQHandler(void) {
    /* The index of the currently active bit. On entry of this function, this is the bit index of the upcoming period.
     * On exit it is the index of the *next* period. */
    static int idx = 0;
    /* We modulate all outputs simultaneously in n periods, with n being the modulation depth (the number of bits).
     * Each period is split into two timer cycles. First, a long one during which the data for the current period is
     * shifted out and subsequently latched to the outputs. Then, a short one that is used to reset all outputs in time
     * for the next period.
     *
     *  bit value: | <-- least significant, shortest period / most significant, longest period --> |
     * bit number: |                             b0             | b1  |      ...       | b10 | b11 |
     *       name: |          data cycle          | reset cycle |     |                |     |     |
     *   function: | shift data <strobe>     wait |             | ... |      ...       | ... | ... |
     *   duration: | fixed               variable |    fixed    |     |                |     |     |
     *
     * Now, alternate between the two cycles in one phase.
     */
    static int clear = 0;
    if ((clear = !clear)) {
        /* Access bits offset by one as we are setting the *next* period based on idx below. */
        uint32_t val = brightness_by_bit[idx];

        /* Shift out the current period's data. The shift register clear and strobe lines are handled by the timers
         * capture/compare channel 3 complementary outputs. The dead-time generator is used to sequence the clear and
         * strobe edges one after another. Since there may be small variations in IRQ service latency it is critical to
         * allow for some leeway between the end of this data transmission and strobe and clear. */
        SPI1->DR = (val&0xffff);
        while (SPI1->SR & SPI_SR_BSY);
        SPI1->DR = (val>>16);
        while (SPI1->SR & SPI_SR_BSY);

        /* Increment the bit index for the next cycle */
        idx++;
        if (idx >= NBITS)
            idx = 0;

        /* Set up the following reset pulse cycle. This cycle is short as it only needs to be long enough for the below
         * part of this ISR handler routine to run. */
        TIM1->ARR = RESET_PERIOD_LENGTH;
        TIM1->CCR3 = 1; /* This value is fixed to produce a very short reset pulse. IOs, PCB and shift registers all can
                           easily handle this. */
    } else {
        /* Set up everything for the data cycle of the *next* period. The timer is set to count from 0 to ARR. ARR and
         * CCR3 are pre-loaded, so the values written above will only be latched on timer overrun at the end of this
         * period. This is a little complicated, but doing it this way has the advantage of keeping both duty cycle and
         * frame rate precisely constant. */
        TIM1->CCR3 = TIMER_CYCLES_FOR_SPI_TRANSMISSIONS;
        TIM1->ARR = timer_period_lookup[idx];
    }
    /* Reset the update interrupt flag. This ISR handler routine is only used for timer update events. */
    TIM1->SR &= ~TIM_SR_UIF_Msk;
}

/* The data format of the serial command interface.
 *
 * The serial interface uses short packets. Currently, there is only one packet type defined: a "set RGBW" packet, using
 * command ID 0x23. The packet starts with the command ID, followed by the addressed channel group, followed by four
 * times two bytes of big-endian RGBW channel data.
 *
 *
 */
union packet {
    struct {
        uint8_t cmd; /* 0x23 */
        uint8_t step; /* logical channel. The USART_CHANNEL_OFFX is applied on this number below. */
        union {
            uint16_t rgbw[4];
            struct {
                uint16_t r, g, b, w;
            };
        };
    } set_step;
    uint8_t data[0];
};

int rxpos = 0;
void TIM3_IRQHandler(void) {
    TIM3->SR &= ~TIM_SR_UIF;
    /*
    if (rxpos != sizeof(union packet)) {
        asm("bkpt");
    }
    */
    rxpos = 0;
}

/* Misc IRQ handlers */
void NMI_Handler(void) {
}

void HardFault_Handler(void) {
    for(;;);
}

void SVC_Handler(void) {
}


void PendSV_Handler(void) {
}

void SysTick_Handler(void) {
    static int n = 0;
    sys_time++;
    if (n++ == 1000) {
        n = 0;
        sys_time_seconds++;
    }
}

/* Misc stuff for nostdlib linking */
void _exit(int status) { while (23); }
void *__bss_start__;
void *__bss_end__;
int __errno;

