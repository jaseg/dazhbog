
#include <stm32f0xx.h>
#include <stdint.h>
#include <system_stm32f0xx.h>
#include <stm32f0xx_ll_utils.h>
#include <math.h>
/* 
 * Part number: STM32F030F4C6
 */

#define NBITS 12
void do_transpose(void);
uint32_t brightness[32];
volatile uint32_t brightness_by_bit[NBITS];

void hsv_set(int idx, int hue, int white) {
    int i = hue>>NBITS;
    int j = hue & (~(-1<<NBITS));
    int r=0, g=0, b=0;
    switch (i) {
        case 0:
        r = (1<<NBITS)-1;
        g = 0;
        b = j;
        break;
        case 1:
        r = (1<<NBITS)-1-j;
        g = 0;
        b = (1<<NBITS)-1;
        break;
        case 2:
        r = 0;
        g = j;
        b = (1<<NBITS)-1;
        break;
        case 3:
        r = 0;
        g = (1<<NBITS)-1;
        b = (1<<NBITS)-1-j;
        break;
        case 4:
        r = j;
        g = (1<<NBITS)-1;
        b = 0;
        break;
        case 5:
        r = (1<<NBITS)-1;
        g = (1<<NBITS)-1-j;
        b = 0;
        break;
    }
    brightness[idx*4 + 0] = white;
    brightness[idx*4 + 1] = r;
    brightness[idx*4 + 2] = g;
    brightness[idx*4 + 3] = b;
}

int hue;
int main(void) {
    RCC->CR |= RCC_CR_HSEON;
    while (!(RCC->CR&RCC_CR_HSERDY));
    RCC->CFGR &= ~RCC_CFGR_PLLMUL_Msk & ~RCC_CFGR_SW_Msk & ~RCC_CFGR_PPRE_Msk & ~RCC_CFGR_HPRE_Msk;
    RCC->CFGR |= (2<<RCC_CFGR_PLLMUL_Pos) | RCC_CFGR_PLLSRC_HSE_PREDIV; /* PLL x4 -> 50.0MHz */
    RCC->CFGR2 &= ~RCC_CFGR2_PREDIV_Msk;
    RCC->CFGR2 |= RCC_CFGR2_PREDIV_DIV2; /* prediv :2 -> 12.5MHz */
    RCC->CR |= RCC_CR_PLLON;
    while (!(RCC->CR&RCC_CR_PLLRDY));
    RCC->CFGR |= (2<<RCC_CFGR_SW_Pos);
    SystemCoreClockUpdate();



    RCC->AHBENR  |= RCC_AHBENR_GPIOAEN | RCC_AHBENR_GPIOBEN;
    RCC->APB2ENR |= RCC_APB2ENR_SPI1EN | RCC_APB2ENR_TIM1EN | RCC_APB2ENR_USART1EN | RCC_APB2ENR_ADCEN;

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
    /* CPOL=0, CPHA=0, prescaler=8 -> 1MBd */
    SPI1->CR1 = SPI_CR1_BIDIMODE | SPI_CR1_BIDIOE | SPI_CR1_SSM | SPI_CR1_SSI | SPI_CR1_SPE | (0<<SPI_CR1_BR_Pos) | SPI_CR1_MSTR;
    SPI1->CR2 = (0xf<<SPI_CR2_DS_Pos);
    /* Configure TIM1 for display strobe generation */
    /* Configure UART for RS485 comm */
    /* 8N1, 115200Bd */
    TIM1->CR1 = TIM_CR1_ARPE; // | TIM_CR1_OPM; // | TIM_CR1_URS;

    TIM1->PSC = 1; // debug
    /* CH2 - clear/!MR, CH3 - strobe/STCP */
    TIM1->CCMR2 = (6<<TIM_CCMR2_OC3M_Pos); // | TIM_CCMR2_OC3PE;
    TIM1->CCER |= TIM_CCER_CC3E | TIM_CCER_CC3NE | TIM_CCER_CC3P | TIM_CCER_CC3NP;
    TIM1->BDTR = TIM_BDTR_MOE | (8<<TIM_BDTR_DTG_Pos); /* 1us dead time */
    TIM1->DIER = TIM_DIER_UIE;
    TIM1->ARR = 1;
    TIM1->CR1 |= TIM_CR1_CEN;

    NVIC_EnableIRQ(TIM1_BRK_UP_TRG_COM_IRQn);
    NVIC_SetPriority(TIM1_BRK_UP_TRG_COM_IRQn, 2);

    TIM1->EGR |= TIM_EGR_UG;

    while (42) {
#define HUE_MAX ((1<<NBITS)*6)
#define HUE_OFFX 0.15F /* 0-1 */
#define HUE_AMPLITUDE 0.05F /* 0-1 */
#define CHANNEL_SPACING 1.5F /* in radians */
#define WHITE 0.2F /* 0-1 */
        for (float v=0; v<8*M_PI; v += 0.01F) {
            GPIOA->ODR ^= GPIO_ODR_6;
            /* generate hsv fade */
            for (int ch=0; ch<8; ch++) {
                hue = HUE_MAX * (HUE_OFFX + HUE_AMPLITUDE*sinf(v + ch*CHANNEL_SPACING));
                hue %= HUE_MAX;
                hsv_set(ch, hue, WHITE*(1<<NBITS));
            }
            do_transpose();
            for (int k=0; k<10000; k++) {
                asm volatile("nop");
            }
        }
    }
}

uint32_t brightness[32] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
volatile uint32_t brightness_by_bit[NBITS] = { 0 };

void do_transpose(void) {
    for (uint32_t i=0; i<NBITS; i++) {
        uint32_t bv = 0;
        uint32_t mask = 1<<i;
        for (uint32_t j=0; j<32; j++) {
            if (brightness[j] & mask)
            bv |= 1<<j;
        }
        brightness_by_bit[i] = bv;
    }
}

void TIM1_BRK_UP_TRG_COM_IRQHandler(void) {
    /* The index of the currently active bit. On entry of this function, this is the bit index of the upcoming period.
     * On exit it is the index of the *next* period. */
    static uint32_t idx = 0;

    /* Access bits offset by one as we are setting the *next* period based on idx below. */
    uint32_t val = brightness_by_bit[idx];

    idx++;
    if (idx >= NBITS)
        idx = 0;

    GPIOA->ODR ^= GPIO_ODR_6; /* LED1 */

    /* Shift out the current period's data. The shift register clear and strobe lines are handled by the timers
     * capture/compare channel 3 complementary outputs. The dead-time generator is used to sequence the clear and strobe
     * edges one after another. Since there may be small variations in IRQ service latency it is critical to allow for
     * some leeway between the end of this data transmission and strobe and clear. */
    SPI1->DR = (val&0xffff);
    while (SPI1->SR & SPI_SR_BSY);
    SPI1->DR = (val>>16);
    while (SPI1->SR & SPI_SR_BSY);

    /* Set up everything for the *next* period. The timer is set to count from 0 to ARR. ARR and CCR3 are pre-loaded, so
     * the values written above will only be latched on timer overrun at the end of this period. This is a little
     * complicated, but doing it this way has the advantage of keeping both duty cycle and frame rate precisely
     * constant. */
    const int period_base = 4; /* 1us */
    const int period = (period_base<<idx) + 4 /* 1us dead time */;
    const int timer_cycles_for_spi_transmissions = 128;
    TIM1->ARR = period + timer_cycles_for_spi_transmissions;
    TIM1->CCR3 = timer_cycles_for_spi_transmissions;
    TIM1->SR &= ~TIM_SR_UIF_Msk;
}

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
}

/* FIXME */
void _exit(void) {}
void *__bss_start__;
void *__bss_end__;

int __errno;
