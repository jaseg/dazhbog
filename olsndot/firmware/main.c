
#include <stm32f0xx.h>
#include <stdint.h>
#include <system_stm32f0xx.h>
#include <stm32f0xx_ll_utils.h>
/* 
 * Part number: STM32F030F4C6
 */

#define NBITS 12
void do_transpose(void);
uint32_t brightness[8];
volatile uint8_t brightness_by_bit[NBITS];

int main(void) {
    RCC->CR |= RCC_CR_HSEON;
    while (!(RCC->CR&RCC_CR_HSERDY));
    RCC->CFGR &= ~RCC_CFGR_PLLMUL_Msk & ~RCC_CFGR_SW_Msk;
    RCC->CFGR |= (2<<RCC_CFGR_PLLMUL_Pos) | RCC_CFGR_PLLSRC; /* PLL x4 */
    RCC->CR |= RCC_CR_PLLON;
    while (!(RCC->CR&RCC_CR_PLLRDY));
    RCC->CFGR |= (2<<RCC_CFGR_SW_Pos);
    SystemCoreClockUpdate();

    LL_Init1msTick(SystemCoreClock);

    RCC->AHBENR  |= RCC_AHBENR_GPIOAEN | RCC_AHBENR_GPIOBEN;
    RCC->APB2ENR |= RCC_APB2ENR_SPI1EN | RCC_APB2ENR_TIM1EN | RCC_APB2ENR_USART1EN | RCC_APB2ENR_ADCEN;

    GPIOA->MODER |=
          (3<<GPIO_MODER_MODER0_Pos)  /* PA0  - Current measurement analog input */
        | (1<<GPIO_MODER_MODER1_Pos)  /* PA1  - RS485 TX enable */
        | (2<<GPIO_MODER_MODER2_Pos)  /* PA2  - RS485 TX */
        | (2<<GPIO_MODER_MODER3_Pos)  /* PA3  - RS485 RX */
        | (1<<GPIO_MODER_MODER4_Pos)  /* PA4  - LED1 open-drain output */
        | (2<<GPIO_MODER_MODER5_Pos)  /* PA5  - Shift register clk/SCLK */
        | (1<<GPIO_MODER_MODER6_Pos)  /* PA6  - LED2 open-drain output */
        | (2<<GPIO_MODER_MODER7_Pos)  /* PA7  - Shift register data/MOSI */
        | (2<<GPIO_MODER_MODER9_Pos)  /* PA9  - Shift register clear (TIM1_CH2) */
        | (2<<GPIO_MODER_MODER10_Pos);/* PA10 - Shift register strobe (TIM1_CH3) */
    GPIOB->MODER |=
          (1<<GPIO_MODER_MODER1_Pos); /* PB1  - Current measurement range selection */


    GPIOA->OTYPER |= GPIO_OTYPER_OT_6 | GPIO_OTYPER_OT_4; /* LED outputs -> open drain */

    /* Set shift register IO GPIO output speed */
    GPIOA->OSPEEDR |=
          (3<<GPIO_OSPEEDR_OSPEEDR4_Pos)  /* LED1   */
        | (3<<GPIO_OSPEEDR_OSPEEDR5_Pos)  /* SCLK   */
        | (3<<GPIO_OSPEEDR_OSPEEDR6_Pos)  /* LED2   */
        | (3<<GPIO_OSPEEDR_OSPEEDR7_Pos)  /* MOSI   */
        | (3<<GPIO_OSPEEDR_OSPEEDR9_Pos)  /* Clear */
        | (3<<GPIO_OSPEEDR_OSPEEDR10_Pos);/* Strobe */

    GPIOA->AFR[0] |=
          (1<<GPIO_AFRL_AFRL2_Pos)   /* USART1_TX */
        | (1<<GPIO_AFRL_AFRL3_Pos)   /* USART1_RX */
        | (0<<GPIO_AFRL_AFRL5_Pos)   /* SPI1_SCK  */
        | (0<<GPIO_AFRL_AFRL7_Pos);  /* SPI1_MOSI */
    GPIOA->AFR[1] |=
          (2<<GPIO_AFRH_AFRH1_Pos)   /* TIM1_CH2 */
        | (2<<GPIO_AFRH_AFRH2_Pos);  /* TIM1_CH3  */
    
    GPIOB->BSRR = GPIO_BSRR_BR_1; /* clear output is active low */

    /* Configure SPI controller */
    /* CPOL=0, CPHA=0, prescaler=8 -> 1MBd */
//    SPI1->CR1 = SPI_CR1_BIDIMODE | SPI_CR1_BIDIOE | SPI_CR1_SSM | SPI_CR1_SSI | SPI_CR1_SPE | (2<<SPI_CR1_BR_Pos) | SPI_CR1_MSTR;
    SPI1->CR1 = SPI_CR1_BIDIMODE | SPI_CR1_BIDIOE | SPI_CR1_SSM | SPI_CR1_SSI | SPI_CR1_SPE | (0<<SPI_CR1_BR_Pos) | SPI_CR1_MSTR;
    SPI1->CR2 = (7<<SPI_CR2_DS_Pos);
    /* Configure TIM1 for display strobe generation */
    /* Configure UART for RS485 comm */
    /* 8N1, 115200Bd */
//    TIM1->CR1 = TIM_CR1_OPM | TIM_CR1_URS;
//    TIM1->CR1 = TIM_CR1_ARPE | TIM_CR1_URS;
    TIM1->CR1 = TIM_CR1_ARPE | TIM_CR1_OPM; // | TIM_CR1_URS;
    TIM1->CR2 = 0; //TIM_CR2_CCPC;
    TIM1->SMCR = 0;
    TIM1->DIER = 0;

    TIM1->PSC = 1; // debug
    /* CH2 - clear/!MR, CH3 - strobe/STCP */
    TIM1->CCR2 = 1;
    TIM1->RCR = 0;
    TIM1->CCMR1 = (6<<TIM_CCMR1_OC2M_Pos); // | TIM_CCMR1_OC2PE;
    TIM1->CCMR2 = (6<<TIM_CCMR2_OC3M_Pos); // | TIM_CCMR2_OC3PE;
    TIM1->CCER |= TIM_CCER_CC2E | TIM_CCER_CC2NE | TIM_CCER_CC2P | TIM_CCER_CC3E;
//    TIM1->CCMR1 = (6<<TIM_CCMR1_OC2M_Pos) | TIM_CCMR1_OC2PE;
//    TIM1->CCMR2 = (6<<TIM_CCMR2_OC3M_Pos) | TIM_CCMR2_OC3PE;
//    TIM1->CCER = TIM_CCER_CC2E | TIM_CCER_CC3E;
//    TIM1->BDTR = TIM_BDTR_MOE;
    TIM1->DIER = TIM_DIER_UIE;

    NVIC_EnableIRQ(TIM1_BRK_UP_TRG_COM_IRQn);
    NVIC_SetPriority(TIM1_BRK_UP_TRG_COM_IRQn, 2);

    TIM1->EGR |= TIM_EGR_UG;

    while (42) {
        /*
        for (uint8_t i=0; i<8; i++) {
            brightness[1] = brightness[5] = i;
            brightness[2] = brightness[6] = 0;
            brightness[3] = brightness[7] = 0;
            do_transpose();
            LL_mDelay(500);
        }
        for (uint8_t i=0; i<8; i++) {
            brightness[1] = brightness[5] = 0;
            brightness[2] = brightness[6] = i;
            brightness[3] = brightness[7] = 0;
            do_transpose();
            LL_mDelay(500);
        }
        for (uint8_t i=0; i<8; i++) {
            brightness[1] = brightness[5] = 0;
            brightness[2] = brightness[6] = 0;
            brightness[3] = brightness[7] = i;
            do_transpose();
            LL_mDelay(500);
        }
        for (uint8_t i=0; i<8; i++) {
            brightness[1] = brightness[5] = i;
            brightness[2] = brightness[6] = i;
            brightness[3] = brightness[7] = i;
            do_transpose();
            LL_mDelay(500);
        }
    }
    {
    */
        for (uint32_t i=0; i<6; i++) {
            for (uint32_t j=0; j<(1<<NBITS); j++) {
                GPIOA->ODR ^= GPIO_ODR_6;
                switch (i) {
                    case 0:
                    brightness[1] = brightness[5] = (1<<NBITS)-1;
                    brightness[2] = brightness[6] = 0;
                    brightness[3] = brightness[7] = j;
                    break;
                    case 1:
                    brightness[1] = brightness[5] = (1<<NBITS)-1-j;
                    brightness[2] = brightness[6] = 0;
                    brightness[3] = brightness[7] = (1<<NBITS)-1;
                    break;
                    case 2:
                    brightness[1] = brightness[5] = 0;
                    brightness[2] = brightness[6] = j;
                    brightness[3] = brightness[7] = (1<<NBITS)-1;
                    break;
                    case 3:
                    brightness[1] = brightness[5] = 0;
                    brightness[2] = brightness[6] = (1<<NBITS)-1;
                    brightness[3] = brightness[7] = (1<<NBITS)-1-j;
                    break;
                    case 4:
                    brightness[1] = brightness[5] = j;
                    brightness[2] = brightness[6] = (1<<NBITS)-1;
                    brightness[3] = brightness[7] = 0;
                    break;
                    case 5:
                    brightness[1] = brightness[5] = (1<<NBITS)-1;
                    brightness[2] = brightness[6] = (1<<NBITS)-1-j;
                    brightness[3] = brightness[7] = 0;
                    break;
                }
                do_transpose();
                LL_mDelay(1);
            }
        }
    }
}

uint32_t brightness[8] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
volatile uint8_t brightness_by_bit[NBITS] = { 0 };

void do_transpose(void) {
    for (uint32_t i=0; i<NBITS; i++) {
        uint8_t bv = 0;
        uint32_t mask = 1<<i;
        for (uint32_t j=0; j<8; j++) {
            if (brightness[j] & mask)
            bv |= 1<<j;
        }
        brightness_by_bit[i] = bv;
    }
}

/*
 * 460ns
 * 720ns
 */

/*
 * 1.00us
 * 1.64us
 * 2.84us
 * 5.36us
 * 10.4us
 * 20.4us
 * 40.4us
 * 80.8us
 */
void TIM1_BRK_UP_TRG_COM_IRQHandler(void) {
    static uint32_t idx = 0;
    idx++;
    if (idx >= NBITS)
        idx = 0;

    GPIOA->ODR ^= GPIO_ODR_4;
    TIM1->CCMR1 = (4<<TIM_CCMR1_OC2M_Pos); // | TIM_CCMR1_OC2PE;

    SPI1->DR = brightness_by_bit[idx]<<8;
    while (SPI1->SR & SPI_SR_BSY);

    const uint32_t period_base = 4; /* 1us */
    const uint32_t period = period_base<<idx;
//    TIM1->BDTR  = TIM_BDTR_MOE | (16<<TIM_BDTR_DTG_Pos);
    TIM1->BDTR = TIM_BDTR_MOE | (0<<TIM_BDTR_DTG_Pos);
    TIM1->CCR3 = period-1;
    TIM1->CNT = period-1;
    TIM1->ARR = period;
    TIM1->CCMR1 = (6<<TIM_CCMR1_OC2M_Pos); // | TIM_CCMR1_OC2PE;
    TIM1->EGR |= TIM_EGR_UG;
    TIM1->ARR = 2;
    TIM1->CR1 |= TIM_CR1_CEN;
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

