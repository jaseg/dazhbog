
#include <stm32f0xx.h>
#include <stdint.h>
#include <system_stm32f0xx.h>
#include <stm32f0xx_ll_utils.h>
/* 
 * Part number: STM32F030F4C6
 */

int main(void) {
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
          (1<<GPIO_OSPEEDR_OSPEEDR5_Pos)  /* SCLK   */
        | (1<<GPIO_OSPEEDR_OSPEEDR7_Pos)  /* MOSI   */
        | (1<<GPIO_OSPEEDR_OSPEEDR9_Pos)  /* Clear */
        | (1<<GPIO_OSPEEDR_OSPEEDR10_Pos);/* Strobe */

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
    SPI1->CR1 = SPI_CR1_BIDIMODE | SPI_CR1_BIDIOE | SPI_CR1_SSM | SPI_CR1_SSI | SPI_CR1_SPE | (7<<SPI_CR1_BR_Pos) | SPI_CR1_MSTR;
    SPI1->CR2 = (7<<SPI_CR2_DS_Pos);
    /* Configure TIM1 for display strobe generation */
    /* Configure UART for RS485 comm */
    /* 8N1, 115200Bd */
    TIM1->CR1 = TIM_CR1_OPM | TIM_CR1_ARPE | TIM_CR1_URS;
    TIM1->PSC = 256; // debug
    TIM1->CCMR1 = (6<<TIM_CCMR1_OC2M_Pos) | TIM_CCMR1_OC2PE;
    TIM1->CCMR2 = (6<<TIM_CCMR2_OC3M_Pos) | TIM_CCMR2_OC3PE;
    TIM1->CCER = TIM_CCER_CC2E | TIM_CCER_CC3E;
    TIM1->BDTR = TIM_BDTR_MOE;
    TIM1->RCR = 2;
    TIM1->DIER = TIM_DIER_UIE;

    NVIC_EnableIRQ(TIM1_BRK_UP_TRG_COM_IRQn);
    NVIC_SetPriority(TIM1_BRK_UP_TRG_COM_IRQn, 2);

    TIM1->ARR = 1024;
    TIM1->EGR |= TIM_EGR_UG;
    TIM1->CR1 |= TIM_CR1_CEN;

    for (;;) {
    }
}

#define NBITS 4
uint8_t brightness_by_bit[NBITS] = {
    0x11, 0x22, 0x44, 0x88
};

void TIM1_BRK_UP_TRG_COM_IRQHandler(void) {
    static uint32_t bitpos = 0;
    bitpos = (bitpos+1)&(NBITS-1);

//    SPI1->DR = ((uint32_t)brightness_by_bit[bitpos])<<8;
    SPI1->DR = (bitpos<<8) | (bitpos<<10) | (bitpos<<12) | (bitpos<<14);
    while (SPI1->SR & SPI_SR_BSY);

    const uint32_t cycles_strobe = 2;
    const uint32_t cycles_clear = 2;
    const uint32_t base_val = 16;
    uint32_t period = base_val<<bitpos;

//    TIM1->ARR = period;
    TIM1->ARR = 1024;
    TIM1->CCR3 = cycles_strobe; /* strobe */
    TIM1->CCR2 = period-cycles_clear; /* clear */
    TIM1->EGR |= TIM_EGR_UG;
//    TIM1->ARR = cycles_strobe+1;
//    LL_mDelay(1);
//    TIM1->CR1 |= TIM_CR1_CEN;
//    GPIOA->BSRR = GPIO_BSRR_BR_4 | GPIO_BSRR_BS_6;
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

