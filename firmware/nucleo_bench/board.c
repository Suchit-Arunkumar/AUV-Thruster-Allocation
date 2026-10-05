/*
 * board.c - 180 MHz clock, USART2 TX and the DWT cycle counter for the
 * NUCLEO-F446RE. Register sequences taken from Nucleo_AUV_Bare_Metal
 * (Core/Src/system_init.c, Drivers/uart/uart.c, Core/Src/bench.c).
 */
#include "board.h"

#define APB1_HZ   45000000U
#define UART_BAUD 115200U

void board_clock_180mhz(void)
{
    /* HSE bypass: 8 MHz from the ST-Link MCO */
    RCC->CR |= RCC_CR_HSEBYP;
    RCC->CR |= RCC_CR_HSEON;
    while (!(RCC->CR & RCC_CR_HSERDY)) {}

    RCC->APB1ENR |= RCC_APB1ENR_PWREN;

    /* 5 wait states at 180 MHz, ART accelerator on */
    FLASH->ACR = FLASH_ACR_ICEN | FLASH_ACR_DCEN | FLASH_ACR_PRFTEN |
                 FLASH_ACR_LATENCY_5WS;

    /* HSE 8 MHz / M 8 * N 360 / P 2 = 180 MHz */
    RCC->PLLCFGR &= ~(RCC_PLLCFGR_PLLM | RCC_PLLCFGR_PLLN |
                      RCC_PLLCFGR_PLLP | RCC_PLLCFGR_PLLSRC);
    RCC->PLLCFGR |=  (8U   << RCC_PLLCFGR_PLLM_Pos) |
                     (360U << RCC_PLLCFGR_PLLN_Pos) |
                     (0U   << RCC_PLLCFGR_PLLP_Pos) |
                     RCC_PLLCFGR_PLLSRC_HSE;
    RCC->CR |= RCC_CR_PLLON;
    while (!(RCC->CR & RCC_CR_PLLRDY)) {}

    /* Over-drive, required above 168 MHz */
    PWR->CR |= PWR_CR_ODEN;
    while (!(PWR->CSR & PWR_CSR_ODRDY)) {}
    PWR->CR |= PWR_CR_ODSWEN;
    while (!(PWR->CSR & PWR_CSR_ODSWRDY)) {}

    /* APB1 45 MHz, APB2 90 MHz, then switch SYSCLK to the PLL */
    RCC->CFGR &= ~(RCC_CFGR_PPRE1 | RCC_CFGR_PPRE2);
    RCC->CFGR |= RCC_CFGR_PPRE1_DIV4 | RCC_CFGR_PPRE2_DIV2;
    RCC->CFGR &= ~RCC_CFGR_SW;
    RCC->CFGR |= RCC_CFGR_SW_PLL;
    while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL) {}
}

void board_uart2_init(void)
{
    /* PA2 = USART2_TX (AF7), routed to the ST-Link virtual COM port */
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
    GPIOA->MODER  &= ~(3U << (2 * 2));
    GPIOA->MODER  |=  (2U << (2 * 2));
    GPIOA->AFR[0] &= ~(0xFU << 8);
    GPIOA->AFR[0] |=  (7U << 8);

    RCC->APB1ENR |= RCC_APB1ENR_USART2EN;
    USART2->BRR = (APB1_HZ + UART_BAUD / 2U) / UART_BAUD;
    USART2->CR1 |= USART_CR1_TE | USART_CR1_UE;
}

void board_puts(const char *s)
{
    while (*s) {
        while (!(USART2->SR & USART_SR_TXE)) {}
        USART2->DR = (uint8_t)*s++;
    }
    while (!(USART2->SR & USART_SR_TC)) {}
}

void board_dwt_init(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL  |= DWT_CTRL_CYCCNTENA_Msk;
}

void board_delay_cycles(uint32_t n)
{
    uint32_t t0 = DWT->CYCCNT;
    while ((DWT->CYCCNT - t0) < n) {}
}

/* Linked with -nostartfiles (startup_stm32f446retx.s is the entry point), so
 * crti's _init, called by __libc_init_array, has to be provided. */
void _init(void) {}
