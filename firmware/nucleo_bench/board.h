#ifndef BOARD_H
#define BOARD_H

#include "stm32f4xx.h"
#include <stdint.h>

#define BOARD_SYSCLK_HZ 180000000U

void board_clock_180mhz(void);
void board_uart2_init(void);
void board_puts(const char *s);
void board_dwt_init(void);
void board_delay_cycles(uint32_t n);

static inline uint32_t board_cycles(void) { return DWT->CYCCNT; }

#endif /* BOARD_H */
