/* UART drivers — reconstruction of the recovered original drivers.
 *
 * USART2 (APRS coprocessor / Bluetooth-app link):
 *   [addr 0x0801F810 | RECOVERED, high confidence]
 *   Evidence: SPL-style init sequence (RCC_APB1ENR bit17, GPIOA CR config
 *   PA2 AF-pushpull 50 MHz / PA3 input-floating, NVIC IRQ 38 prio 3/3,
 *   BRR 0x2580 = 9600 baud, CR1 mode TX|RX, RXNE interrupt enabled,
 *   300 ms settle delay), recovered by direct disassembly.
 *
 * USART1 (GPS input, probable):
 *   [addr 0x0802932C | RECOVERED, high confidence]
 *   Evidence: same SPL pattern with RCC_APB2ENR bit14, GPIOA PA9/PA10,
 *   NVIC IRQ 37, BRR 0x2580 = 9600 baud, RXNE interrupt.
 *
 * TX path  [0x0801FEAA / 0x080293F8 | RECOVERED]: poll Transmit-Complete
 * flag per byte, write DR.
 *
 * RX ISR   [0x0801F920 / 0x080292F8 | RECOVERED]: on RXNE read DR into a
 * ring buffer; USART2 additionally reassembles CRLF-terminated '$'-frames
 * and signals readiness via a flag byte (0x20005094+9 bit0 in the
 * original; mailbox post in this reconstruction).
 */
#ifndef UART_H
#define UART_H

#include <stdint.h>

void usart2_init(void);

extern uint8_t u2_rx_open;   /* [session 71] the request-gated RX window */
uint8_t u2_rx_poll(void);    /* a complete '$'-frame is held in the ring  */
void u2_rx_drain(void);      /* route the first held frame to the router  */

void usart2_send(const uint8_t *buf, uint32_t len);
void usart1_init(void);

void usart2_send(const uint8_t *buf, uint32_t len);
void usart1_send(const uint8_t *buf, uint32_t len);

/* RX ISR entry points (wired in the vector table). */
void USART2_IRQHandler(void);
void USART1_IRQHandler(void);

/* USART1 byte pump: the original posts received bytes to a mailbox that a
 * dedicated task consumes (0x200050DC handle; task region 0x0802942A). */
void usart1_set_sink(void (*sink)(uint8_t byte));

#endif /* UART_H */
