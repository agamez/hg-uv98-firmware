/* UART drivers — see uart.h for address/provenance annotations. */
#include "uart.h"
#include "hwregs.h"
#include "protocol.h"

/* ---- USART2: APRS coprocessor / BT-app link (RECOVERED) ------------------ */

static uint8_t  u2_ring[64];          /* original reassembles frames directly */
static volatile uint32_t u2_idx;

void usart2_init(void)
{
    RCC_APB1ENR |= RCC_APB1ENR_USART2EN;
    USART_CR1(USART2_BASE) = 0;                       /* disable while configuring */

    /* GPIOA: PA2 = TX (mode 3 = output 50 MHz, AF push-pull -> CR21 bits),
     *         PA3 = RX (mode 4 = input floating -> CR2 bits). Original
     *         writes CR2/CR1 masks 0xCFFF/0xE9F3 via the SPL; we set the
     *         same pin fields directly in CRL. */
    GPIO_CRL(GPIOA_BASE) &= ~0x0000FF00u;
    GPIO_CRL(GPIOA_BASE) |=  0x00004B00u;   /* PA2: 0xB (AF-PP 50MHz), PA3: 0x4 (floating) */

    /* NVIC: IRQ 38, priority 3 (original NVIC_Init pre=3 sub=3). */
    REG8(NVIC_IP_BASE + IRQ_USART2) = (3 << 4) & 0xFF;

    USART_BRR(USART2_BASE) = 0x2580;                  /* 9600 baud at 72/108 MHz per SPL */
    USART_CR1(USART2_BASE) = USART_CR1_TE | USART_CR1_RE | USART_CR1_RXNEIE;
    USART_CR2(USART2_BASE) = 0;
    USART_CR3(USART2_BASE) = 0;
    USART_CR1(USART2_BASE) |= USART_CR1_UE;

    u2_idx = 0;
    NVIC_ISER1 = (1u << (IRQ_USART2 - 32u));
}

void usart2_send(const uint8_t *buf, uint32_t len)
{
    /* [0x0801FEAA | RECOVERED]: poll TC then write DR, byte per iteration. */
    for (uint32_t i = 0; i < len; i++) {
        while (!(USART_SR(USART2_BASE) & USART_SR_TC))
            ;
        USART_DR(USART2_BASE) = buf[i] & 0x1FF;
    }
}

/* [0x0801F920 | RECOVERED]: RXNE ISR; accumulate into ring; on '\r\n'
 * require buf[0]=='$' and post the frame to the protocol layer. */
void USART2_IRQHandler(void)
{
    uint8_t b;

    if (!(USART_SR(USART2_BASE) & USART_SR_RXNE))
        return;                                       /* original: GetITStatus gate */

    b = (uint8_t)USART_DR(USART2_BASE);
    if (u2_idx < sizeof(u2_ring))
        u2_ring[u2_idx++] = b;

    if (u2_idx >= 3 &&
        u2_ring[u2_idx - 2] == '\r' && u2_ring[u2_idx - 1] == '\n') {
        if (u2_ring[0] == '$') {
            protocol_frame_received(u2_ring, u2_idx);
        }
        /* non-'$' frames: original clears the ready flag and resets
         * (app-text commands are documented as unreachable in the June
         * build behind the '$' gate — research log session 3). */
        u2_idx = 0;
    }
}

/* ---- USART1: GPS input (RECOVERED) --------------------------------------- */

static void (*u1_sink)(uint8_t);
static volatile uint32_t u1_widx;

void usart1_set_sink(void (*sink)(uint8_t byte))
{
    u1_sink = sink;
}

void usart1_init(void)
{
    RCC_APB2ENR |= RCC_APB2ENR_USART1EN;
    USART_CR1(USART1_BASE) = 0;

    /* GPIOA: PA9 = TX (0xB AF-PP), PA10 = RX (0x4 floating) in CRH. */
    GPIO_CRH(GPIOA_BASE) &= ~0x00000FF0u;
    GPIO_CRH(GPIOA_BASE) |=  0x000004B0u;

    REG8(NVIC_IP_BASE + IRQ_USART1) = (3 << 4) & 0xFF;

    USART_BRR(USART1_BASE) = 0x2580;                  /* 9600 baud */
    USART_CR1(USART1_BASE) = USART_CR1_TE | USART_CR1_RE | USART_CR1_RXNEIE;
    USART_CR1(USART1_BASE) |= USART_CR1_UE;

    u1_widx = 0;
    NVIC_ISER1 = (1u << (IRQ_USART1 - 32u));
}

void usart1_send(const uint8_t *buf, uint32_t len)
{
    for (uint32_t i = 0; i < len; i++) {
        while (!(USART_SR(USART1_BASE) & USART_SR_TC))
            ;
        USART_DR(USART1_BASE) = buf[i] & 0x1FF;
    }
}

/* [0x080292F8 | RECOVERED]: ring-buffer writer; original also posts each
 * byte to a mailbox consumed by a pump task (0x200050DC). Here the byte
 * goes straight to the registered sink (same data flow, one hop less). */
void USART1_IRQHandler(void)
{
    if (!(USART_SR(USART1_BASE) & USART_SR_RXNE))
        return;
    uint8_t b = (uint8_t)USART_DR(USART1_BASE);
    if (u1_sink)
        u1_sink(b);
    u1_widx = (u1_widx + 1) & 0xFFFF;                 /* original wraps a u16 index */
}
