/* Inter-MCU protocol layer (USART2, 9600 8N1, '$'-prefixed CRLF frames).
 *
 * RECOVERED from three cross-verified sources (research log sessions 3-4):
 *   1. the ARM RX ISR / dispatch chain   [0x0801F920, high confidence]
 *   2. the 8051 counterpart UARTX.C      (UART4 handler, ground truth)
 *   3. the vendor protocol spreadsheet   (APRS CPU与对讲机CPU通讯协议.xlsx)
 *
 * Commands confirmed in the ARM image / 8051 source:
 *   radio->APRS : $A00 (read settings), $A01,<512B> (write, 519 B frame),
 *                 $A09/$A10/$A11 (beacon triggers), $B/$C (beacon lists),
 *                 $D00/$F00 (acks), $A30/$A31, $A50/$A51, AT+* set
 *   APRS->radio : $A00,ID,512B (upload), $A02 telemetry, $A08 live beacon,
 *                 $E00/$E01/$E02 (channel/PTT), $F01/$F02 (beeps)
 */
#ifndef PROTOCOL_H
#define PROTOCOL_H

#include <stdint.h>

/* Called by the USART2 RX ISR when a complete '$' frame arrives. */
void protocol_frame_received(const uint8_t *frame, uint32_t len);

/* TX helpers (RECOVERED shapes):
 * - 6-byte command send: dispatcher cases send "$Axx\r\n" via usart2_send
 *   (command table lives in the stripped tail of the 190416 image; the
 *   command set is known from the 8051 source).
 * - settings write: 519-byte frame "$A01," + 512 B + "\r\n"
 *   [0x08016702 | RECOVERED: 5-byte prefix memcpy + 514-byte body + send]. */
void protocol_send_cmd(uint8_t cmd_id);        /* sends "$Axx\r\n"          */
void protocol_send_settings(const uint8_t body512[512]);
void protocol_send_status_text(const char *text6);  /* 6-byte status feed   */

#endif /* PROTOCOL_H */
