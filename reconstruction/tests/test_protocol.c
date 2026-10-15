/* Host-side conformance tests for the inter-MCU protocol layer.
 *
 * Compiled natively (see Makefile target `test-host`). The firmware's
 * protocol layer is hardware-independent: only usart2_send() touches
 * hardware, and it is mocked here with a capture sink. Running these
 * tests upgrades the Level-3 claim for the protocol layer from
 * "builds + consistent with static evidence" to "passes executed
 * conformance tests" (research log, session 17).
 *
 * Frame rules under test (research log sessions 3-5, 8051 UARTX.C +
 * vendor spreadsheet ground truth):
 *   $A01,<512B>\r\n  settings write  — length MUST be 0x216 (534)
 *   $A00,<...>       (with the same 0x216 check in this direction)
 *   $A02,/,$A08,     telemetry / beacon payloads
 *   $A09\r\n         side-key beacon trigger (len >= 4)
 *   $Bxx\r\n,$Cxx\r\n  beacon list/detail reads
 *   anything else    ignored (tail-table commands, stripped region)
 */
#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <assert.h>

/* --- capture sink for usart2_send ------------------------------------- */
static uint8_t sent[2048];
static uint32_t sent_len;
void usart2_send(const uint8_t *buf, uint32_t len)
{
    memcpy(sent + sent_len, buf, len);
    sent_len += len;
}

/* pull the implementation in (gives access to its internals) */
#include "../src/protocol.c"

static int failures;
#define CHECK(cond) do { \
    if (!(cond)) { printf("  FAIL %s:%d  %s\n", __FILE__, __LINE__, #cond); failures++; } \
} while (0)

static uint8_t body[512];

static void test_send_cmd(void)
{
    puts("test_send_cmd:");
    sent_len = 0;
    protocol_send_cmd(CMD_A00_READ);
    CHECK(sent_len == 6);
    CHECK(memcmp(sent, "$A00\r\n", 6) == 0);
    sent_len = 0;
    protocol_send_cmd(CMD_E01_CHB);
    CHECK(memcmp(sent, "$E01\r\n", 6) == 0);
    sent_len = 0;
    protocol_send_cmd(0xFF);            /* out of range: no crash, no send */
    CHECK(sent_len == 0);
}

static void test_settings_upload(void)
{
    puts("test_settings_upload:");
    memset(body, 0xA5, sizeof(body));
    sent_len = 0;
    protocol_send_settings(body);
    CHECK(sent_len == 519);
    CHECK(memcmp(sent, "$A01,", 5) == 0);
    CHECK(sent[5] == 0xA5 && sent[516] == 0xA5);
    CHECK(sent[517] == '\r' && sent[518] == '\n');

    /* RX side: the exact 0x216 frame is accepted, flag set */
    static uint8_t frame[0x216];
    memcpy(frame, "$A01,", 5);
    memset(frame + 5, 0x5A, 0x216 - 7);
    frame[0x214] = '\r'; frame[0x215] = '\n';
    proto_flags = 0;
    protocol_frame_received(frame, 0x216);
    CHECK(proto_flags & FLAG_FRAME_READY);
    CHECK(rxLen == 0x216);
    CHECK(memcmp(rxBuf, "$A01,", 5) == 0);
    CHECK(rxBuf[100] == 0x5A);

    /* wrong length is REJECTED (the 0x216 check) */
    proto_flags = 0;
    protocol_frame_received(frame, 0x215);
    CHECK((proto_flags & FLAG_FRAME_READY) == 0);
}

static void test_telemetry_and_reads(void)
{
    puts("test_telemetry_and_reads:");
    proto_flags = 0;
    protocol_frame_received((const uint8_t *)"$A02,52.3N,008.5E", 17);
    CHECK(proto_flags & FLAG_FRAME_READY);
    CHECK(rxLen == 17);

    proto_flags = 0;
    protocol_frame_received((const uint8_t *)"$A08,LIVEBEACON", 15);
    CHECK(proto_flags & FLAG_FRAME_READY);

    proto_flags = 0;
    protocol_frame_received((const uint8_t *)"$A09\r\n", 6);
    CHECK(proto_flags & FLAG_FRAME_READY);

    proto_flags = 0;
    protocol_frame_received((const uint8_t *)"$B03\r\n", 6);
    CHECK(proto_flags & FLAG_FRAME_READY);
    CHECK(rxLen == 6);

    proto_flags = 0;
    protocol_frame_received((const uint8_t *)"$C41\r\n", 6);
    CHECK(proto_flags & FLAG_FRAME_READY);
}

static void test_unknown_ignored(void)
{
    puts("test_unknown_ignored:");
    proto_flags = 0;
    protocol_frame_received((const uint8_t *)"$Z99\r\n", 6);
    CHECK((proto_flags & FLAG_FRAME_READY) == 0);
    protocol_frame_received((const uint8_t *)"$A50\r\n", 6);   /* unrouted tail cmd */
    CHECK((proto_flags & FLAG_FRAME_READY) == 0);
    /* short garbage never trips the $A01 prefix check */
    protocol_frame_received((const uint8_t *)"$A01", 4);
    CHECK((proto_flags & FLAG_FRAME_READY) == 0);
}

int main(void)
{
    test_send_cmd();
    test_settings_upload();
    test_telemetry_and_reads();
    test_unknown_ignored();
    if (failures) {
        printf("== %d FAILURE(S) ==\n", failures);
        return 1;
    }
    puts("== all protocol conformance tests passed ==");
    return 0;
}
