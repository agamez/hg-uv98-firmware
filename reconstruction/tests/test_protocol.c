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
/* [session 71] the uart-side RX window is stubbed for the host build: the
 * protocol conformance tests exercise protocol_frame_received directly. */
uint8_t u2_rx_open;

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

    /* RX side: the exact 0x216 frame is accepted, flag set.
     * [session 56 triage] the buffer target corrected to the session-50
     * matrix-v2 routing: $A01 -> rxBuf2 ONLY (rxBuf = the $A00 working
     * copy); the earlier rxBuf expectations were the session-20-era
     * routing, superseded by the 7/7 emulation matrix. */
    static uint8_t frame[0x216];
    memcpy(frame, "$A01,", 5);
    memset(frame + 5, 0x5A, 0x216 - 7);
    frame[0x214] = '\r'; frame[0x215] = '\n';
    proto_flags = 0;
    memset(rxBuf, 0, sizeof(rxBuf));
    protocol_frame_received(frame, 0x216);
    CHECK(proto_flags & FLAG_FRAME_READY);
    CHECK(rxLen == 0x216);
    CHECK(memcmp(rxBuf2, "$A01,", 5) == 0);
    CHECK(rxBuf2[100] == 0x5A);
    CHECK(rxBuf[0] == 0);               /* the $A00 working copy untouched */

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

/* Session 20 locked the ISR-chain matching at 4 bytes for $A00; the
 * session-50 matrix-v2 routing (7/7 emulation stimuli + decompiles)
 * superseded it: the frame router requires the 5-byte "$A00," prefix AND
 * the 0x216 length gate, $A01/$A02/$A08 match at 5 bytes ($A08 copies
 * only the prefix), $B/$C at 2, $A09 at a 4-byte prefix gated len>=5.
 * [session 56 triage: the stale 4-byte $A00 expectations re-baselined.] */
static void test_sentence_prefix_boundaries(void)
{
    puts("test_sentence_prefix_boundaries:");

    /* $A00 requires BOTH the 5-byte "$A00," prefix and len == 0x216 ... */
    proto_flags = 0;
    protocol_frame_received((const uint8_t *)"$A00", 4);
    CHECK((proto_flags & FLAG_FRAME_READY) == 0);
    /* ... the comma-prefixed 6-byte form is still length-gated out ... */
    proto_flags = 0;
    protocol_frame_received((const uint8_t *)"$A00,x", 6);
    CHECK((proto_flags & FLAG_FRAME_READY) == 0);
    /* ... and only the full 0x216 frame trips the ready flag. */
    proto_flags = 0;
    {
        static uint8_t a00[0x216];
        memcpy(a00, "$A00,", 5);
        memset(a00 + 5, 0x5A, 0x216 - 7);
        a00[0x214] = '\r'; a00[0x215] = '\n';
        protocol_frame_received(a00, 0x216);
    }
    CHECK((proto_flags & FLAG_FRAME_READY) != 0);
    /* 3-byte fragment never matches */
    proto_flags = 0;
    protocol_frame_received((const uint8_t *)"$A0", 3);
    CHECK((proto_flags & FLAG_FRAME_READY) == 0);

    /* $A02/$A08 require the comma: 4 bytes without it must NOT match */
    proto_flags = 0;
    protocol_frame_received((const uint8_t *)"$A02", 4);
    CHECK((proto_flags & FLAG_FRAME_READY) == 0);
    proto_flags = 0;
    protocol_frame_received((const uint8_t *)"$A08", 4);
    CHECK((proto_flags & FLAG_FRAME_READY) == 0);
    proto_flags = 0;
    protocol_frame_received((const uint8_t *)"$A02,x", 6);
    CHECK((proto_flags & FLAG_FRAME_READY) != 0);

    /* $A09: compares a 4-byte prefix but gates on len >= 5 (the full
     * sentence is "$A09\r\n") — 4 bytes alone must NOT trip it */
    proto_flags = 0;
    protocol_frame_received((const uint8_t *)"$A09", 4);
    CHECK((proto_flags & FLAG_FRAME_READY) == 0);
    proto_flags = 0;
    protocol_frame_received((const uint8_t *)"$A09\r", 5);
    CHECK((proto_flags & FLAG_FRAME_READY) != 0);

    /* $B/$C match at 2 bytes */
    proto_flags = 0;
    protocol_frame_received((const uint8_t *)"$B", 2);
    CHECK((proto_flags & FLAG_FRAME_READY) != 0);
    proto_flags = 0;
    protocol_frame_received((const uint8_t *)"$C", 2);
    CHECK((proto_flags & FLAG_FRAME_READY) != 0);
}

int main(void)
{
    test_send_cmd();
    test_settings_upload();
    test_telemetry_and_reads();
    test_unknown_ignored();
    test_sentence_prefix_boundaries();
    if (failures) {
        printf("== %d FAILURE(S) ==\n", failures);
        return 1;
    }
    puts("== all protocol conformance tests passed ==");
    return 0;
}
