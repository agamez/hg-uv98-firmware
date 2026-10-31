/* Inter-MCU protocol layer — see protocol.h for provenance. */
#include "protocol.h"
#include "uart.h"
#include "mem.h"

/* ---- command table --------------------------------------------------------
 * The original dispatches received frames via an ~18-entry memcmp chain
 * against a string table that lives in the stripped tail of the 190416
 * image (0x0804481C+). The command SET is fully known from the 8051 source
 * and vendor spreadsheet; the original per-slot ordering is only partially
 * verifiable in the June build (non-constant string shift, research log
 * session 5). This reconstruction therefore uses a computed dispatcher with
 * the confirmed command set — a documented reordering, not a byte copy. */

typedef enum {
    CMD_A00_READ   = 0,     /* $A00\r\n   read 512 B settings (+ID)         */
    CMD_A01_WRITE,          /* $A01,<512B> write settings                      */
    CMD_A02_TELEM,          /* $A02,...  telemetry (APRS -> radio)            */
    CMD_A08_BEACON,         /* $A08,...  live beacon                          */
    CMD_A09_KEY,            /* $A09\r\n   side-key beacon trigger           */
    CMD_A10_ALARM,          /* $A10\r\n   alarm beacon trigger              */
    CMD_A11_PTT,            /* $A11\r\n   PTT-release beacon trigger        */
    CMD_A30_ODO,            /* $A30\r\n   odometer clear                    */
    CMD_A31_FIXED,          /* $A31\r\n   set fixed station                 */
    CMD_A35_DELWP,          /* $A35\r\n   delete waypoint records           */
    CMD_A50_VER,            /* $A50\r\n   read version                      */
    CMD_A51_ID,             /* $A51\r\n   read CPU ID                       */
    CMD_B_LIST,             /* $Bxx\r\n   beacon list read (pages B00-B19)  */
    CMD_C_DETAIL,           /* $Cxx\r\n   beacon detail read (C00-C99)      */
    CMD_D00,                /* $D00\r\n   ack -> $D00,OK\r\n                */
    CMD_E00_CHA,            /* $E00\r\n   channel A / PTT off               */
    CMD_E01_CHB,            /* $E01\r\n   channel B / PTT on                */
    CMD_F01_BEEP,           /* $F01\r\n   beep once                         */
    CMD_COUNT
} proto_cmd_t;

/* 6-byte wire forms (strings absent from the 190416 file: they lived in the
 * stripped tail; values follow the 8051 CHECK_AT_CMD strings). */
static const char *const cmd_table[CMD_COUNT] = {
    [CMD_A00_READ]   = "$A00\r\n",
    [CMD_A09_KEY]    = "$A09\r\n",
    [CMD_A10_ALARM]  = "$A10\r\n",
    [CMD_A11_PTT]    = "$A11\r\n",
    [CMD_A30_ODO]    = "$A30\r\n",
    [CMD_A31_FIXED]  = "$A31\r\n",
    [CMD_A35_DELWP]  = "$A35\r\n",
    [CMD_A50_VER]    = "$A50\r\n",
    [CMD_A51_ID]     = "$A51\r\n",
    [CMD_D00]        = "$D00\r\n",
    [CMD_E00_CHA]    = "$E00\r\n",
    [CMD_E01_CHB]    = "$E01\r\n",
    [CMD_F01_BEEP]   = "$F01\r\n",
};

/* RX side: ready-flag byte. Original keeps protocol/UI flags at
 * 0x20005094 (+9 bit0 = frame ready, +0xA/+0xB/+0xC UI refresh bits). */
static volatile uint8_t proto_flags;
#define FLAG_FRAME_READY 0x01

static char    rxBuf[536];      /* [0x08023EEC -> 0x20003A84] the $A00 working
                                 * copy — the original's parse surface (its
                                 * consumer mutates settings[8..9] in place) */
static char    rxBuf2[536];     /* [0x08023EF0 -> 0x20002F74] the $A01/$B/$C
                                 * archive buffer */
static char    rxBuf3[64];      /* [0x08023EFC -> 0x20004B94] the $A02 telemetry
                                 * buffer (the original's target for the
                                 * 5-byte-prefix telemetry frames) */
static uint32_t rxLen;

/* [session 62] the flags/length accessor for the dispatcher's
 * reply-gated boot path (see main.c dispatcher_task). */

uint8_t protocol_flags_read(void)
{
    return proto_flags;
}

void protocol_flags_clear(uint8_t mask)
{
    proto_flags &= (uint8_t)~mask;
}

/* [session 62] the last frame's length — the dispatcher gates the radio
 * apply on the REAL 0x216-byte $A00 settings reply (a spurious
 * short-frame receipt must not trigger the boot apply). */
uint32_t protocol_rx_len(void)
{
    return rxLen;
}

void protocol_frame_received(const uint8_t *frame, uint32_t len)
{
    /* [session 50 matrix | RECOVERED routing; CORRECTED session 58 — the
     * live marker test (ck2-set-scan.resc): 2 s after the handshake the
     * ORIGINAL has the frame ONLY in rxBuf (0x20003A84, the settings at
     * +0x14 = 0x20003A98); rxBuf2 (0x20002F74) stays zero. The earlier
     * "copies to BOTH buffers" decompile note conflated the receipt with
     * a later consumer-side archive step.] the ISR dispatch copies per
     * command: $A00 -> buffer1 (with the 0x216 length gate);
     * $A01/$B/$C -> buffer2; $A02 -> buffer3. */
    if (len >= 5 && mem_cmp(frame, "$A01,", 5) == 0) {
        if (len != 0x216)        /* settings upload frame length check   */
            return;              /* original clears the ready flag       */
        mem_copy(rxBuf2, frame, len);
        rxLen = len;
        proto_flags |= FLAG_FRAME_READY;
        return;
    }
    if (len >= 2 && mem_cmp(frame, "$B", 2) == 0) {
        mem_copy(rxBuf2, frame, len);
        rxLen = len;
        proto_flags |= FLAG_FRAME_READY;
        return;
    }
    if (len >= 2 && mem_cmp(frame, "$C", 2) == 0) {
        mem_copy(rxBuf2, frame, len);
        rxLen = len;
        /* original additionally checks frame+5 against a 7-byte weather
         * marker ("W," prefix) and routes to a 347 B detail buffer; the
         * else-branch copies the whole frame either way. */
        proto_flags |= FLAG_FRAME_READY;
        return;
    }
    /* [session 49 decompile 0x08023920; CORRECTED session 58 — the live
     * marker test shows the frame lands in rxBuf ONLY (rxBuf2 stays
     * zero; the "both buffers" step belongs to a later consumer)]: the
     * $A00 branch requires the 5-byte prefix "$A00," AND length ==
     * 0x216, then copies the frame to rxBuf and clears a UI-refresh
     * flag bit. */
    if (len >= 5 && mem_cmp(frame, "$A00,", 5) == 0) {
        if (len != 0x216)        /* settings response length gate          */
            return;              /* original clears the ready flag         */
        mem_copy(rxBuf, frame, len);
        rxLen = len;
        proto_flags |= FLAG_FRAME_READY;
        return;
    }
    if (len >= 5 && mem_cmp(frame, "$A02,", 5) == 0) {
        mem_copy(rxBuf3, frame, len);
        rxLen = len;
        proto_flags |= FLAG_FRAME_READY;
        return;
    }
    if (len >= 5 && mem_cmp(frame, "$A08,", 5) == 0) {
        /* [session 50 decompile] the original copies ONLY the 5-byte
         * prefix to buffer2 (FUN_0801d8c0(EF0, buf, 5)), then conditionally
         * processes buf+5 against a 7-byte marker gated on the app-state
         * byte at 0x20004E68+0x38. */
        mem_copy(rxBuf2, frame, 5);
        rxLen = 5;
        proto_flags |= FLAG_FRAME_READY;
        return;
    }
    if (len >= 5 && mem_cmp(frame, "$A09", 4) == 0) {
        mem_copy(rxBuf, frame, len);
        rxLen = len;
        proto_flags |= FLAG_FRAME_READY;
        return;
    }
    /* Remaining confirmed-but-unrouted commands ($A10/$A11/$A30/$A31/$D00/
     * $F00/$A50/$A51/$E0x/$F0x and the GB2312 app-text set) fall through in
     * the original's tail table, whose contents are in the stripped region
     * (open question, research log O1/session 3). Documented as unknown. */
}

void protocol_send_cmd(uint8_t cmd_id)
{
    /* [0x08010128 dispatcher | RECOVERED]: switch on command id, send the
     * 6-byte string via usart2_send(str, 6). */
    if (cmd_id < CMD_COUNT && cmd_table[cmd_id])
        usart2_send((const uint8_t *)cmd_table[cmd_id], 6);
}

void protocol_send_settings(const uint8_t body512[512])
{
    /* [0x08016702 | RECOVERED]: 5-byte prefix + 514-byte copy + send(519). */
    uint8_t frame[519];
    frame[0] = '$'; frame[1] = 'A'; frame[2] = '0'; frame[3] = '1'; frame[4] = ',';
    mem_copy(&frame[5], body512, 512);
    frame[517] = '\r';
    frame[518] = '\n';
    usart2_send(frame, sizeof(frame));
}

void protocol_send_status_text(const char *text6)
{
    /* Dispatcher pool strings ("05 S", "-4.5 dB", GB2312 options): the same
     * 6-byte send path (statuses feed the Bluetooth app through the 8051). */
    usart2_send((const uint8_t *)text6, 6);
}
