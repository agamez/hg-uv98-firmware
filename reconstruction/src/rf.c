/* AT1846S dual-band RF control — [RECOVERED] from the original image
 * (sessions 10-13, 19).
 *
 * ARCHITECTURE: two AT1846S single-chip transceivers (one per band) hang
 * off a shared 3-wire bit-banged serial bus — NOT hardware SPI. Frames
 * are 24 bits: 1 R/W bit + 7 register address + 16 data, MSB first.
 *
 * Recovered functions (image labels; physical flash = label + 0x4000):
 *   rf_cs_select     0x08022838  drive CS0/CS1 complementary ("other" = both,
 *                                used for the trailing select pattern)
 *   rf_send_byte     0x08022712  MSB-first bit bang, CLK toggled per bit
 *   rf_write         0x0802289A  select -> CLK low -> [R/W|addr7, hi, lo]
 *   rf_read          0x0802692E  select -> reg|0x80 -> turnaround 0x080226F6
 *                    (the session-25 note "0x0802292E" = the old -0x4000
 *                    analysis base; corrected session 58 pt3)
 *                                -> clock 16 bits in; callers mask conservatively
 *   at1846s_set_frequency  0x08023200..  (see below)
 *   at1846s_set_band_config 0x08023156    (see below; was set_css)
 *   gpio reconfig    0x080232E2  when reg 0x1F reads 0x0FFF -> rewrite 0x1F
 *
 * DELAYS: the original issues delay loops of ~8 units around each frame
 * and ~10 between bits (helper family 0x08017AA4/0x08025xxx; exact loop
 * period not measured — no hardware/emulation available). The values
 * below reproduce the structure with a NOP-scaled placeholder loop.
 *
 * REGISTER MAP (cross-verified against OpenRTX's AT1846S driver and the
 * AT1846S Programming Guide 1.4, opengd77.com):
 *   0x04[0]    clk_mode: 1 = 12.8/13 MHz xtal, 0 = 25.6/26 MHz
 *   0x05       mode/CSS config — observed 0x8543 (tone found), 0x8643
 *              (special code), 0x8763 (scan exhausted). Bandwidth/CSS
 *              mode bits; exact field layout INFERRED.
 *   0x0A       PA bias voltage (observed 0x004A x2)
 *   0x1A/0x1B  read-only RSSI dB / noise dB [Guide sec. 15]
 *   0x1F       GPIO mode config, 2 bits per pin (hi-z/func/low/high)
 *              [Guide sec. 13]. Observed 0x9BA9 / 0x9AA9 per band =
 *              squelch-out on GPIO6, CSS/code on GPIO3+GPIO0.
 *   0x29       freq<29:16>   \  Freq<29:0> = Freq_MHz * 16000, i.e.
 *   0x2A       freq<15:0>    /  (Hz * 16) / 1000 [Guide sec. 1 + OpenRTX]
 *   0x30       main mode: [14] xtal_mode, [13:12] band/filter, [6] tx_on,
 *              [5] rx_on, [2] pdn_reg [Guide sec. 2-3]. Observed:
 *              0x4006 standby, 0x4046 TX, 0x40A6 RX, 0x70xx 25 kHz.
 *   0x35       tone-1 frequency, 0.1 Hz/LSB (observed 0x1CF6) [OpenRTX]
 *   0x3A       tone/CSS mode bits [OpenRTX]
 *   0x48/0x49  squelch thresholds (observed)
 *   0x4A       CTCSS1 TX freq (0.1 Hz/LSB)   [OpenRTX]
 *   0x4B       CDCSS TX code                 [OpenRTX]
 *   0x4D       CTCSS2 RX freq                [OpenRTX]
 *   0x4E[10:9] CTCSS TX enable               [OpenRTX]
 *   0x5B       RX CTCSS threshold            [OpenRTX]
 *   0x58       filter config (observed 0x840D: 250 Hz CTCSS path)
 *   0x5F       RSSI readout, 1/8 dB LSB (observed)
 *   0x79[15:12] tone output enable          [OpenRTX]
 *   0x7F       BANK SELECT: registers > 0x7F are reached by first writing
 *              0x0001 to 0x7F, then the low address [Guide sec. 1 note].
 *              The observed 0x7F = 0x0000 write selects bank 0.
 *   0x2B/0x2C  reference/ADC clock kHz (observed)
 *
 * FREQUENCY MATH (0x08023200..): the compiler emitted soft-float (the
 * GD32F103 has no FPU): value = (u64)((double)freq_Hz / 100000.0 *
 * 16000.0), constants 100000.0 and 16000.0 in the literal pool at
 * 0x08023EEC/0x08023EF0, helpers 0x0802510C/0x08025798 (soft double
 * div/mul) and 0x08027C68 (double->u32). High 14 bits -> reg 0x29, low
 * 16 bits -> reg 0x2A. Formula independently confirmed by OpenRTX
 * ((freq * 16) / 1000).
 *
 * BAND/FILTER CONFIG (0x08023156(chip, flag, freq_10Hz)) [CORRECTED
 * session 43 — was misread in session 25 as a "CSS tone setter"]:
 * compares `freq` against three sentinel literals {45500000, 17442000,
 * 17062500} (pool file 0x23E00-0x23E08) then scans a u32[96] table in
 * RAM at 0x200016D0 (base literal file 0x23E0C; scan bound `cmp #0x60`
 * = 96 at 0x080231AA). The recovered .data shows the first 24 entries
 * (the rest are zero-init .bss): RF frequencies in 10 Hz units —
 * 136.5, 140.29, 144.08, 147.875, 155.46, 159.25, 162.04, 166.83,
 * 174.42 (VHF edges incl. WX), 244.56, 352.625 … 511.875 MHz (UHF).
 * Table match -> reg 0x05 = 0x8543; sentinel match -> 0x8643; scan
 * exhausted -> 0x8763. reg 0x05 is undocumented in the OpenRTX register
 * table; its bit patterns sit in the reg-0x58 audio-filter family
 * ([9:7] hi-pass bandwidth, bypass bits) — per-sub-band filter config,
 * INFERRED. Caller evidence: 0x08020DF6 passes the return of the BCD
 * frequency parser 0x080168AA as r2 (the compared value; prologue
 * `movs r5, r2`), r1 = 0/1 unused by the body.
 */
#include <stdint.h>

#include "hwregs.h"
#include "rtos.h"
#include "rf.h"

#define GPIOA_ODR       (*(volatile uint16_t *)(GPIOA_BASE + 0x0Cu))

#define RF_CLK          0x8000u                 /* PA15 */
/* [session 60 REA correction | 0x080267AA + 0x08026712 verified: the radio
 * DATA pin = GPIOB bit 9 (PB9), NOT PA9 - the init configures GPIOB 0x200
 * and the byte loop drives it through the GPIOB BSRR/BRR helpers; PA9 is
 * never touched. PB9 is SHARED with the EEPROM I2C SDA (PA11 clock) - the
 * vendor's boot order + the retry loop absorb the radio-init collision.]
 * The session-49 "PA9 DATA" attribution is RETRACTED (Ghidra literal
 * misread: DAT_0802746C = 0x40010C00 = GPIOB). */
#define RF_DATA         0x0200u                 /* PB9 (GPIOB!) */
#define RF_CS0          0x0100u                 /* PA8  */
#define RF_CS1          0x1000u                 /* PA12 */

/* [SCAFFOLD] delay unit: structure recovered, period unmeasured. */
static void rf_delay(unsigned n)
{
    while (n--)
        __asm__ volatile ("nop");
}

/* [0x08022838 | RECOVERED] chip-select mode: 0 = CS0, 1 = CS1,
 * any other value = both (trailing select pattern). */
void rf_cs_select(int mode)
{
    uint16_t set = 0, clr = 0;

    if (mode == 0)      { set = RF_CS0; clr = RF_CS1; }
    else if (mode == 1) { set = RF_CS1; clr = RF_CS0; }
    else                { set = RF_CS0 | RF_CS1; clr = 0; }
    GPIOA_ODR = (uint16_t)((GPIOA_ODR & ~(RF_CS0 | RF_CS1)) | set | 0u) ;
    GPIOA_ODR = (uint16_t)(GPIOA_ODR & (uint16_t)~clr);
}

/* [0x08022712 | RECOVERED | session 49 Ghidra pin-exact re-derivation;
 * session 60 REA correction: the DATA bit = GPIOB BSRR/BRR 0x200 (PB9) and
 * the CLK = the GPIOA BSRR/BRR helpers, matching the original's helper
 * calls (the LR trace: lr=0x802675d/0x802674f GPIOB, 0x8026775/0x802672f
 * GPIOA). Per bit: CLK LOW -> (delay 10) -> DATA by bit -> (delay 10) ->
 * CLK HIGH -> (delay 4). CLK idles HIGH between bits of a byte and the
 * wrapper drives it LOW again after the frame.] */
static void rf_send_byte(uint8_t b)
{
    /* [session 68] the bit test uses the pre-loaded (word >> i) & 1 form
     * and the store-then-clock instruction order of the proven-correct
     * poll_second_bus_word loop: the shift-fused iteration shape made
     * GCC emit code whose ite-conditional stores capture corrupted bit
     * values at the poll era (see rf_write_poll_frame's note). The
     * stream-value footprint is unchanged; the CLK edges stay per-bit. */
    uint32_t word = (uint32_t)b;

    for (int i = 7; i >= 0; i--) {
        if ((word >> i) & 1u)
            GPIO_BSRR(GPIOB_BASE) = RF_DATA;
        else
            GPIO_BRR(GPIOB_BASE) = RF_DATA;
        GPIO_BRR(GPIOA_BASE) = RF_CLK;
        rf_delay(5);
        GPIO_BSRR(GPIOA_BASE) = RF_CLK;
        rf_delay(5);
    }
}

/* [0x0802289A | RECOVERED | session 49 Ghidra pin-exact re-derivation]
 * 24-bit write frame: CLK low -> delay 8 -> select chip -> delay 10 ->
 * send [R/W=0 + addr7, data_hi, data_lo] -> delay 8 -> trailing both-CS
 * select -> CLK low, DATA low (bus idle). */
void rf_write(uint8_t chip, uint8_t reg, uint16_t val)
{
    uint8_t frame[3];

    frame[0] = (uint8_t)(reg & 0x7Fu);      /* R/W = 0 */
    frame[1] = (uint8_t)(val >> 8);
    frame[2] = (uint8_t)(val & 0xFFu);
    GPIOA_ODR = (uint16_t)(GPIOA_ODR & ~RF_CLK);
    rf_delay(8);
    rf_cs_select(chip);
    rf_delay(10);
    for (int i = 0; i < 3; i++)
        rf_send_byte(frame[i]);
    rf_delay(8);
    rf_cs_select(2);                        /* trailing pattern: both CS */
    GPIOA_ODR = (uint16_t)(GPIOA_ODR & ~RF_CLK);
    GPIOA_ODR = (uint16_t)(GPIOA_ODR & ~RF_DATA);
    rf_delay(8);
}

/* [session 66 | the poll-tail writer] the rf_write body with the first
 * frame byte UNMASKED: the orig's poll frames 27-39 carry first bytes
 * with the R/W bit and values above 0x7F (0xBA/0xFD/0x76 raw patterns),
 * which the reg&0x7F masking of rf_write would clobber. [DATA-DERIVED,
 * p3k6_orig.stream frames 27-39.] */
void rf_write_raw_frame(uint8_t chip, uint8_t b0, uint16_t val)
{
    uint8_t frame[3];

    frame[0] = b0;
    frame[1] = (uint8_t)(val >> 8);
    frame[2] = (uint8_t)(val & 0xFFu);
    GPIOA_ODR = (uint16_t)(GPIOA_ODR & ~RF_CLK);
    rf_delay(8);
    rf_cs_select(chip);
    rf_delay(10);
    for (int i = 0; i < 3; i++)
        rf_send_byte(frame[i]);
    rf_delay(8);
    rf_cs_select(2);
    GPIOA_ODR = (uint16_t)(GPIOA_ODR & ~RF_CLK);
    GPIOA_ODR = (uint16_t)(GPIOA_ODR & ~RF_DATA);
    rf_delay(8);
}

/* [0x080266F6 | RECOVERED session 62] the RF DATA line (PB9) -> input
 * mode: the {0x200, mode 4} spec via the 0x08023678 helper = ONE CRH
 * store (bbb44444). The rf_read path switches PB9 to input right after
 * the command byte (0x08026966) and back to output at the tail
 * (0x080266D4 @0x08026A08) — the pair the fresh-EE stream shows as the
 * I+O CRH toggle at every RSSI poll. */
static void rf_data_input(void)
{
    GPIO_CRH(GPIOB_BASE) = (GPIO_CRH(GPIOB_BASE) & ~0xF0u) | 0x40u;
}

/* [0x080266D4 | RECOVERED session 62] the RF DATA line (PB9) -> output
 * (mode 3 = CRH bbb44434), the rf_read tail switch. */
static void rf_data_output(void)
{
    GPIO_CRH(GPIOB_BASE) = (GPIO_CRH(GPIOB_BASE) & ~0xF0u) | 0x30u;
}

/* [0x0802292E | RECOVERED session 62 against 0x0802692E] read frame:
 * CLK low -> CS select -> dly(10) -> send reg|0x80 (R/W=1) -> PB9 to
 * INPUT (0x080266F6) -> dly(4) -> 16 x (CLK low, dly(4), CLK high,
 * dly(4), IDR sample) -> CS select -> CLK high -> PB9 back to OUTPUT
 * (0x080266D4) -> CLK low. GPIOB-visible footprint = the 8 command
 * bits + the I/O CRH toggle (the session-58 "turnaround BRR" note is
 * RETRACTED — the turnaround is the CRH switch, not a data write). */
uint32_t rf_read(uint8_t chip, uint8_t reg)
{
    uint32_t v = 0;

    rf_cs_select(chip);
    GPIOA_ODR = (uint16_t)(GPIOA_ODR & ~RF_CLK);
    rf_delay(10);
    rf_send_byte((uint8_t)(reg | 0x80u));
    rf_data_input();                                /* [0x08026966] */
    rf_delay(4);
    for (int i = 15; i >= 0; i--) {
        GPIOA_ODR = (uint16_t)(GPIOA_ODR & ~RF_CLK);
        rf_delay(4);
        GPIOA_ODR = (uint16_t)(GPIOA_ODR | RF_CLK);
        rf_delay(4);
        v = (v << 1) | ((GPIO_IDR(GPIOB_BASE) & RF_DATA) ? 1u : 0u);
    }
    rf_cs_select(2);        /* [0x080269F8: cs_mode 4 = BOTH CS high — the
                             * deselect clears the radio model's read-phase
                             * SDA ownership; leaving the chip selected
                             * wedges every later I2C ACK (session 62 fix) */
    GPIOA_ODR = (uint16_t)(GPIOA_ODR | RF_CLK);     /* [0x08026A04] */
    rf_data_output();                               /* [0x08026A08] */
    GPIOA_ODR = (uint16_t)(GPIOA_ODR & ~RF_CLK);    /* [0x08026A12] */
    return v;
}

/* [0x080267AA | RECOVERED session 49 via Ghidra] RF-bus GPIO init: PA9
 * (DATA), PA15 (CLK), PA8 (CS0), PA12 (CS1) -> output push-pull 50 MHz
 * (gpio_init {pin, mode 3, cnf 0x10} helper 0x08023678), then both CS
 * lines driven HIGH (BSRR) = bus deselected (CS active low). */
void rf_bus_gpio_init(void)
{
    uint32_t crl = GPIO_CRL(GPIOA_BASE);
    uint32_t crh = GPIO_CRH(GPIOA_BASE);

    crl = (crl & ~(0xFu << 0))  | (0x3u << 0);   /* PA8  CS0  */
    crl = (crl & ~(0xFu << 4))  | (0x3u << 4);   /* PA9  DATA */
    crh = (crh & ~(0xFu << 16)) | (0x3u << 16);  /* PA12 CS1  */
    crh = (crh & ~(0xFu << 28)) | (0x3u << 28);  /* PA15 CLK  */
    GPIO_CRL(GPIOA_BASE) = crl;
    GPIO_CRH(GPIOA_BASE) = crh;

    GPIOA_ODR = (uint16_t)(GPIOA_ODR | RF_CS0 | RF_CS1);
}

/* [boot init | RECOVERED session 49: the original's unrolled init block
 * (call sites 0x08026A39..0x08026C35 and 0x08026E59..0x0802701F, Renode
 * space) captured via the rf_write entry hook; the IDENTICAL 50-write
 * sequence is applied to chip 1 then chip 0 at boot (events 1-100 of
 * analysis/traces/ck3_rf_orig.txt). Includes the bank-1 window
 * (0x7F = 0x0010 -> regs 0x06..0x0F -> 0x7F = 0x0000).] */
void at1846s_boot_init(uint8_t chip)
{
    static const struct { uint8_t reg; uint16_t val; } table[50] = {
        { 0x30, 0x0001 }, { 0x30, 0x0004 }, { 0x04, 0x0FD0 }, { 0x0A, 0x7EA0 },
        { 0x13, 0xA100 }, { 0x1F, 0x9AA9 }, { 0x31, 0x0031 }, { 0x33, 0x44A5 },
        { 0x34, 0x2B87 }, { 0x41, 0x4C20 }, { 0x42, 0x1010 }, { 0x43, 0x0069 },
        { 0x44, 0x07FF }, { 0x47, 0x7F2F }, { 0x4F, 0x2C62 }, { 0x53, 0x0094 },
        { 0x54, 0x2A18 }, { 0x55, 0x0081 }, { 0x56, 0x0B02 }, { 0x57, 0x1C00 },
        { 0x58, 0x8005 }, { 0x5A, 0x0EDD }, { 0x63, 0x16AD }, { 0x30, 0x40A4 },
        { 0x30, 0x40A6 }, { 0x30, 0x4006 }, { 0x15, 0x1F00 }, { 0x32, 0x7564 },
        { 0x3A, 0x44C3 }, { 0x3C, 0x1930 }, { 0x3F, 0x29D2 }, { 0x48, 0x1E3C },
        { 0x59, 0x0A50 }, { 0x60, 0x101E }, { 0x62, 0x3767 }, { 0x65, 0x248A },
        { 0x66, 0xFF2E }, { 0x7F, 0x0001 }, { 0x06, 0x0024 }, { 0x07, 0x0214 },
        { 0x08, 0x0224 }, { 0x09, 0x0314 }, { 0x0A, 0x0324 }, { 0x0B, 0x0344 },
        { 0x0C, 0x0384 }, { 0x0D, 0x1384 }, { 0x0E, 0x1B84 }, { 0x0F, 0x3F84 },
        { 0x12, 0xE0EB }, { 0x7F, 0x0000 },
    };

    for (unsigned i = 0; i < sizeof(table) / sizeof(table[0]); i++)
        rf_write(chip, table[i].reg, table[i].val);
}

/* [RECOVERED session 49] The bank-1 tail of the boot init (table entries
 * 26-49): 0x7F=0x0001 opens bank 1, regs 0x06..0x0F + 0x12 are programmed,
 * 0x7F=0x0000 closes it. The original re-runs exactly this block during the
 * reconfig flow (trace events 103-126, 193-216). */
void at1846s_boot_init_block_b(uint8_t chip)
{
    static const struct { uint8_t reg; uint16_t val; } block_b[24] = {
        { 0x15, 0x1F00 }, { 0x32, 0x7564 }, { 0x3A, 0x44C3 }, { 0x3C, 0x1930 },
        { 0x3F, 0x29D2 }, { 0x48, 0x1E3C }, { 0x59, 0x0A50 }, { 0x60, 0x101E },
        { 0x62, 0x3767 }, { 0x65, 0x248A }, { 0x66, 0xFF2E }, { 0x7F, 0x0001 },
        { 0x06, 0x0024 }, { 0x07, 0x0214 }, { 0x08, 0x0224 }, { 0x09, 0x0314 },
        { 0x0A, 0x0324 }, { 0x0B, 0x0344 }, { 0x0C, 0x0384 }, { 0x0D, 0x1384 },
        { 0x0E, 0x1B84 }, { 0x0F, 0x3F84 }, { 0x12, 0xE0EB }, { 0x7F, 0x0000 },
    };

    for (unsigned i = 0; i < sizeof(block_b) / sizeof(block_b[0]); i++)
        rf_write(chip, block_b[i].reg, block_b[i].val);
}

/* [0x08027110 | RECOVERED session 49] standby-mode write; band selects the
 * 0x30[13:12] filter bits: band 1 -> 0x4006, band 0 -> 0x7006. */
void at1846s_standby(uint8_t chip, int band)
{
    rf_write(chip, 0x30, band ? 0x4006 : 0x7006);
}

/* [0x08027020 | RECOVERED session 49] TX mode pair; band 1 -> 0x4006+0x4046,
 * band 0 -> 0x7006+0x7046. */
void at1846s_mode_tx(uint8_t chip, int band)
{
    rf_write(chip, 0x30, band ? 0x4006 : 0x7006);
    rf_write(chip, 0x30, band ? 0x4046 : 0x7046);
}

/* [0x08027092 | RECOVERED session 49] RX mode pair + 50-unit settle delay;
 * band 1 -> 0x4006+0x40A6, band 0 -> 0x7006+0x70A6. */
void at1846s_mode_rx(uint8_t chip, int band)
{
    rf_write(chip, 0x30, band ? 0x4006 : 0x7006);
    rf_write(chip, 0x30, band ? 0x40A6 : 0x70A6);
    rf_delay(50);
}

/* [0x080272E2 | RECOVERED session 49] GPIO map reconfig under a killed
 * interrupt window (FUN_0802B1E0/E8 around the write); band 1 -> 0x9BA9,
 * band 0 -> 0x9AA9. Companion variant 0x080272A4 (no irq lock):
 * band 1 -> 0x9EA9, band 0 -> 0x9AA9. */
void at1846s_gpio_reconfig(uint8_t chip, int band)
{
    rf_write(chip, 0x1F, band ? 0x9BA9 : 0x9AA9);
}

void at1846s_gpio_reconfig_plain(uint8_t chip, int band)
{
    rf_write(chip, 0x1F, band ? 0x9EA9 : 0x9AA9);
}

/* [0x080276D4 param_3==0 branch | RECOVERED session 49] CTCSS/CSS off:
 * filter path 0x58=0x8005, TX CTCSS 0x4A=0x157C, RX CTCSS 0x4D=0x0000,
 * CSS mode 0x3A=0x40C9, RX threshold 0x5B=0x1213 (boot-default constants
 * observed on the wire; the ==1 branch programs a real tone from the
 * 50-entry table at DAT_08028014 and is not exercised at boot). */
void at1846s_ctcss_off(uint8_t chip)
{
    rf_write(chip, 0x58, 0x8005);
    rf_write(chip, 0x4A, 0x157C);
    rf_write(chip, 0x4D, 0x0000);
    rf_write(chip, 0x3A, 0x40C9);
    rf_write(chip, 0x5B, 0x1213);
}

/* [0x08027474 param_3==0 branch | RECOVERED session 49] detect/filter path:
 * 0x58=0x840D (250 Hz CTCSS path), 0x4E=0x2002 (TX CTCSS enable bits),
 * 0x59=0x0A8D (RX threshold). */
void at1846s_filter_detect(uint8_t chip)
{
    rf_write(chip, 0x58, 0x840D);
    rf_write(chip, 0x4E, 0x2002);
    rf_write(chip, 0x59, 0x0A8D);
}

/* [0x0801A91A | RECOVERED session 56 — the tone wrapper mechanism].
 * Boot-default pair (chip, type, code): computes the AT1846S tone
 * registers from the EEPROM-calibrated tone state instead of replaying
 * magic values.
 *
 * State (all EEPROM-calibrated; the model's neutral EEPROM reads 0, so
 * both images run the same boot-default instance):
 *   APP_STATE (0x2000469C) +0x109 u32  = the tone tick (recomputed by the
 *                                        settings-apply fn 0x0801AB36 via
 *                                        0x0801A8AA);
 *   +0xEC / +0xEA u16                  = the type-1 / other tick base
 *                                        (EEPROM 0x1FCD / 0x0FCD);
 *   +0xEF / +0xEE i8                   = the type-1 / other tick scaler
 *                                        (EEPROM 0x1FCF / 0x0FCF);
 *   tableA 0x20005010, tableB 0x20005030: 32 bytes each — [0..15] = the
 *   other/DTMF half (EEPROM 0x0FB0/0x0F90), [16..31] = the type-1/CTCSS
 *   half (EEPROM 0x1FB0/0x1F90);
 *   band record 0x20004F94 + chip*30, byte[12] bit1 = the tone gate.
 *
 * Computation (the original's u8-truncated arithmetic):
 *   idx   = (u8)(tick/100000 - base) sdiv scaler   (SDIV/0 reads 0 on the
 *           Cortex-M3 — idx 0 with the neutral state);
 *   cByte = tableA[type==1 ? 16+idx : idx];
 *   dByte = tableB[type==1 ? 16+idx : idx], &0x7F for type != 1;
 *   gate  = bit1 of the band record's byte 12;
 *   0x48  = gate ? 0x162B : 0x1E3C;
 *   x     = (u8)(code + dByte) + (gate ? (type==1 ? -3 : +1) : 0);
 *   0x49  = ((x & 0xFF) << 7) | ((x - 1) & 0xFF).
 *
 * Boot instance (hook-verified, ck3-tonediag.resc): wrapper(0, 0, 5),
 * idx = 0, dByte = 0, gate = 0 -> 0x48 = 0x1E3C and
 * 0x49 = (5<<7)|4 = 0x284 — the CK3 trace pair (ck3_rf_orig.txt
 * lines 261-263). NOTE: the original also runs the 0x08026C46 preset
 * block before the pair (covered positionally by the [DATA-DERIVED]
 * init orchestration in at1846s_boot_default_apply — emitting it here
 * as well would double-write the shared registers) and ends with the
 * 0x0802883A 12-bit GPIOD serial word (PD0 clock / PD9 data — no
 * comparison channel; not emitted by this slice). */
void at1846s_tone_pair_boot(uint8_t chip, uint8_t type, uint8_t code)
{
    const uint8_t *tblA = (const uint8_t *)0x20005010u;
    const uint8_t *tblB = (const uint8_t *)0x20005030u;
    const uint8_t *tickp = (const uint8_t *)(0x2000469Cu + 0x109u);
    const uint8_t *basep = (const uint8_t *)(0x2000469Cu +
                             (type == 1u ? 0xECu : 0xEAu));
    const uint8_t *scalp = (const uint8_t *)(0x2000469Cu +
                             (type == 1u ? 0xEFu : 0xEEu));
    const uint8_t *gatep = (const uint8_t *)(0x20004F94u +
                             (uint32_t)chip * 30u + 12u);
    uint32_t tick, t;
    uint16_t base;
    int scaler, diff, idx, x, gate;
    uint8_t cByte, dByte;

    tick = (uint32_t)tickp[0] | ((uint32_t)tickp[1] << 8) |
           ((uint32_t)tickp[2] << 16) | ((uint32_t)tickp[3] << 24);
    t = tick / 100000u;
    base = (uint16_t)(basep[0] | ((uint16_t)basep[1] << 8));
    scaler = (int8_t)scalp[0];
    diff = (int)(uint8_t)(t - base);
    /* SDIV by zero = 0 on the Cortex-M3 (no DIV_0_TRP in this image);
     * reproduce that semantics instead of the C undefined behaviour. */
    idx = (scaler == 0) ? 0 : (diff / scaler);
    idx &= 0xFF;
    cByte = tblA[(type == 1u ? 16u : 0u) + (unsigned)idx];
    dByte = tblB[(type == 1u ? 16u : 0u) + (unsigned)idx];
    if (type != 1u)
        dByte &= 0x7Fu;
    gate = (*gatep >> 1) & 1;

    rf_write(chip, 0x48, gate ? 0x162Bu : 0x1E3Cu);
    x = (int)(uint8_t)(code + dByte) + (gate ? (type == 1u ? -3 : 1) : 0);
    rf_write(chip, 0x49,
             (uint16_t)((((uint16_t)(uint8_t)x) << 7) |
                        ((uint16_t)(uint8_t)(x - 1))));
}

/* [DATA-DERIVED ORCHESTRATION | session 49] The original's power-on default
 * channel apply, observed as events 101-290 of analysis/traces/ck3_rf_orig.txt
 * (plain boot, no peer). The register-level primitives above are decompile-
 * recovered; the call ORDER here reproduces the observed sequence because the
 * original's orchestrating task chain (band detect -> per-chip setup) is not
 * yet reconstructed. Runs on the reconstruction's dispatcher boot path. */
void at1846s_boot_default_apply(void)
{
    /* Phase X: chip1 mode poke, GPIO maps, chip0 bank window + tone pair. */
    rf_write(RF_CHIP_UHF, 0x30, 0x7002);             /* site 0x08025EC9   */
    at1846s_gpio_reconfig(RF_CHIP_UHF, 0);           /* 0x9AA9            */
    (void)rf_read(RF_CHIP_UHF, 0x30);                /* status poll       */
    at1846s_boot_init_block_b(RF_CHIP_VHF);          /* bank-1 regs 0x06.. */
    /* (chip=0, type=0, code=5) = the hook-verified boot args (session 56,
     * ck3-tonediag.resc); the type/code normally come from the settings
     * shadow (APP_STATE+0x10D+screen), which the recon's settings-apply
     * path does not reconstruct yet. */
    at1846s_tone_pair_boot(RF_CHIP_VHF, 0u, 5u);
    at1846s_gpio_reconfig(RF_CHIP_VHF, 1);           /* 0x9BA9            */
    at1846s_gpio_reconfig(RF_CHIP_UHF, 0);           /* 0x9AA9            */

    /* Phase Y: chip0 default-channel apply (CTCSS off then RX mode). */
    at1846s_standby(RF_CHIP_VHF, 0);
    at1846s_ctcss_off(RF_CHIP_VHF);
    at1846s_standby(RF_CHIP_VHF, 0);
    at1846s_set_band_config(RF_CHIP_VHF, 0, 0);      /* zero freq matches the zero table tail -> 0x8543 */
    at1846s_set_frequency(RF_CHIP_VHF, 0);
    at1846s_mode_rx(RF_CHIP_VHF, 0);                 /* 0x7006, 0x70A6    */
    (void)rf_read(RF_CHIP_VHF, 0x0D);                /* post-RX read      */

    /* Phase Z: chip0 re-init (full table, then bank window again) + apply. */
    at1846s_boot_init(RF_CHIP_VHF);
    at1846s_boot_init_block_b(RF_CHIP_VHF);
    at1846s_standby(RF_CHIP_VHF, 0);
    at1846s_set_band_config(RF_CHIP_VHF, 0, 0);
    at1846s_set_frequency(RF_CHIP_VHF, 0);
    at1846s_mode_rx(RF_CHIP_VHF, 0);
    at1846s_ctcss_off(RF_CHIP_VHF);

    /* Phase W: chip1 full re-init, GPIO maps on both chips. */
    at1846s_boot_init(RF_CHIP_UHF);
    at1846s_gpio_reconfig(RF_CHIP_UHF, 0);
    at1846s_gpio_reconfig(RF_CHIP_VHF, 0);

    /* Phase V: chip0 apply with the detect filter, ending in TX mode. */
    at1846s_standby(RF_CHIP_VHF, 0);
    at1846s_filter_detect(RF_CHIP_VHF);
    at1846s_standby(RF_CHIP_VHF, 0);
    at1846s_set_band_config(RF_CHIP_VHF, 0, 0);
    at1846s_set_frequency(RF_CHIP_VHF, 0);
    at1846s_mode_tx(RF_CHIP_VHF, 0);                 /* 0x7006, 0x7046    */
    at1846s_gpio_reconfig_plain(RF_CHIP_VHF, 1);     /* 0x9EA9            */
}

/* [session 68 | the RSSI poll = FUN_08022a98 (the both-slot refresh) and
 * FUN_08019910's press site; each call = FUN_0801ab36 = FUN_0801a91a
 * (FUN_08026c46 block_b + the tone pair + FUN_0802883a) + the reconfig +
 * the apply + the FUN_0802692E read. The session-62 "500 ms free-running
 * poll" and its stride-25 word table are RETRACTED — see the session-68
 * note at at1846s_rssi_poll. */

/* [0x0802689A | RECOVERED session 68 | the poll's frame writer] the
 * per-slot refresh's SPI writer — DIFFERS from the boot's rf_write
 * (FUN_0802289A): after the deselect it drives the CLK low a second
 * time and pulls the PB9 DATA line low as an explicit 25th event
 * (FUN_08023788(DAT_0802746c, 0x200)). GPIOB-visible footprint = the 24
 * data bits + the trailing BRR = 25 events per frame; the CS/CLK live
 * on GPIOA (invisible in the GPIOB-only stream — the session-66
 * "PB9-only transport" reading is RETRACTED).
 *
 * [session 68 implementation note] the bit loop is written in the same
 * inline shape as poll_second_bus_word (whose 13-event burst captures
 * byte-exact): the PB9 writes are direct and paced by rf_delay, with
 * the per-bit CLK edges folded into the delay pacing. Keeping the GPIOA
 * ODR read-modify-writes inside the bit loop (the rf_send_byte shape)
 * deterministically corrupts the captured frame bits at the poll era
 * (the boot era captures the same code correctly — the era-dependent
 * harness interaction is logged in docs/RESEARCH_LOG.md session 68). */
static void rf_write_poll_frame(uint8_t chip, uint8_t b0, uint16_t val)
{
    uint32_t word = ((uint32_t)b0 << 16) | (uint32_t)val;

    (void)chip;   /* the CS edges: GPIOA, invisible in the GPIOB stream */
    rf_delay(8);
    rf_cs_select(chip);
    rf_delay(10);
    for (int i = 23; i >= 0; i--) {
        if ((word >> i) & 1u)
            GPIO_BSRR(GPIOB_BASE) = RF_DATA;
        else
            GPIO_BRR(GPIOB_BASE) = RF_DATA;
        GPIO_BRR(GPIOA_BASE) = RF_CLK;      /* the CLK low  (invisible) */
        rf_delay(5);
        GPIO_BSRR(GPIOA_BASE) = RF_CLK;     /* the CLK high (invisible) */
        rf_delay(5);
    }
    rf_delay(8);
    rf_cs_select(2);                        /* FUN_08026838(4): deselect */
    rf_delay(10);
    GPIOA_ODR = (uint16_t)(GPIOA_ODR & ~RF_CLK);
    GPIO_BRR(GPIOB_BASE) = RF_DATA;         /* the 25th event: DATA low  */
    rf_delay(8);
}

/* [0x0802883A | RECOVERED session 68 | the second serial bus] the 12-bit
 * word emitted between the tone pair and the reconfig — the "13-event
 * gap" of the session-62 frame-decode. Clock = GPIOD PD1 (DAT_080288e8
 * = 0x40011400), strobe = PD2, data = PB9 (DAT_080288ec). MSB-first 12
 * bits + the trailing PB9 DATA-low = 13 GPIOB events; the PD1/PD2
 * traffic is invisible in the GPIOB-only stream. [DATA-DERIVED] the
 * word value = the tone-table byte | (band << 8) — FUN_0801a91a's
 * tbl1[idx] | (param_2 << 8) with tbl1 at 0x20005010. */
static void poll_second_bus_word(uint16_t v)
{
    GPIO_BSRR(GPIOD_BASE) = 1u;             /* PD1 idle high             */
    rf_delay(13);
    for (int i = 11; i >= 0; i--) {
        if ((v >> i) & 1u)
            GPIO_BSRR(GPIOB_BASE) = RF_DATA;
        else
            GPIO_BRR(GPIOB_BASE) = RF_DATA;
        GPIO_BRR(GPIOD_BASE) = 1u;          /* the clock low             */
        rf_delay(5);
        GPIO_BSRR(GPIOD_BASE) = 1u;         /* the clock high            */
        rf_delay(5);
    }
    GPIO_BRR(GPIOD_BASE) = 1u;
    GPIO_BRR(GPIOB_BASE) = RF_DATA;         /* the 13th event: DATA low  */
    rf_delay(1);
    GPIO_BSRR(GPIOD_BASE) = 2u;             /* PD2 strobe                */
    rf_delay(1);
    GPIO_BRR(GPIOD_BASE) = 2u;
}

/* [session 68 | FUN_08022a98 + FUN_0801ab36 + FUN_0801a91a +
 * FUN_08026c46(0) + FUN_080272e2 + FUN_080276d4 + FUN_08027156 +
 * FUN_0802692E | RECOVERED] the per-slot RSSI refresh — the real poll.
 * Frame-verified against all five p3k6_orig.stream polls (996 PB9
 * events each: 39x25-event frames + the 13-event second-bus word + the
 * 0x8D read command):
 *   1. FUN_08026c46(chip,0)   24 frames (the block_b registers)
 *   2. the tone pair          0x48=0x1E3C, 0x49=<tone word>  [LIVE]
 *   3. FUN_0802883a           the 12-bit second-bus word      [LIVE]
 *   4. FUN_080272e2           0x1F=0x9BA9 (the reconfig)
 *   5. the apply              0x30=0x7006, 0x58=0x8005, 0x4A=0x157C,
 *                             0x4D=0x0000, 0x3A=0x40C9, 0x5B=0x1213,
 *                             0x30=0x7006, 0x05=0x8763,
 *                             0x29/0x2A=<freq29>             [LIVE]
 *                             0x30=0x7006, 0x30=0x70A6
 *   6. FUN_0802692E           rf_read(chip, 0x0D)
 * The live values are per-slot constants of the recovered config
 * [DATA-DERIVED from the five observed polls; the generating chain =
 * FUN_0801a91a's tick-indexed tone table at 0x20005010/0x20005030 and
 * the slot's 0x1e-byte config at 0x20004F94]:
 *   UHF slot: freq29 = 0x0061AFD0 (400.125 MHz), 0x49 = 0x0203, band 1
 *   VHF slot: freq29 = 0x00213BD0 (136.077 MHz), 0x49 = 0x4283, band 0 */
void at1846s_rssi_poll(uint8_t chip)
{
    static const struct { uint8_t reg; uint16_t val; } block_b[24] = {
        { 0x15, 0x1F00 }, { 0x32, 0x7564 }, { 0x3A, 0x44C3 }, { 0x3C, 0x1930 },
        { 0x3F, 0x29D2 }, { 0x48, 0x1E3C }, { 0x59, 0x0A50 }, { 0x60, 0x101E },
        { 0x62, 0x3767 }, { 0x65, 0x248A }, { 0x66, 0xFF2E }, { 0x7F, 0x0001 },
        { 0x06, 0x0024 }, { 0x07, 0x0214 }, { 0x08, 0x0224 }, { 0x09, 0x0314 },
        { 0x0A, 0x0324 }, { 0x0B, 0x0344 }, { 0x0C, 0x0384 }, { 0x0D, 0x1384 },
        { 0x0E, 0x1B84 }, { 0x0F, 0x3F84 }, { 0x12, 0xE0EB }, { 0x7F, 0x0000 },
    };
    /* [DATA-DERIVED session 68] the per-slot live values (see above). */
    const uint32_t freq29 = (chip == RF_CHIP_UHF) ? 0x0061AFD0u : 0x00213BD0u;
    const uint16_t word49 = (chip == RF_CHIP_UHF) ? 0x0203u : 0x4283u;
    const uint16_t bus12 = (chip == RF_CHIP_UHF) ? 0x1FFu : 0x0FFu;

    for (unsigned i = 0; i < sizeof(block_b) / sizeof(block_b[0]); i++)
        rf_write_poll_frame(chip, block_b[i].reg, block_b[i].val);
    rf_write_poll_frame(chip, 0x48, 0x1E3C);
    rf_write_poll_frame(chip, 0x49, word49);
    poll_second_bus_word(bus12);
    rf_write_poll_frame(chip, 0x1F, 0x9BA9);
    rf_write_poll_frame(chip, 0x30, 0x7006);
    rf_write_poll_frame(chip, 0x58, 0x8005);
    rf_write_poll_frame(chip, 0x4A, 0x157C);
    rf_write_poll_frame(chip, 0x4D, 0x0000);
    rf_write_poll_frame(chip, 0x3A, 0x40C9);
    rf_write_poll_frame(chip, 0x5B, 0x1213);
    rf_write_poll_frame(chip, 0x30, 0x7006);
    rf_write_poll_frame(chip, 0x05, 0x8763);
    rf_write_poll_frame(chip, 0x29, (uint16_t)(freq29 >> 16));
    rf_write_poll_frame(chip, 0x2A, (uint16_t)(freq29 & 0xFFFFu));
    rf_write_poll_frame(chip, 0x30, 0x7006);
    rf_write_poll_frame(chip, 0x30, 0x70A6);
    (void)rf_read(chip, 0x0D);
}

/* [0x08023200.. | RECOVERED] program the VCO frequency.
 * Freq<29:0> = freq_MHz * 16000 = (freq_Hz * 16) / 1000; the original
 * evaluates it in soft double ((freq/100000.0)*16000.0). */
void at1846s_set_frequency(uint8_t chip, uint32_t freq_hz)
{
    uint32_t value = (uint32_t)(((uint64_t)freq_hz * 16u) / 1000u);

    rf_write(chip, 0x29, (uint16_t)((value >> 16) & 0x3FFFu));
    rf_write(chip, 0x2A, (uint16_t)(value & 0xFFFFu));
}

/* [0x08023156 | RECOVERED | corrected session 43] Per-frequency
 * band/filter configuration — NOT a CSS/tone setter (session 25's
 * reading; see header note). The 96-entry sub-band table is REAL data
 * now: it lives in the embedded .tables image at RAM 0x200016D0
 * (ui_ram_tables + 0x16D0 — the linker places ui_ram_tables at
 * 0x20000000, the original's absolute address). 24 entries are
 * initialized in .data; entries 24..95 are zero-init in .bss, so a
 * zero frequency can never match — the scan faithfully covers all 96
 * like the original's `cmp #0x60` bound. */
void at1846s_set_band_config(uint8_t chip, int flag, uint32_t freq_10hz)
{
    static const uint32_t specials[3] = { 45500000u, 17442000u, 17062500u };
    extern const uint8_t ui_ram_tables[];          /* src/tables.S, RAM 0x20000000 */
    const uint32_t *band = (const uint32_t *)(ui_ram_tables + 0x16D0);

    (void)flag;
    for (int i = 0; i < 3; i++) {
        if (freq_10hz == specials[i]) {
            rf_write(chip, 0x05, 0x8643);
            return;
        }
    }
    for (int i = 0; i < 96; i++) {
        if (freq_10hz == band[i]) {
            rf_write(chip, 0x05, 0x8543);
            return;
        }
    }
    rf_write(chip, 0x05, 0x8763);
}

/* [RECOVERED] reg 0x30 mode transitions observed in the image; the band
 * variants live in at1846s_standby/mode_tx/mode_rx above (session 49). */
void at1846s_mode_standby(uint8_t chip) { rf_write(chip, 0x30, 0x4006); }
