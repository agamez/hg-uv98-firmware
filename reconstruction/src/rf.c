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
 *   rf_read          0x0802292E  select -> reg|0x80 -> turnaround 0x080226F6
 *                                -> clock 16 bits in; callers mask conservatively
 *   at1846s_set_frequency  0x08023200..  (see below)
 *   at1846s_set_css        0x08023156    (see below)
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
 * CSS SELECTION (0x08023156(chip, css, flag)): compares `css` against
 * three observed sentinel codes (45500000, 17442000, 17062500 — the
 * first is exactly 45.5 MHz; exact sentinel semantics open) then scans
 * a 96-entry u32 table in RAM at 0x200016D0. Match -> reg 0x05 = 0x8543;
 * special match -> 0x8643; scan exhausted -> 0x8763. The table and the
 * sibling key tables are boot-time .data copies whose flash image is
 * NOT present in the distributed file (verified: no pointer runs, no
 * CSS-range runs, no u32 670 anywhere in the image) — contents closed.
 */
#include <stdint.h>

#include "rf.h"

#define GPIOA_BASE      0x40010800u
#define GPIOA_ODR       (*(volatile uint16_t *)(GPIOA_BASE + 0x0Cu))

#define RF_CLK          0x8000u                 /* PA15 */
#define RF_DATA         0x0200u                 /* PA9  */
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

/* [0x08022712 | RECOVERED] shift one byte MSB-first; CLK idles low and
 * toggles once per bit; DATA changes while CLK low. */
static void rf_send_byte(uint8_t b)
{
    for (int i = 7; i >= 0; i--) {
        if ((b >> i) & 1u)
            GPIOA_ODR = (uint16_t)(GPIOA_ODR | RF_DATA);
        else
            GPIOA_ODR = (uint16_t)(GPIOA_ODR & ~RF_DATA);
        GPIOA_ODR = (uint16_t)(GPIOA_ODR | RF_CLK);
        rf_delay(10);
        GPIOA_ODR = (uint16_t)(GPIOA_ODR & ~RF_CLK);
        rf_delay(10);
    }
}

/* [0x0802289A | RECOVERED] 24-bit write frame: R/W=0 + 7-bit address,
 * then 16-bit data high byte first. */
void rf_write(uint8_t chip, uint8_t reg, uint16_t val)
{
    rf_cs_select(chip);
    GPIOA_ODR = (uint16_t)(GPIOA_ODR & ~RF_CLK);
    rf_delay(8);
    rf_send_byte((uint8_t)(reg & 0x7Fu));   /* R/W = 0 */
    rf_send_byte((uint8_t)(val >> 8));
    rf_send_byte((uint8_t)(val & 0xFFu));
    rf_delay(8);
    rf_cs_select(2);                        /* trailing pattern: both CS */
}

/* [0x0802292E | RECOVERED] read frame: send reg|0x80 (R/W=1), turn the
 * bus around, clock 16 bits in. Original callers mask the result
 * conservatively (frames are 24-bit; return value widened to u32). */
uint32_t rf_read(uint8_t chip, uint8_t reg)
{
    uint32_t v = 0;

    rf_cs_select(chip);
    GPIOA_ODR = (uint16_t)(GPIOA_ODR & ~RF_CLK);
    rf_send_byte((uint8_t)(reg | 0x80u));
    GPIOA_ODR = (uint16_t)(GPIOA_ODR & ~RF_DATA);   /* turnaround */
    rf_delay(8);
    for (int i = 15; i >= 0; i--) {
        GPIOA_ODR = (uint16_t)(GPIOA_ODR | RF_CLK);
        rf_delay(10);
        GPIOA_ODR = (uint16_t)(GPIOA_ODR & ~RF_CLK);
        v = (v << 1) | ((GPIOA_ODR & RF_DATA) ? 1u : 0u);
        rf_delay(10);
    }
    rf_cs_select(2);
    return v;
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

/* [0x08023156 | RECOVERED] CSS/mode selection; reg 0x05 values as
 * observed. The 96-entry tone table lives in RAM at 0x200016D0 (boot
 * .data copy; flash image absent from the distributed file — see
 * header note), so the table-driven half is scaffolded. */
void at1846s_set_css(uint8_t chip, uint32_t css, int flag)
{
    (void)flag;
    static const uint32_t specials[3] = { 45500000u, 17442000u, 17062500u };

    for (int i = 0; i < 3; i++) {
        if (css == specials[i]) {
            rf_write(chip, 0x05, 0x8643);
            return;
        }
    }
    /* 96-entry table scan (RAM 0x200016D0) omitted: table contents not
     * present in the distributed artifact. Fallback config = scan
     * exhausted path. */
    rf_write(chip, 0x05, 0x8763);
}

/* [RECOVERED] reg 0x30 mode transitions observed in the image. */
void at1846s_mode_standby(uint8_t chip) { rf_write(chip, 0x30, 0x4006); }
void at1846s_mode_tx(uint8_t chip)      { rf_write(chip, 0x30, 0x4046); }
void at1846s_mode_rx(uint8_t chip)      { rf_write(chip, 0x30, 0x40A6); }

/* [0x080232E2 | RECOVERED] GPIO reconfiguration / soft-reset path:
 * entered when reg 0x1F reads 0x0FFF; the value is band-dependent. */
void at1846s_gpio_reconfig(uint8_t chip, int band)
{
    rf_write(chip, 0x1F, band ? 0x9BA9 : 0x9AA9);
}
