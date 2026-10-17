/* I2C EEPROM bit-bang driver — [RECOVERED session 51, originals:
 *   0x0802846A sda_output, 0x08028448 sda_input, 0x080284F2 start,
 *   0x080285B6 byte+ack, 0x08028532 stop, 0x08028566 master ack,
 *   0x08028666 read byte, 0x080286C0 write page, 0x0802873C read.
 * SDA = PB9, SCL = PA11; the device address byte = 0xA0 (write) /
 * 0xA1 (read) — a 24Cxx-class slave. Data polarity on SDA is
 * NON-INVERTED (bit=1 -> SDA high) [session 55 correction; see
 * i2c_send_byte below]. The ACK is sampled through a PB9
 * direction switch (CRH nibble output 0x3 -> input 0x4); on the
 * canonical platform the pb_displaybus model returns IDR = 0, so the
 * ACK flag always reads "received" — the documented harness workaround
 * that keeps the boot alive. The original's consumer wipes/uses the
 * archive buffer within ~2 ms (matrix v2, session 50 pt3). */
#include "eeprom.h"
#include "hwregs.h"

#define I2C_SDA_BIT     0x200u      /* PB9  */
#define I2C_SCL_BIT     0x800u      /* PA11 */

#define EE_ACK_FLAG     1u          /* flag value meaning "slave ACKed" */

static uint8_t ee_ack_flag;         /* [0x2000515C in the original]     */

/* [0x080284E4] short delay: n empty iterations. */
static void dly(uint32_t n)
{
    volatile uint32_t i;
    while (n--)
        ;
}

/* [0x0802846A] PB9 -> output (CRH nibble 0x3); the rest of CRH is an
 * RMW in the original (mask 0x100). delay(10) follows. */
static void sda_output(void)
{
    GPIO_CRH(GPIOB_BASE) = (GPIO_CRH(GPIOB_BASE) & ~0xF0u) | 0x30u;
    dly(10);
}

/* [0x08028448] PB9 -> input (CRH nibble 0x4). */
static void sda_input(void)
{
    GPIO_CRH(GPIOB_BASE) = (GPIO_CRH(GPIOB_BASE) & ~0xF0u) | 0x40u;
    dly(10);
}

/* [0x080284F2] start/restart: SDA low, SCL low, then SDA high, SCL high. */
static void i2c_start(void)
{
    /* [session 53 | 0x080284F2 verified] SDA high, SCL high, SDA low,
     * SCL low — the captured sequence CRH, BSRR, BSRR, BRR, BRR. */
    sda_output();
    GPIO_BSRR(GPIOB_BASE) = I2C_SDA_BIT;
    GPIO_BSRR(GPIOA_BASE) = I2C_SCL_BIT;
    dly(10);
    GPIO_BRR(GPIOB_BASE) = I2C_SDA_BIT;
    dly(10);
    GPIO_BRR(GPIOA_BASE) = I2C_SCL_BIT;
}

/* [0x080285B6] send one byte MSB first, then the 9th-clock
 * ACK sample. [session 55 correction, re-verified against the
 * disassembly AND the captured stream: the SDA data polarity is
 * NON-INVERTED - bit=1 drives SDA HIGH (BSRR helper 0x08023780),
 * bit=0 drives SDA LOW (BRR helper 0x08023788); the captured 0xA0
 * device byte = BSRR,BRR,BSRR,BRR,BRR,BRR,BRR,BRR proves it. The
 * session-51/54 "inverted polarity" note is RETRACTED (it came from
 * misreading the bl targets - 0x08023780=BSRR(+0x10),
 * 0x08023788=BRR(+0x14) stands). The ack tail raises SDA (BSRR)
 * before the direction switch, then one SCL pulse; sets
 * ee_ack_flag = 1 when SDA reads 0. The original's per-byte capture
 * = CRH(out) + 8 SDA writes + BSRR + CRH(in) = 11 events.]
 * [session 60 REA decompile correction: the per-bit SCL order is
 * SDA write (while SCL is LOW) -> SCL BSRR (rise) -> dly(10) -> SCL
 * BRR (fall) - STANDARD timing, the SDA stable across the rise; the
 * session-51 order (SCL low -> dly -> SCL high) was a misreading.
 * The ACK tail = SCL BSRR (the 9th rise) -> dly(10) -> the IDR read
 * (while SCL is HIGH, 0x08028638) -> ee_ack_flag -> SCL BRR.] */
static void i2c_send_byte(uint8_t b)
{
    sda_output();
    for (uint32_t i = 0; i < 8u; i++) {
        if (b & 0x80u)
            GPIO_BSRR(GPIOB_BASE) = I2C_SDA_BIT;    /* bit=1 -> SDA high */
        else
            GPIO_BRR(GPIOB_BASE) = I2C_SDA_BIT;     /* bit=0 -> SDA low  */
        GPIO_BSRR(GPIOA_BASE) = I2C_SCL_BIT;        /* SCL high (rise)   */
        dly(10);
        GPIO_BRR(GPIOA_BASE) = I2C_SCL_BIT;         /* SCL low (fall)    */
        b = (uint8_t)(b << 1);
    }
    dly(10);
    GPIO_BSRR(GPIOB_BASE) = I2C_SDA_BIT;
    dly(1);
    sda_input();
    dly(1);
    GPIO_BSRR(GPIOA_BASE) = I2C_SCL_BIT;            /* the 9th clock rise */
    dly(10);
    /* the ACK is sampled while the SCL is HIGH, after the rise        */
    ee_ack_flag = ((GPIO_IDR(GPIOB_BASE) & I2C_SDA_BIT) == 0u) ? EE_ACK_FLAG : 0u;
    GPIO_BRR(GPIOA_BASE) = I2C_SCL_BIT;             /* the 9th clock fall */
}

/* [0x08028566] the master ack/nack phase: param 0 -> SDA high, non-zero
 * -> SDA low; one SCL low-high pulse around it.
 * [session 60 REA decompile: the pulse = SCL BSRR (rise) -> dly(10) ->
 * SCL BRR (fall); the SDA is set while the SCL is LOW before the pulse.] */
static void i2c_master_ack(uint8_t val)
{
    /* [session 53 | 0x08028566 verified] val==0 = the ACK = the SDA LOW;
     * val!=0 = the NAK = the SDA HIGH. */
    sda_output();
    if (val == 0u)
        GPIO_BRR(GPIOB_BASE) = I2C_SDA_BIT;
    else
        GPIO_BSRR(GPIOB_BASE) = I2C_SDA_BIT;
    dly(5);
    GPIO_BSRR(GPIOA_BASE) = I2C_SCL_BIT;
    dly(10);
    GPIO_BRR(GPIOA_BASE) = I2C_SCL_BIT;
    dly(5);
}

/* [0x08028666] read one byte MSB first; per bit: SCL LOW -> dly(10) ->
 * SCL HIGH (rise) -> sample the IDR (while the SCL is HIGH, after the
 * rise); leaves SCL LOW after a trailing dly(10) + SCL BRR.
 * [session 60 REA decompile: the session-51 order (rise -> fall ->
 * sample) was inverted.] */
static uint8_t i2c_read_byte(void)
{
    uint8_t v = 0;

    sda_input();
    for (uint32_t i = 0; i < 8u; i++) {
        GPIO_BRR(GPIOA_BASE) = I2C_SCL_BIT;
        dly(10);
        GPIO_BSRR(GPIOA_BASE) = I2C_SCL_BIT;
        v = (uint8_t)(v << 1);
        if (GPIO_IDR(GPIOB_BASE) & I2C_SDA_BIT)
            v = (uint8_t)(v + 1u);
    }
    dly(10);
    GPIO_BRR(GPIOA_BASE) = I2C_SCL_BIT;
    return v;
}

/* [0x08028532 | session 53 fix] stop: SDA low, SCL high, SDA high —
 * the standard I2C stop, ending with the bus idle HIGH. The session-51
 * note ("SDA high, SCL low, SDA low") mis-ordered the sequence: the
 * original's captured tail is CRH, BRR(low), BSRR(high), and the PB9
 * latch state propagates into every later display ODR read-modify-
 * write (the bit9 of the port). Verified against the original's stream. */
static void i2c_stop(void)
{
    /* [session 53 | 0x08028532 verified] SDA low, SCL high, SDA high —
     * the captured sequence CRH, BRR, BSRR. */
    sda_output();
    GPIO_BRR(GPIOB_BASE) = I2C_SDA_BIT;
    GPIO_BSRR(GPIOA_BASE) = I2C_SCL_BIT;
    dly(10);
    GPIO_BSRR(GPIOB_BASE) = I2C_SDA_BIT;
    dly(10);
}

/* [0x080286C0] write count bytes to addr (one byte per transaction,
 * address hi/lo then data, each ACK-retried). */
void eeprom_write(uint16_t addr, uint8_t count, const uint8_t *buf)
{
    for (uint8_t n = 0; n < count; n++) {
        i2c_start();
        do {
            i2c_start();
            i2c_send_byte(0xA0);
        } while (ee_ack_flag != EE_ACK_FLAG);
        i2c_send_byte((uint8_t)(addr >> 8));
        while (ee_ack_flag == 0u)
            ;
        i2c_send_byte((uint8_t)addr);
        while (ee_ack_flag == 0u)
            ;
        i2c_send_byte(*buf++);
        while (ee_ack_flag == 0u)
            ;
        i2c_stop();
        addr = (uint16_t)(addr + 1u);
    }
}

/* [0x0802873C] read count bytes from addr (a write-addressed setup, a
 * restart into 0xA1, then count bytes; NAK-style phase 0 between bytes
 * and phase 1 after the last). */
void eeprom_read(uint16_t addr, uint8_t count, uint8_t *buf)
{
    uint8_t i;

    i2c_start();
    do {
        i2c_start();
        i2c_send_byte(0xA0);
    } while (ee_ack_flag != EE_ACK_FLAG);
    i2c_send_byte((uint8_t)(addr >> 8));
    while (ee_ack_flag == 0u)
        ;
    i2c_send_byte((uint8_t)addr);
    while (ee_ack_flag == 0u)
        ;
    i2c_start();
    i2c_send_byte(0xA1);
    while (ee_ack_flag == 0u)
        ;
    for (i = 0; (uint8_t)(count - 1u) > i; i++) {
        *buf++ = i2c_read_byte();
        i2c_master_ack(0);
    }
    *buf = i2c_read_byte();
    i2c_master_ack(1);
    i2c_stop();
}
