/* AT1846S dual-band RF control — recovered 3-wire serial transport and
 * register semantics. See rf.c for the full provenance and register map. */
#ifndef RF_H
#define RF_H

#include <stdint.h>

/* Chip select indices (one AT1846S per band). */
#define RF_CHIP_VHF  0u
#define RF_CHIP_UHF  1u

/* Transport (image labels; physical flash = label + 0x4000). */
void rf_bus_gpio_init(void);            /* 0x080267AA (Renode space): PA9/PA15/PA8/PA12 outputs, CS idle high */
void rf_cs_select(int mode);            /* 0x08022838: 0 -> CS0, 1 -> CS1, other -> both */
void rf_write(uint8_t chip, uint8_t reg, uint16_t val);  /* 0x0802289A */
uint32_t rf_read(uint8_t chip, uint8_t reg);             /* 0x0802292E, reg|0x80 */

/* Boot-time register programming (session 49: captured 50-write table). */
void at1846s_boot_init(uint8_t chip);   /* the unrolled init block, both chips */

/* Register-level operations recovered from the image. */
void at1846s_set_frequency(uint8_t chip, uint32_t freq_hz); /* 0x08023200..: MHz*16000 -> 0x29/0x2A */
void at1846s_set_band_config(uint8_t chip, int flag, uint32_t freq_10hz); /* 0x08023156: sub-band filter scan (RAM 0x200016D0), reg 0x05 */
void at1846s_mode_standby(uint8_t chip);  /* reg 0x30 = 0x4006 */
void at1846s_mode_tx(uint8_t chip);       /* reg 0x30 = 0x4046 (0x7046 = 25 kHz variant) */
void at1846s_mode_rx(uint8_t chip);       /* reg 0x30 = 0x40A6 (0x70A6 = 25 kHz variant) */
void at1846s_gpio_reconfig(uint8_t chip, int band); /* 0x080232E2: 0x1F = 0x9BA9/0x9AA9 */

#endif /* RF_H */
