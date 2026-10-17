/* I2C EEPROM bit-bang driver — the original's settings-storage link.
 * SDA = PB9, SCL = PA11, a 24Cxx-class slave at 0xA0/0xA1.
 * Recovered in session 51 from 0x0802846A/0x08028448/0x080284F2/
 * 0x080285B6/0x08028532/0x08028566/0x08028666/0x080286C0/0x0802873C. */
#ifndef EEPROM_H
#define EEPROM_H

#include <stdint.h>

void eeprom_write(uint16_t addr, uint8_t count, const uint8_t *buf);
void eeprom_read(uint16_t addr, uint8_t count, uint8_t *buf);

#endif /* EEPROM_H */
