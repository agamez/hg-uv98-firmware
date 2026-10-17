# p1_gpioa.py - GPIOA passthrough + bit-bang I2C EEPROM slave (goal phase 1).
#
# WHY: the original's settings EEPROM is a bit-bang I2C slave on SDA=PB9 /
# SCL=PA11 (device 0xA0/0xA1, 2-byte word address, 24C32-class). The harness
# had NO slave: every ee_read returned 0x00 (the displaybus pins IDR=0), so
# the boot settings were sql=0 / scan-on and the squelch-open RX sessions
# re-armed forever (FLAG_BANK+2 bit2) - the "boot state gate" root cause
# (session 59 in docs/RESEARCH_LOG.md).
#
# This model REPLACES the stock GPIOA peripheral at 0x40010800 with:
#  1. a faithful register-level GPIOA (CRL/CRH/ODR/BSRR/BRR mirrors), and
#  2. a write-forwarder to the REAL GPIO peripheral mapped offscreen at
#     0x50010800 (so EXTI/AFIO wiring and pin semantics stay real), and
#  3. the I2C slave state machine, driven by the SCL (PA11, bit 0x800) edges
#     it sees on its own writes; the SDA (PB9, bit 0x200) is sampled through
#     the sysbus from the displaybus model's ODR mirror.
#
# Slave output (ACK + read data) is published via the engine-global EE_SDA_BIT,
# which pb_displaybus.py mirrors into IDR bit 9 (SDA-in). EE_SDA_BIT = 0 =
# the slave drives SDA low (an ACK, or a data 0); 1 = released.
#
# NOTE: the pythonEngine namespace is SHARED between the python peripherals,
# so ALL state here is ga_/i2c_ prefixed to avoid the displaybus collisions.
#
# EEPROM image: /tmp/opencode/ee_image.bin (32768 bytes). If absent it is
# created all-0xFF (erased device), so the vendor's own first-boot path
# programs its factory defaults - the sane-settings unlock happens through
# the vendor's own logic, not harness magic. Writes are persisted per byte.
if request.IsInit:
    ga_crl = 0x44444444
    ga_crh = 0x44444444
    ga_odr = 0x0000
    i2c_scl_prev = 0
    i2c_sda_last = 1         # last SDA level seen at an SCL edge (idle high)
    i2c_state = 'IDLE'       # IDLE ADDR AHI ALO DATAW DATAR
    i2c_bits = 0
    i2c_shift = 0
    i2c_word = 0
    i2c_outbyte = 0
    i2c_outbit = 0
    i2c_mem = None
    globals()['EE_SDA_BIT'] = 1
    import os
    if not os.path.exists('/tmp/opencode/ee_image.bin'):
        f = open('/tmp/opencode/ee_image.bin', 'wb')
        f.write(b'\xff' * 32768)
        f.close()
elif request.IsRead:
    m = emulationManager.CurrentEmulation.Machines[0]
    if request.Offset == 0x00:
        request.Value = ga_crl
    elif request.Offset == 0x04:
        request.Value = ga_crh
    elif request.Offset == 0x08:
        # IDR: real pins (all floating low on this board)
        request.Value = m['sysbus'].ReadDoubleWord(0x50010808) & 0xFFFF
    elif request.Offset == 0x0C:
        request.Value = ga_odr
    else:
        request.Value = 0x00000000
elif request.IsWrite:
    v = request.Value
    m = emulationManager.CurrentEmulation.Machines[0]
    if request.Offset == 0x00:
        ga_crl = v
        m['sysbus'].WriteDoubleWord(0x50010800, v)
    elif request.Offset == 0x04:
        ga_crh = v
        m['sysbus'].WriteDoubleWord(0x50010804, v)
    elif request.Offset == 0x0C:
        ga_odr = v & 0xFFFF
        m['sysbus'].WriteDoubleWord(0x5001080C, ga_odr)
        i2c_tick(m)
    elif request.Offset == 0x10:
        ga_odr = (ga_odr | (v & 0xFFFF)) & ~((v >> 16) & 0xFFFF) & 0xFFFF
        m['sysbus'].WriteDoubleWord(0x50010810, v)
        i2c_tick(m)
    elif request.Offset == 0x14:
        ga_odr = ga_odr & ~(v & 0xFFFF) & 0xFFFF
        m['sysbus'].WriteDoubleWord(0x50010814, v)
        i2c_tick(m)

def i2c_tick(m):
    """Called after every ODR-changing GPIOA write: the SCL edge engine."""
    global i2c_scl_prev, i2c_sda_last, i2c_state, i2c_bits, i2c_shift
    global i2c_word, i2c_outbyte, i2c_outbit
    scl = (ga_odr >> 11) & 1
    if scl == i2c_scl_prev:
        return
    i2c_scl_prev = scl
    sda = (m['sysbus'].ReadDoubleWord(0x40010C0C) >> 9) & 1  # displaybus ODR PB9
    if scl == 1:
        # ---- SCL rising edge: the master samples the SDA. ----
        if i2c_state in ('ADDR', 'AHI', 'ALO', 'DATAW'):
            i2c_shift = ((i2c_shift << 1) | sda) & 0xFF
            i2c_bits += 1
            if i2c_bits == 8:
                byte = i2c_shift
                i2c_bits = 0
                i2c_shift = 0
                globals()['EE_SDA_BIT'] = 0      # ACK: drive SDA low on the 9th clock
                if i2c_state == 'ADDR':
                    if (byte & 0xF0) == 0xA0:
                        if (byte & 1) == 0:
                            i2c_state = 'AHI'    # write: expect the word address
                        else:
                            i2c_state = 'DATAR'  # read from the current address
                            i2c_outbyte = ee_read_at(i2c_word)
                            i2c_outbit = 0
                    else:
                        i2c_state = 'IDLE'
                        globals()['EE_SDA_BIT'] = 1
                elif i2c_state == 'AHI':
                    i2c_word = (byte << 8) & 0xFFFF
                    i2c_state = 'ALO'
                elif i2c_state == 'ALO':
                    i2c_word = (i2c_word & 0xFF00) | byte
                    i2c_state = 'DATAW'
                elif i2c_state == 'DATAW':
                    ee_write_at(i2c_word, byte)
                    i2c_word = (i2c_word + 1) & 0xFFFF
        # DATAR: the master samples our EE_SDA_BIT; nothing to do here.
        return
    # ---- SCL falling edge ----
    if sda != i2c_sda_last:
        # SDA moved while SCL was high: START (0) or STOP (1). Both reset us;
        # the EEPROM keeps the current-address pointer across transactions.
        i2c_state = 'IDLE'
        i2c_bits = 0
        i2c_shift = 0
        globals()['EE_SDA_BIT'] = 1
        if sda == 0:
            i2c_state = 'ADDR'   # START: expect the device address
        i2c_sda_last = sda
        return
    i2c_sda_last = sda
    if i2c_state in ('ADDR', 'AHI', 'ALO', 'DATAW'):
        # the ACK was clocked; release SDA
        globals()['EE_SDA_BIT'] = 1
    elif i2c_state == 'DATAR':
        # shift out the next data bit (MSB first)
        globals()['EE_SDA_BIT'] = (i2c_outbyte >> (7 - i2c_outbit)) & 1
        i2c_outbit += 1
        if i2c_outbit == 8:
            i2c_word = (i2c_word + 1) & 0xFFFF
            i2c_outbyte = ee_read_at(i2c_word)
            i2c_outbit = 0

def ee_read_at(addr):
    global i2c_mem
    if i2c_mem is None:
        f = open('/tmp/opencode/ee_image.bin', 'rb')
        i2c_mem = bytearray(f.read())
        f.close()
    return i2c_mem[addr & 0x7FFF]

def ee_write_at(addr, val):
    global i2c_mem
    if i2c_mem is None:
        f = open('/tmp/opencode/ee_image.bin', 'rb')
        i2c_mem = bytearray(f.read())
        f.close()
    i2c_mem[addr & 0x7FFF] = val
    f = open('/tmp/opencode/ee_image.bin', 'r+b')
    f.seek(addr & 0x7FFF)
    f.write(bytes([val]))
    f.close()
