# pb_displaybus.py - GPIOB substitute for the HG-UV98 platform (session 49).
#
# Harness-side synthesized peripheral (followup sec4 item 3; workaround
# register: see below). The stock STM32F1GPIOPort was REMOVED from
# gd32f103.repl in session 47 because the original's software-I2C (DATA=PB9,
# CLK=PA11, peer = an I2C-EEPROM-class settings slave) samples the ACK bit
# on the 9th clock and a floating PB9 IDR read 1 = NACK, stalling boot.
# This model restores the port for the display bus (PB0 WR-strobe,
# PB8-15 data, control lines on other ports) while keeping the ACK
# workaround: IDR reads return 0 for every pin (externally pulled low).
#
# Everything else is a faithful register-level GPIOB: CRL/CRH/ODR hold
# written state, BSRR/BRR perform their atomic set/reset on ODR, and every
# ODR/BSRR/BRR/CRL/CRH write is logged to a capture file so the display
# stream can be diffed between the original and the reconstruction (CK4).
#
# Capture file: /tmp/opencode/pb_stream.log (truncated at peripheral init,
# one line per bus write; flushed per line - the boot-time volume is small).
if request.IsInit:
    crl = 0x44444444      # reset value: floating inputs
    crh = 0x44444444
    odr = 0x0000
    seq = 0
    f = open('/tmp/opencode/pb_stream.log', 'w')
    f.write('# seq CRL CRH ODR BSRR BRR  (values after the write)\n')
    f.flush()
elif request.IsRead:
    if request.Offset == 0x00:
        request.Value = crl
    elif request.Offset == 0x04:
        request.Value = crh
    elif request.Offset == 0x08:
        # IDR: all pins externally low EXCEPT PB9 (SDA-in), which mirrors the
        # I2C slave's output (EE_SDA_BIT published by p1_gpioa.py; 0 = ACK or
        # a data 0, 1 = released). The session-47 always-ACK semantics are
        # preserved: an unaddressed slave leaves bit9 released (0-read).
        m = emulationManager.CurrentEmulation.Machines[0]
        request.Value = (m['sysbus'].ReadByte(0x2000F800) & 1) << 9
    elif request.Offset == 0x0C:
        request.Value = odr
    else:
        request.Value = 0x00000000    # LCKR etc.
elif request.IsWrite:
    v = request.Value
    if request.Offset == 0x00:
        crl = v
        seq += 1
        f.write('%d CRL %08x\n' % (seq, crl))
    elif request.Offset == 0x04:
        crh = v
        # publish the PB9 direction for the I2C slave in p1_gpioa.py:
        # the PB9 CRH nibble = bits 4-7 of CRH (the pin (9-8) * 4):
        # 0x3 = the master output (push-pull), anything else (0x4 = the
        # floating input) = the master released the line and the slave's
        # EE_SDA_BIT is the line state.
        m = emulationManager.CurrentEmulation.Machines[0]
        m['sysbus'].WriteByte(0x2000F801, 1 if ((v >> 4) & 0xF) == 0x3 else 0)
        seq += 1
        f.write('%d CRH %08x\n' % (seq, crh))
    elif request.Offset == 0x0C:
        odr = v & 0xFFFF
        seq += 1
        f.write('%d ODR %04x\n' % (seq, odr))
        i2c_sda_event(m)
    elif request.Offset == 0x10:
        odr = (odr | (v & 0xFFFF)) & ~((v >> 16) & 0xFFFF) & 0xFFFF
        seq += 1
        f.write('%d BSRR %08x %04x\n' % (seq, v, odr))
        i2c_sda_event(m)
    elif request.Offset == 0x14:
        odr = odr & ~(v & 0xFFFF) & 0xFFFF
        seq += 1
        f.write('%d BRR %04x %04x\n' % (seq, v, odr))
        i2c_sda_event(m)
    f.flush()

def i2c_sda_event(m):
    """The SDA changed (this write): if SCL (PA11, bit 11) is HIGH this is an
    I2C START (SDA=0) or STOP (SDA=1). Publish to the bridge scratch:
    0x2000F802 = the SDA level, 0x2000F803 = the event pending flag."""
    # only meaningful while the MASTER drives PB9 (the output mode); during
    # the ACK/read phases the master released the line and the stale ODR is
    # not the line state.
    if m['sysbus'].ReadByte(0x2000F801) == 1 and (m['sysbus'].ReadDoubleWord(0x4001080C) >> 11) & 1:
        m['sysbus'].WriteByte(0x2000F802, (odr >> 9) & 1)
        m['sysbus'].WriteByte(0x2000F803, 1)
        globals()['i2c_ev_n'] = globals().get('i2c_ev_n', 0) + 1
        if globals()['i2c_ev_n'] < 40:
            df = open('/tmp/opencode/i2c_ev.log', 'a')
            df.write('EVENT n=%d sda=%d odr=%04x\n' % (globals()['i2c_ev_n'], (odr >> 9) & 1, odr))
            df.close()
