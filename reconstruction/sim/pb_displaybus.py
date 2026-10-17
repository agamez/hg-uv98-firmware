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
        request.Value = ((globals().get('EE_SDA_BIT', 1) & 1) << 9)
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
        seq += 1
        f.write('%d CRH %08x\n' % (seq, crh))
    elif request.Offset == 0x0C:
        odr = v & 0xFFFF
        seq += 1
        f.write('%d ODR %04x\n' % (seq, odr))
    elif request.Offset == 0x10:
        odr = (odr | (v & 0xFFFF)) & ~((v >> 16) & 0xFFFF) & 0xFFFF
        seq += 1
        f.write('%d BSRR %08x %04x\n' % (seq, v, odr))
    elif request.Offset == 0x14:
        odr = odr & ~(v & 0xFFFF) & 0xFFFF
        seq += 1
        f.write('%d BRR %04x %04x\n' % (seq, v, odr))
    f.flush()
