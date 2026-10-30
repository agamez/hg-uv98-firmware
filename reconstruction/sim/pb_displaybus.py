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
# Capture file: /home/sslayer/hguv98-sim/pb_stream.log (truncated at peripheral init,
# one line per bus write; flushed per line - the boot-time volume is small).
if request.IsInit:
    crl = 0x44444444      # reset value: floating inputs
    crh = 0x44444444
    odr = 0x0000
    seq = 0
    i2c_last_lvl = 1      # the SDA level at the last published event (idle high)
    f = open('/home/sslayer/hguv98-sim/pb_stream.log', 'w')
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
    m = emulationManager.CurrentEmulation.Machines[0]
    globals()['pb_wseq'] = globals().get('pb_wseq', 0) + 1
    m['sysbus'].WriteByte(0x2000F810, globals()['pb_wseq'] & 0xFF)
    if request.Offset == 0x00:
        crl = v
        seq += 1
        f.write('%d CRL %08x w%d\n' % (seq, crl, pb_wseq))
    elif request.Offset == 0x04:
        # publish the PB9 direction for the I2C slave in p1_gpioa.py ONLY on
        # an actual nibble transition [session 60 fix]: the PB9 CRH nibble =
        # bits 4-7 of CRH (the pin (9-8) * 4): 0x3 = the master output
        # (push-pull), anything else (0x4 = the floating input) = the master
        # released the line and the slave's EE_SDA_BIT is the line state.
        # The display scan rewrites the same CRH (nibble 3 -> 3) constantly;
        # the old unconditional publish let a scan write flip 0x2000F801 back
        # to 1 DURING the I2C input phases (the ACK clock), so the resolver
        # returned the master's stale ODR instead of the slave's ACK.
        new_nib = (v >> 4) & 0xF
        old_nib = (crh >> 4) & 0xF
        crh = v
        if new_nib == 0x3 and old_nib != 0x3:
            m['sysbus'].WriteByte(0x2000F801, 1)
        elif new_nib != 0x3 and old_nib == 0x3:
            m['sysbus'].WriteByte(0x2000F801, 0)
        seq += 1
        f.write('%d CRH %08x w%d\n' % (seq, crh, pb_wseq))
    elif request.Offset == 0x0C:
        odr = v & 0xFFFF
        seq += 1
        f.write('%d ODR %04x w%d\n' % (seq, odr, pb_wseq))
        i2c_sda_event(m)
    elif request.Offset == 0x10:
        odr = (odr | (v & 0xFFFF)) & ~((v >> 16) & 0xFFFF) & 0xFFFF
        seq += 1
        f.write('%d BSRR %08x %04x w%d\n' % (seq, v, odr, pb_wseq))
        i2c_sda_event(m)
    elif request.Offset == 0x14:
        odr = odr & ~(v & 0xFFFF) & 0xFFFF
        seq += 1
        f.write('%d BRR %04x %04x w%d\n' % (seq, v, odr, pb_wseq))
        i2c_sda_event(m)
    # [session 60 fix] the master-out SDA mirror for the I2C slave's
    # resolver: published INSIDE this write handler (same engine, same
    # ordering as the odr update) - the slave reads 0x2000F804 instead of
    # the lagging cross-peripheral ODR read.
    m['sysbus'].WriteByte(0x2000F804, (odr >> 9) & 1)
    f.flush()

def i2c_sda_event(m):
    """Track the master-driven SDA level and publish I2C START/STOP events:
    a START (SDA 1->0) or a STOP (SDA 0->1) is a level change while the SCL
    is HIGH. Published to the bridge scratch: 0x2000F802 = the SDA level,
    0x2000F803 = the event pending flag.
    [session 60] the level tracking must run on EVERY master-driven write
    regardless of the SCL: the ack-tail's SDA=1 write happens while the SCL
    is LOW (not a STOP), and if it is not tracked the NEXT START's 1->0
    looks like no change and the slave goes deaf (the observed late-era
    state=IDLE-forever defect).
    [session 60 fix] the SCL level = the GPIOA's scratch mirror at
    0x2000F805 (published inside the GPIOA's own write handler) - the old
    direct cross-peripheral read of 0x4001080C lags the GPIOA state."""
    if m['sysbus'].ReadByte(0x2000F801) != 1:
        return          # the input phase: the master's ODR = not the line
    lvl = (odr >> 9) & 1
    prev = globals().get('i2c_last_lvl')
    globals()['i2c_last_lvl'] = lvl
    if prev == lvl:
        return          # no level change: no START/STOP possible
    if not (m['sysbus'].ReadByte(0x2000F805) & 1):
        return          # the level changed while the SCL was LOW: a data bit
    m['sysbus'].WriteByte(0x2000F802, lvl)
    m['sysbus'].WriteByte(0x2000F803, 1)
    globals()['i2c_ev_n'] = globals().get('i2c_ev_n', 0) + 1
    if globals()['i2c_ev_n'] < 120:
        df = open('/home/sslayer/hguv98-sim/i2c_ev.log', 'a')
        df.write('EVENT n=%d sda=%d\n' % (globals()['i2c_ev_n'], lvl))
        df.close()
