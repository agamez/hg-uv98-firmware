# sp2_fontflash.py - minimal SPI2 slave model for the HG-UV98 external font
# flash (session 47). Harness-side synthesized peripheral (followup sec4 item
# 3): the real chip's content is unavailable; bytes read as 0xFF (erased)
# until the synthesized glyph image lands.
#
# Handles the polls the firmware uses:
#   SR (0x08): TXE (bit1) always set; RXNE (bit0) set after each DR write
#              (full-duplex byte transfer).
#   DR (0x0C): writes feed the command state machine (0x03 READ + 24-bit
#              address, MSB first; 0x9F JEDEC ID); reads return the transfer
#              byte (flash content in data phase, ID bytes for 0x9F).
if request.IsInit:
    phase = 0          # 0 = idle/opcode, 1..3 = address bytes, 4 = data
    op = 0
    addr = 0
    txbyte = 0xFF
    id_bytes = [0xEF, 0x40, 0x17]   # plausible JEDEC ID (W25Q64-class)
    id_idx = 0
elif request.IsRead:
    if request.Offset == 0x08:
        request.Value = 0x0003        # TXE | RXNE
    elif request.Offset == 0x0C:
        request.Value = txbyte & 0xFF
    elif request.Offset == 0x00:      # CR1 readback
        request.Value = 0x0000
    else:
        request.Value = 0x0000
elif request.IsWrite:
    if request.Offset == 0x0C:
        b = request.Value & 0xFF
        if phase == 0:
            op = b
            if op == 0x03:
                phase = 1
                addr = 0
            elif op == 0x9F:
                phase = 4
                id_idx = 0
                txbyte = id_bytes[0]
            else:
                phase = 0
        elif phase >= 1 and phase <= 3:
            addr = (addr << 8) | b
            if phase == 3:
                phase = 4
                txbyte = 0xFF          # first data byte served on next read
            else:
                phase += 1
        else:
            # data phase: this write is the MCU's dummy byte; the read side
            # already holds the served byte. Advance the address.
            if op == 0x9F:
                id_idx = (id_idx + 1) % 3
                txbyte = id_bytes[id_idx]
            else:
                addr = (addr + 1) & 0xFFFFFF
                txbyte = 0xFF          # erased-flash content
    # CR1/CR2 writes ignored
