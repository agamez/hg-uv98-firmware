#!/usr/bin/env python3
"""bldec.py - CORRECT Thumb BL decoder for the HG-UV98 firmware image.

Encoding (ARMv7-M, T1):
  hw1: 11110 S imm10
  hw2: 11 J1 1 J2 imm11        (BL; bit12=1; BLX has bit12=0)
  I1 = NOT(J1 ^ S), I2 = NOT(J2 ^ S)
  imm32 = SignExtend(S:I1:I2:imm10:imm11:'0')
  target = PC + imm32, PC = addr_of_bl + 4
The immediate is byte-granular already (the trailing '0' is the halfword bit);
do NOT multiply by 2 again and do NOT shift imm10 by 13.
"""
import sys

FW = '/home/sslayer/Proyectos/HG-UV98/firmware/MAIN-MCU-HG-UV98-V1.0.1-190416.bin'
BASE = 0x08004000
N = 273303


def img():
    with open(FW, 'rb') as f:
        return f.read()


def a2o(addr):
    a = addr - BASE
    return a if 0 <= a < N else None


def decode_bl(addr, b):
    """Return (target, 'BL'|'BLX') or None if not a BL/BLX pair at addr."""
    o = a2o(addr)
    if o is None or o + 4 > N:
        return None
    hw1 = b[o] | (b[o + 1] << 8)
    hw2 = b[o + 2] | (b[o + 3] << 8)
    if (hw1 & 0xF800) != 0xF000:
        return None
    if (hw2 & 0x1800) != 0x1000 or (hw2 & 0xE000) != 0xC000:
        # tail must be 11 x 1 x .... ; BL: bit14=1? no --
        # BL tail = 11 J1 1 J2 imm11  -> bits15:14=11, bit12=1
        # BLX tail= 11 J1 0 J2 imm11H -> bits15:14=11, bit12=0
        pass
    blx = (hw2 & 0x1000) == 0
    if (hw2 & 0xC000) != 0xC000:
        return None
    S = (hw1 >> 10) & 1
    imm10 = hw1 & 0x3FF
    J1 = (hw2 >> 13) & 1
    J2 = (hw2 >> 11) & 1
    imm11 = hw2 & 0x7FF
    I1 = (~(J1 ^ S)) & 1
    I2 = (~(J2 ^ S)) & 1
    imm = (S << 23) | (I1 << 22) | (I2 << 21) | (imm10 << 11) | imm11
    if S:
        imm -= 1 << 24
    off = imm * 2
    if blx:
        pc = (addr + 4) & ~3
        tgt = (pc + off) & 0xFFFFFFFF  # H=imm11 bit0 -> ARM state; rare here
        tgt &= ~1
    else:
        tgt = (addr + 4 + off) & 0xFFFFFFFF
    return tgt, ('BLX' if blx else 'BL')


def main():
    b = img()
    mode = sys.argv[1] if len(sys.argv) > 1 else 'scan'

    if mode == 'at':  # bldec.py at 0x8022ae6 [more...]
        for a in sys.argv[2:]:
            addr = int(a, 16)
            r = decode_bl(addr, b)
            print(f"{a}: {r if r else 'NOT A BL/BLX PAIR'}")
        return

    if mode == 'callers':  # bldec.py callers 0x08022a98
        tgt = int(sys.argv[2], 16) & ~1
        n = 0
        for i in range(0, N - 3, 2):
            r = decode_bl(BASE + i, b)
            if r and r[0] == tgt:
                print(f"caller {BASE + i:#x} ({r[1]}) -> {r[0]:#x}")
                n += 1
        print(f"total {n}")
        return

    # scan: sanity-verify known pairs then dump all BL targets
    print("== verification against known rf_read (0x0802692E) callers ==")
    for a in (0x8025b42, 0x8025c5a, 0x8025eee):
        r = decode_bl(a, b)
        print(f"  {a:#x} -> {r}")
    if mode == 'verify':
        return
    print("== all BLs targeting 0x08022a98 ==")
    decode_bl(0, b)  # noqa
    tgt = 0x08022a98
    n = 0
    for i in range(0, N - 3, 2):
        r = decode_bl(BASE + i, b)
        if r and r[0] == tgt:
            print(f"caller {BASE + i:#x}")
            n += 1
    print(f"total {n}")


if __name__ == '__main__':
    main()
