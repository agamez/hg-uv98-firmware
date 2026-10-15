#!/usr/bin/env python3
"""Disassemble a range of the original 190416 image (Thumb-2)."""
import sys, capstone

ORIG = '/home/sslayer/Proyectos/HG-UV98/firmware/MAIN-MCU-HG-UV98-V1.0.1-190416.bin'
data = open(ORIG, 'rb').read()

def show(start, end):
    md = capstone.Cs(capstone.CS_ARCH_ARM, capstone.CS_MODE_THUMB)
    md.detail = False
    off = start - 0x08000000
    n = 0
    for i in md.disasm(data[off:end - 0x08000000], start):
        print(f"0x{i.address:08X}: {i.mnemonic:<8} {i.op_str}")
        n += 1
        if i.mnemonic in ('bx', 'pop') and 'pc' in i.op_str:
            break

if __name__ == '__main__':
    a, b = int(sys.argv[1], 0), int(sys.argv[2], 0)
    show(a, b)
