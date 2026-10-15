#!/usr/bin/env python3
"""HG-UV98 main-MCU firmware reconnaissance.

Reproduces the phase-1 findings against the two distributed main-MCU images:
  * image identity, extent, vector table
  * 4 KiB sector map (0xFF density, 0xBEEF density, Thumb decodability)
  * 0xBEEF "placeholder" runs
  * version/protocol string extraction
  * first-derivative byte diff between the two images

Addresses are flash addresses (file offset + 0x08000000).

Usage:
    python3 tools/recon_map.py [path-to-firmware-dir]

Outputs a compact report to stdout. Heavy artifacts (string lists, sector
maps) are written to analysis/ (git-ignored).
"""
import collections
import json
import math
import os
import re
import struct
import sys

BASE = 0x08000000
FLASH_END = 0x08080000
RAM_LO, RAM_HI = 0x20000000, 0x20010000

# STM32F103xE/GD32F103 high-density IRQ names for vector entries [16..75]
IRQ_NAMES = [
    "WWDG", "PVD", "TAMPER", "RTC", "FLASH", "RCC", "EXTI0", "EXTI1",
    "EXTI2", "EXTI3", "EXTI4", "DMA1_Ch1", "DMA1_Ch2", "DMA1_Ch3", "DMA1_Ch4",
    "DMA1_Ch5", "DMA1_Ch6", "DMA1_Ch7", "ADC1_2", "USB_HP_CAN_TX", "USB_LP_CAN_RX0",
    "CAN_RX1", "CAN_SCE", "EXTI9_5", "TIM1_BRK", "TIM1_UP", "TIM1_TRG_COM",
    "TIM1_CC", "TIM2", "TIM3", "TIM4", "I2C1_EV", "I2C1_ER", "I2C2_EV",
    "I2C2_ER", "SPI1", "SPI2", "USART1", "USART2", "USART3", "EXTI15_10",
    "RTC_Alarm", "USBWake", "TIM8_BRK", "TIM8_UP", "TIM8_TRG_COM", "TIM8_CC",
    "ADC3", "FSMC", "SDIO", "TIM5", "SPI3", "UART4", "UART5", "TIM6_DAC",
    "TIM7", "DMA2_Ch1", "DMA2_Ch2", "DMA2_Ch3", "DMA2_Ch4_5",
]

try:
    from capstone import Cs, CS_ARCH_ARM, CS_MODE_THUMB
    _md = Cs(CS_ARCH_ARM, CS_MODE_THUMB)
except ImportError:
    _md = None


def load_images(fwdir):
    names = {
        "190416": os.path.join(fwdir, "MAIN-MCU-HG-UV98-V1.0.1-190416.bin"),
        "KT": os.path.join(fwdir, "V1.0.1_190619-KT.bin"),
    }
    return {k: open(v, "rb").read() for k, v in names.items()}


def vector_table(d):
    """Scan vector table until a non-plausible entry; return list of (idx, value)."""
    entries = []
    for i in range(len(d) // 4):
        v = struct.unpack_from("<I", d, i * 4)[0]
        a = v & ~1
        ok = (BASE <= a < FLASH_END) or (RAM_LO <= a < RAM_HI) or v == 0
        if not ok:
            break
        entries.append(v)
    return entries


def beef_runs(d, minlen=32):
    runs = []
    i = 0
    n = len(d)
    while i < n - 1:
        if d[i] == 0xEF and d[i + 1] == 0xBE:
            j = i
            while j + 1 < n and d[j] == 0xEF and d[j + 1] == 0xBE:
                j += 2
            if j - i >= minlen:
                runs.append((BASE + i, BASE + j, j - i))
            i = j
        else:
            i += 1
    return runs


def sector_map(d, sector=0x1000):
    rows = []
    for base in range(0, len(d), sector):
        chunk = d[base:base + sector]
        n = len(chunk)
        beef = sum(1 for i in range(0, n - 1) if chunk[i] == 0xEF and chunk[i + 1] == 0xBE) // 2
        ff = chunk.count(0xFF)
        cov = 0.0
        if _md:
            cov = sum(i.size for i in _md.disasm(chunk, 0, count=5000)) / n * 100
        rows.append((BASE + base, beef * 100 // n, ff * 100 // n, cov))
    return rows


def strings(d, minlen=4):
    out = []
    cur = bytearray()
    start = 0
    for i, b in enumerate(d):
        if 0x20 <= b < 0x7F:
            if not cur:
                start = i
            cur.append(b)
        else:
            if len(cur) >= minlen:
                out.append((BASE + start, cur.decode("ascii")))
            cur = bytearray()
    if len(cur) >= minlen:
        out.append((BASE + start, cur.decode("ascii")))
    return out


def diff_ranges(a, b, gap=32):
    n = min(len(a), len(b))
    ranges = []
    i = 0
    while i < n:
        if a[i] != b[i]:
            start = i
            last = i
            j = i + 1
            while j < n:
                if a[j] != b[j]:
                    if j - last > gap:
                        break
                    last = j
                j += 1
            ranges.append((BASE + start, BASE + last, last + 1 - start))
            i = last + 1
        else:
            i += 1
    return ranges


def last_non_ff(d):
    last = len(d)
    while last > 0 and d[last - 1] == 0xFF:
        last -= 1
    return last


def report(fwdir):
    imgs = load_images(fwdir)
    out = {"images": {}, "diff": {}}
    for tag, d in imgs.items():
        end = BASE + len(d)
        vt = vector_table(d)
        sp = vt[0] if vt else 0
        reset = vt[1] if len(vt) > 1 else 0
        outside = [(i, hex(v)) for i, v in enumerate(vt)
                   if (v & ~1) >= end and BASE <= (v & ~1) < FLASH_END]
        runs = beef_runs(d)
        beef_bytes = sum(r[2] for r in runs)
        info = {
            "size": len(d),
            "extent": [hex(BASE), hex(end - 1)],
            "initial_sp": hex(sp),
            "reset_vector": hex(reset),
            "reset_inside": (reset & ~1) < end,
            "vector_entries": len(vt),
            "irq_count": max(0, len(vt) - 16),
            "vectors_beyond_file": outside,
            "beef_runs_ge32B": len(runs),
            "beef_bytes": beef_bytes,
            "last_non_ff": hex(BASE + last_non_ff(d) - 1),
        }
        # strings of interest
        ss = strings(d)
        info["strings_total"] = len(ss)
        interesting = [(hex(a), s) for a, s in ss
                       if re.search(r"(V1\.|P3107|P3LL|^\$[A-Z]\d|\bSURE\b|UpData|VERSION)", s)]
        info["strings_interesting"] = interesting[:40]
        out["images"][tag] = info

        os.makedirs("analysis", exist_ok=True)
        with open(f"analysis/strings_{tag}.json", "w") as f:
            json.dump([[hex(a), s] for a, s in ss], f, indent=0)
        with open(f"analysis/sector_map_{tag}.json", "w") as f:
            json.dump([[hex(a), b, f_, round(c, 1)] for a, b, f_, c in sector_map(d)], f, indent=0)

    d1, d2 = imgs["190416"], imgs["KT"]
    n = min(len(d1), len(d2))
    first = next((i for i in range(n) if d1[i] != d2[i]), -1)
    # first divergence AFTER the vector table (vectors differ due to shift)
    post_vt = next((i for i in range(0x130, n) if d1[i] != d2[i]), -1)
    out["diff"] = {
        "compared": n,
        "first_diff": hex(BASE + first) if first >= 0 else None,
        "identical_run_after_vector_table": [hex(BASE + 0x130), hex(BASE + post_vt - 1)] if post_vt >= 0 else None,
        "total_diff_bytes": sum(1 for i in range(n) if d1[i] != d2[i]),
    }

    print(json.dumps(out, indent=1))


if __name__ == "__main__":
    fwdir = sys.argv[1] if len(sys.argv) > 1 else "firmware"
    report(fwdir)
