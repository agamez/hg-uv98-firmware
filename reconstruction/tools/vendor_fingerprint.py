#!/usr/bin/env python3
"""Vendor-fingerprint probe (constant-cluster edition).

The original firmware was built with armcc, the vendored SPL is compiled here
with GCC — code bytes differ even for identical C. Constants, however, are
compiler-invariant: the same register writes materialize the same immediates
and literal-pool words. For every vendored function this tool extracts:

  - all immediates shown by objdump -d as `@ 0x...` annotations
  - all 32-bit literal-pool words (word-aligned values >= 0x100 in .text)

and searches the original image for a window (±256 B) where at least
MIN_CO (default 3) distinctive constants co-occur. Co-occurrence of 3+
function-specific constants is strong evidence the original embedded that
vendor function nearby.

Output: analysis/vendor_const_hits.json + console report.
"""
import json, re, subprocess, glob, os, sys
from collections import defaultdict

TC = '/tmp/opencode/arm-tc/usr/bin'
ROOT = os.path.dirname(os.path.abspath(__file__))
ORIG = os.path.join(ROOT, '..', '..', 'firmware', 'MAIN-MCU-HG-UV98-V1.0.1-190416.bin')
INC = ['-Ivendor/gd32f10x/CMSIS', '-Ivendor/gd32f10x/CMSIS/GD/GD32F10x/Include',
       '-Ivendor/gd32f10x/conf', '-Ivendor/gd32f10x/SPL/Include']
WINDOW = 256
MIN_CO = 3

orig = open(ORIG, 'rb').read()

def func_consts(obj):
    """{name: set(constants)} for every FUNC symbol."""
    dis = subprocess.run([f'{TC}/arm-none-eabi-objdump', '-d', obj],
                         capture_output=True, text=True).stdout
    secs = subprocess.run([f'{TC}/arm-none-eabi-objdump', '-s', '-j', '.text', obj],
                          capture_output=True, text=True).stdout
    text = b''
    for line in secs.splitlines():
        toks = line.split()
        if len(toks) >= 2 and re.fullmatch(r'[0-9a-f]+', toks[0]):
            hx = [t for t in toks[1:] if re.fullmatch(r'[0-9a-f]+', t) and len(t) % 2 == 0]
            if hx:
                text += bytes.fromhex(''.join(hx))
    syms = subprocess.run([f'{TC}/arm-none-eabi-nm', '--print-size', obj],
                          capture_output=True, text=True).stdout
    addr2sym = []
    for line in syms.splitlines():
        p = line.split()
        if len(p) == 4 and p[2] in ('T', 't'):
            addr2sym.append((int(p[0], 16), int(p[1], 16), p[3]))
    addr2sym.sort()
    res = defaultdict(set)
    # immediates per function: walk disassembly, attribute @ 0x... to last seen symbol
    cur = None
    for line in dis.splitlines():
        m = re.match(r'^([0-9a-f]+) <(.+)>:', line)
        if m:
            cur = m.group(2)
            continue
        m = re.search(r'@\s*0x([0-9a-f]+)$', line)
        if cur and m:
            v = int(m.group(1), 16)
            if v >= 0x100:
                res[cur].add(v)
    # literal-pool words: whole .text, attributed to the enclosing function
    for off in range(0, len(text) - 3, 4):
        v = int.from_bytes(text[off:off + 4], 'little')
        if v >= 0x100:
            owner = None
            for a, s, n in addr2sym:
                if a <= off < a + s:
                    owner = n
                    break
            if owner:
                res[owner].add(v)
    return res

def main():
    srcs = (glob.glob('vendor/gd32f10x/SPL/Source/*.c')
            + ['vendor/gd32f10x/CMSIS/GD/GD32F10x/Source/system_gd32f10x.c'])
    srcs = [s for s in srcs if 'enet' not in s]
    # pre-index the original image: for every 32-bit word value, the addresses it occurs at
    occ = defaultdict(list)
    for i in range(0, len(orig) - 3):
        v = int.from_bytes(orig[i:i + 4], 'little')
        if v >= 0x100:
            occ[v].append(i)
    report = []
    for src in srcs:
        obj = '/tmp/probe.o'
        r = subprocess.run([f'{TC}/arm-none-eabi-gcc', '-mcpu=cortex-m3', '-mthumb',
                            '-O2', '-DGD32F10X_HD', *INC, '-c', src, '-o', obj],
                           capture_output=True, text=True)
        if r.returncode != 0:
            print(f'  !! compile failed: {src}', file=sys.stderr)
            continue
        for fname, consts in func_consts(obj).items():
            if len(consts) < MIN_CO or fname in ('SystemCoreClockUpdate',):
                continue
            # candidate anchors: constants that occur in the image at all
            per_const = {c: occ[c] for c in consts if c in occ}
            if len(per_const) < MIN_CO:
                continue
            # find windows with >= MIN_CO distinct constants
            best = defaultdict(set)
            for c, addrs in per_const.items():
                for a in addrs:
                    best[a // WINDOW].add(c)
            for bucket, cs in best.items():
                if len(cs) >= MIN_CO:
                    report.append({'src': os.path.basename(src), 'func': fname,
                                   'img_addr': f'0x{0x08000000 + bucket * WINDOW:08X}',
                                   'n_consts': len(cs), 'of': len(consts),
                                   'consts': [f'0x{c:X}' for c in sorted(cs)][:8]})
    report.sort(key=lambda r: (-r['n_consts'], r['img_addr']))
    out = os.path.join(ROOT, '..', '..', 'analysis', 'vendor_const_hits.json')
    json.dump(report, open(out, 'w'), indent=1)
    print(f'=== {len(report)} constant-cluster hits (>= {MIN_CO} co-occurring vendor constants) ===')
    for r in report[:50]:
        print(f"  {r['src']:<26} {r['func']:<30} @ {r['img_addr']}  "
              f"{r['n_consts']}/{r['of']} consts  {r['consts']}")

if __name__ == '__main__':
    main()
