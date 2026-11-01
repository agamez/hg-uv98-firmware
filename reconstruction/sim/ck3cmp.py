#!/usr/bin/env python3
"""ck3cmp.py ORIG_TRACE RECON_LOG — the CK3 (cs,reg,val) triple comparator.

Orig side: analysis/traces/ck3_rf_orig.txt
  RFW cs=1 buf@[20005134]=30 0 1 0   -> (W, cs, reg=bs[0], val=bs[2]|bs[3]<<8)
  RFRD r0=1 r1=30                    -> (R, cs, reg)
  (SENDN lines are the wire view of the same write — skipped)
Recon side: the Renode console log of ck3-recon.resc
  RFW cs=1 reg=48 val=0x1e3c         -> (W, cs, reg, val)
  RFRD cs=1 reg=30                   -> (R, cs, reg)
"""
import re
import sys


def parse_orig(path):
    ev = []
    for ln in open(path, encoding='utf-8', errors='replace'):
        m = re.match(r"RFW cs=(\d+) buf@\[[0-9A-Fa-f]+\]=([0-9A-Fa-f ]+)$", ln.strip())
        if m:
            bs = [int(x, 16) for x in m.group(2).split()]
            ev.append(('W', int(m.group(1)), bs[0], bs[2] | (bs[3] << 8)))
            continue
        m = re.match(r"RFRD r0=(\d+) r1=([0-9A-Fa-f]+)$", ln.strip())
        if m:
            ev.append(('R', int(m.group(1)), int(m.group(2), 16), None))
    return ev


def parse_rec(path):
    ev = []
    for ln in open(path, encoding='utf-8', errors='replace'):
        # the hook prints decimal regs and 'val=0x' + hex() = "val=0x0x1e3cL"
        m = re.search(r"RFW cs=(\d+) reg=(\d+) val=0x(?:0x)?([0-9A-Fa-f]+)L?", ln)
        if m:
            ev.append(('W', int(m.group(1)), int(m.group(2)), int(m.group(3), 16)))
            continue
        m = re.search(r"RFRD cs=(\d+) reg=(\d+)", ln)
        if m:
            ev.append(('R', int(m.group(1)), int(m.group(2)), None))
    return ev


def main():
    o = parse_orig(sys.argv[1])
    r = parse_rec(sys.argv[2])
    print(f"orig {len(o)} events, rec {len(r)} events")
    n = min(len(o), len(r))
    for i in range(n):
        if o[i] != r[i]:
            print(f"FIRST DIFF @#{i}: orig {o[i]} vs rec {r[i]}")
            for j in range(max(0, i - 2), min(n, i + 3)):
                print(f"  #{j}: orig {o[j]} | rec {r[j]}")
            return 1
    if len(o) != len(r):
        print(f"LENGTH DIFF (the first {n} events identical)")
        for j in range(n, min(len(o), len(r) if len(r) > len(o) else len(o)))[  :0] or []:
            pass
        tail = o[n:] if len(o) > len(r) else r[n:]
        who = 'orig' if len(o) > len(r) else 'rec'
        print(f"  extra {who} events: {tail[:8]}{'...' if len(tail) > 8 else ''}")
        return 1
    print(f"ALL {n} EVENTS IDENTICAL")
    return 0


if __name__ == '__main__':
    sys.exit(main())
