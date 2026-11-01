#!/usr/bin/env python3
"""ck2mxcmp.py ORIG_LOG REC_LOG — the CK2 matrix checkpoint comparator.
Each '=== CK2CHK <family> BEGIN ===' block = 49 dump lines '0xNN'
(rxBuf[0..15], rxBuf2[0..15], rxBuf3[0..15], the flag byte)."""
import re
import sys


def parse(path):
    fams = {}
    cur = None
    for ln in open(path, encoding='utf-8', errors='replace'):
        m = re.match(r"=== CK2CHK (\S+) BEGIN ===", ln.strip())
        if m:
            cur = m.group(1)
            fams[cur] = []
            continue
        if cur is not None:
            m = re.match(r"0x([0-9A-Fa-f]{2})\s*$", ln.strip())
            if m:
                fams[cur].append(int(m.group(1), 16))
                if len(fams[cur]) == 49:
                    cur = None
    return fams


def main():
    o = parse(sys.argv[1])
    r = parse(sys.argv[2])
    names = ['rxBuf[%d]' % i for i in range(16)] + \
            ['rxBuf2[%d]' % i for i in range(16)] + \
            ['rxBuf3[%d]' % i for i in range(16)] + ['FLAG']
    tot = same = 0
    diffs = []
    for fam in sorted(o):
        if fam not in r:
            print(f"MISSING family {fam} on the rec side")
            continue
        ov, rv = o[fam], r[fam]
        for i in range(min(len(ov), len(rv), 49)):
            tot += 1
            if ov[i] == rv[i]:
                same += 1
            else:
                diffs.append((fam, i, names[i] if i < 49 else '?', ov[i], rv[i]))
    print(f"{same}/{tot} observables identical")
    for fam, i, nm, ov, rv in diffs:
        print(f"  DIFF {fam} #{i:2d} {nm}: orig=0x{ov:02X} rec=0x{rv:02X}")
    return 0


if __name__ == '__main__':
    sys.exit(main())
