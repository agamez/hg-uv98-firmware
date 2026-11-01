#!/usr/bin/env python3
"""peer_tx.py LOG — extract the USART2 TX observable + the fault registers
from a Renode console log of peer-8051.resc / peer-recon.resc.
TX lines:  [INFO] usart2: [host: ... |virt: ...] <chars>
Faults:    the monitor echo lines after the 'fault registers' echo
(the ReadDoubleWord results print as bare 0xNNNNNNNN value lines)."""
import re
import sys

tx = []
virt = []
for ln in open(sys.argv[1], encoding='utf-8', errors='replace'):
    m = re.search(r"\[INFO\] usart2: \[host: [^\]]*\|virt: ([^]]+)\] (.*)$", ln)
    if m:
        virt.append(m.group(1).strip())
        tx.append(m.group(2))
print(f"TX bursts: {len(tx)}")
for v, t in zip(virt, tx):
    print(f"  @virt {v}: {t!r}")
# the fault-register echoes: the last two bare 0xxxxxxxxx lines in the file
vals = re.findall(r"^\s*(0x[0-9A-Fa-f]{8})\s*$", open(sys.argv[1], encoding='utf-8', errors='replace').read(), re.M)
print(f"last bare word values (the OSCtxSwCtr/OSTime/fault echoes): {vals[-6:]}")
