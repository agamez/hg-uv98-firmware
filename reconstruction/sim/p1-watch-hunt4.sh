#!/bin/bash
# p1-watch-hunt.sh — attach gdb-multiarch, watch the flag-bank bytes, collect the writer PCs.
RENODE_DIR=~/tools/renode_1.17.0-portable
GDB=/tmp/opencode/gdb-ma/usr/bin/gdb-multiarch
cd "$RENODE_DIR"
(sleep 220; echo quit) | timeout 240 ./renode --console --disable-xwt /home/sslayer/Proyectos/HG-UV98/reconstruction/sim/p1-watch-orig.resc > /tmp/opencode/p1_watch_renode.log 2>&1 &
RNODE=$!
sleep 6
"$GDB" -q -ex "set architecture arm" \
  -ex "target remote :3334" \
  -ex "set confirm off" \
  -ex "watch *(unsigned char*)0x220a12c8" \
  -ex "watch *(unsigned char*)0x220a12d0" \
  -ex "info breakpoints" \
  -ex "continue" \
  -ex "info registers pc" \
  -ex "continue" \
  -ex "info registers pc" \
  -ex "continue" \
  -ex "info registers pc" \
  -ex "continue" \
  -ex "info registers pc" \
  -ex "continue" \
  -ex "info registers pc" \
  -ex "continue" \
  -ex "info registers pc" \
  -ex "continue" \
  -ex "info registers pc" \
  -ex "continue" \
  -ex "info registers pc" \
  -ex "continue" \
  -ex "info registers pc" \
  -ex "continue" \
  -ex "info registers pc" \
  -ex "continue" \
  -ex "info registers pc" \
  -ex "continue" \
  -ex "info registers pc" \
  -ex "kill" \
  -ex "detach" \
  -ex "quit" > /tmp/opencode/p1_watch_gdb.log 2>&1
kill $RNODE 2>/dev/null
wait $RNODE 2>/dev/null
echo "=== gdb log (watchpoint events) ==="
grep -E "atchpoint|alue =|0x08" /tmp/opencode/p1_watch_gdb.log | head -60
