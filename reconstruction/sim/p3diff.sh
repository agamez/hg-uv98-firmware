#!/usr/bin/env bash
# p3diff.sh NAME ORIG_RESC RECON_RESC — the one-command differential driver
# (session 69 new method): runs the orig + recon scenarios fresh-EE
# sequentially (the shared pb_stream.log forbids parallel runs), copies both
# streams, greps the resc markers, and prints the prefix match + the first
# divergence context.
set -u
NAME="$1"; ORIG="$2"; RECON="$3"
RUN=/home/sslayer/hguv98-sim/run
RENODE="$HOME/tools/renode_1.17.0-portable/renode"

rm -f /home/sslayer/hguv98-sim/ee_image.bin /home/sslayer/hguv98-sim/pb_stream.log
timeout 1400 "$RENODE" --console -e "i @$ORIG" > "$RUN/${NAME}_orig.log" 2>&1
cp /home/sslayer/hguv98-sim/pb_stream.log "$RUN/${NAME}_orig.stream"

rm -f /home/sslayer/hguv98-sim/ee_image.bin /home/sslayer/hguv98-sim/pb_stream.log
timeout 1400 "$RENODE" --console -e "i @$RECON" > "$RUN/${NAME}_rec.log" 2>&1
cp /home/sslayer/hguv98-sim/pb_stream.log "$RUN/${NAME}_rec.stream"

echo "--- markers (orig | rec) ---"
grep -hE "scr=" "$RUN/${NAME}_orig.log" | sed 's/^/orig /'
grep -hE "scr=" "$RUN/${NAME}_rec.log"   | sed 's/^/rec  /'
python3 /home/sslayer/hguv98-sim/run/p3diff_cmp.py "$RUN/${NAME}_orig.stream" "$RUN/${NAME}_rec.stream"
