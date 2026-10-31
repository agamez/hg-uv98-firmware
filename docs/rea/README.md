# REA decompilation artifacts (the [REA-C-VALIDATED] pipeline)

- `dispatch_08019dcc.c` — Ghidra decompilation of the key-dispatch function
  at 0x08019dcc (the function was undiscovered by auto-analysis; created via
  ReaCreateDecomp.java with the Thumb TMode set, then decompiled). The
  ground truth for the key-11/12/13/14/18/19/20/1/2 press handling:
  * every key branch: `func_0x08020c9e()` then the hold counter (u32 at
    r5+0x21) incremented; the key-specific action gated on the counter.
  * keys 11/12: `if (counter > 39) { vtbl[slot*0x10+4](key);
    if (slot[2]) { vtbl[slot*0x10+8](key); slot[2]=0; } }` — the repeat
    fires EVERY pass once the hold exceeds 40.
  * key 13: `if (counter == 40) FUN_08022cb4()` (once).
  * key 14: `if (counter == 40) FUN_08021238(1)` (once).
  * keys 18/19/20: the settings consumers FUN_08021238/FUN_080214e4/
    func_0x0802166c with the per-key arg from the settings shadow.
  * keys 1/2 on screen 0x67: the 0x83 switch at count 40 (the session-61
    model, now C-confirmed).
  * the release tail (key == 0xFF): the flag clears + `counter = 0`.
  * the vtable = `[slot]*0x10` stride, +4 = press, +8 = the gated second
    call; `_DAT_0801a3a4` = the vtable base pointer (runtime-assigned by
    the screen machine), `DAT_0801a390` = the current slot struct.
- `ReaCreateDecomp.java` — the reusable Ghidra headless script: force-create
  a function at any address (Thumb TMode) and print its decompiled C. Usage:
  `analyzeHeadless /tmp/opencode/ghproj hguv98 -process MAIN-MCU-HG-UV98-V1.0.1-190416.bin -noanalysis -scriptPath /tmp/opencode -postScript ReaCreateDecomp.java 0x<addr>`
  (the project at /tmp/opencode/ghproj = disposable; ~40 s per cold import).
- Validated clean (zero divergences, via REA MCP batch_decompile):
  `key_scan` 0x080281A6 (the A/B/C ladders, bands 661..3253 / 671..1751,
  the upper-first pair layout, the TIM2 accessory branch) and
  `key_get_code` 0x080282FC (the maps {10,11,12,13,1,2,3,14}/{4,5,6,0,7,8,9,15}/{18,19,20}, the A→B→C order, the keypad-lock gate).

## Session 69 (cont. 11): the vtable + the key-11/12/14 handler pairs

- **The vtable found in flash at base 0x8040AEC**, stride 0x10, layout
  `[key_index, press_handler, second_handler, once_flag]` (the words = the
  odd thumb addresses). Entries (key: press/second/flag):
  10: 0x8018ED4/0x8018F1A/1 (the session-61 cross-check ✓);
  11: 0x8018222/0x8018264/1; 12: 0x8017218/0x801735C/2;
  13: 0x8016FD0/0x8017064/0; 14: 0x801709C/0x80171DC/2;
  15: 0x8016F08/0x8016F96/0.
- **func_08014660 (the key-11 action) / func_0801471e (the key-12) = the
  FINE-TUNE pair**, NOT a +5 kHz sweep: a 1-kHz sub-counter at ctx[5]
  (±1 per call, reset to 4/0), the coarse ±5 kHz on the sub-counter wrap,
  the combined value at ctx[0x12] (the smooth 1-kHz resolution), the
  redraw flag DAT_08014f34+2 = 1 per call, the wrap handling at the
  band edges ([2] = the low limit, [3] = the high limit).
- The press handlers 0x8018222/0x801709C = the per-screen multi-key
  dispatchers (10 = the menu state 0x34; 11/12 = the step functions;
  13 = func_08014cd4).
- The files rea_fn_*.c = the decompiled bodies (the key-11/12/14 press +
  second, the two helpers).
- OPEN: the observed p3k11/p3k14 +5 kHz-per-slice series is NOT produced by
  the fine-tune pair; the remaining candidates = the pre-counter helper
  func_08020c9e (a small flag-setter, decompiled - not the renderer) and
  the per-key timestamp/auto-repeat helper func_08021a50 (the session-61
  0x0801b2a0 table) - the next decompile target before touching the recon.
