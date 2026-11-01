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

## Session 74: the Tier-D validation sweep opens — the draw pipeline decompiled and validated

Fresh Ghidra headless decompiles (ReaCreateDecomp.java, this session), all
compared line-by-line against the recon C and event-proven on the bench:

- `FUN_0801cb60(y, x, str, ink, cls)` — the master text draw: the GBK
  double-byte branch (the >=0x81 pairs -> the c914 fetch + the bd3c 16x16
  blit, +16 x), the BCD digit bias in-place (<10 -> +0x30), the single-char
  blit via `FUN_0801c95e` (the lookup + the bfb8), the advance via
  `FUN_0801c99e` (the wrap: x > {0x9B,0x9A,0x99,0x98,0x96,0x94,0x94,0x91,
  0x8D,0x90} -> x=6, y += {7,10,12,16,16,16,20,15,24,32}) and
  `FUN_0801cac0` (the steps {6,6,7,8,10,12,12,15,19,16}). The d840 bracket
  = VESTIGIAL (movs r1,r0; movs r0,#0xff; bx lr). Validated:
  recon `ui_draw_text` (the fonts 1/3/6 subset) — CK4 189,036/189,036
  md5-identical thrice; recon `display_draw_text_cls` = the full 10-class
  mirror (the tables confirmed exactly).
- `FUN_0801bfb8(y, x, glyph, ink, inv, cls)` — the glyph blit: the ink
  palette {0:(0,0xEFBE), 1:(0x1F), 2:(0x8D5E), 3:(0xF800), 4:the state
  swap, 5:(0x13A2), 6:(0x58E1)} + the inv variants keyed on
  [0x20004E68+0x18]; the per-cls pixel loops (the cls 1 = the 6x10/12 B,
  the cls 3 = the 8x16/16 B, the cls 6 = the 12x20/36 B, the cls 9 =
  the 16x32/64 B). Validated: recon display_blit_glyph_cell{,_6x10,_12x20}.
- `FUN_08028f8c(c, cls)` — the glyph lookup: the ROM base pointers at
  _DAT_080290c8..ec, the strides {10:*,1:12,2:14,3:16,4:20,5:24,6:36,
  7:45,8:57,9:64}, the cls 0 = the special 0x0802bec8 big-digit path,
  the cls 4/7 = the ' '/'.'/'-' special slots. Matches font.c
  asset_table_get (0x08024F8C = the ROM-address form).
- `FUN_0801bd3c` — the 16x16 1-bpp blit (the window +0xF/+0xF, the 8+8
  row loops, the 32-byte column-major record) = the cb60 GBK glyph
  transport AND the battery-icon class. Validated: recon
  display_blit_icon_16x16.
- `FUN_0801cd32` — the fill (validated: recon display_fill_rect).
- `0x0801bb22` — the display_init register table (validated: recon
  display_init, the byte-exact mirror). `FUN_0801ba30/ba6a/ba8c/b9c8` +
  `FUN_0802377a/90/88/80` — the parallel transport (validated: recon
  bus_write/tft_write_cmd/tft_write_byte/display_set_window).
- The builders: `func_08014cd4` = the screen0 record builder (the
  0x14-byte memcpy + the pop{r1,r2,r3} stack-arg idiom -> the cfd0 slot
  renderer + the 0801460c state advance); `func_0801f05c` = the VFO row
  builder (the 0x1E records); `FUN_0801cfd0` = the slot renderer (the
  slot 0 = the 158x16 fill, the slot 1 = the cf5e text rows, the +17
  line stride); `FUN_0801cf5e` = the fixed-col-3 text row; `0x0801E6F4`
  = the status label-row builder (the 0x50/0x5C/0x37/0x1E/0x44/0x05
  draws + the FUN_0801f57c H/L at 0x32/0x16); `0x0801219c` = THE MAIN
  MENU RENDERER (the 5 rows at x=6, the +0x12a page states 0-4, the
  D334 menu backgrounds 0x22 x 0x9E = 0x29F8 bytes, the ROM item
  tables DAT_08012ef4..); `0x08014530` = the VFO-row rebuild trigger
  (the redraw flag [0x08014f34+2]).
- The Tier-C: `0x08024d76` = the radio-apply chain AND THE +0x119 FLAG
  SETTER (the 0x30/0x7006 writes then *(_DAT_08024e28+0x119)=1) — the
  gate question CONFIRMED; `0x08010D5E` = the dispatcher per-pass step;
  `0x08028D74` = the mailbox ring reset (+0x32/+0x33/+0x1c/+0x20) +
  `0x0802b1e8` = the privileged IRQ-enable wrapper; `0x08027938` = the
  AT1846S POWER-DOWN helper (the 0x30=0x70A6 write) — the old
  "timeout task" annotation corrected.
- The LED: `display_led_write` 0x0802883A has EXACTLY ONE caller =
  0x0801ab2e inside the VFO-apply FUN_0801ab36 (the +0x11E per-slot
  word table) — the warm LED replay's generator identified.
- OPEN (parked with evidence): the pb9/vendor head-serial framing (the
  vendor = 2 x identical 1,250-bit halves; the pb9 save = 5,227 bits +
  853 CRH switches = the I2C ack-poll class), the battery icon's
  callback-driven draw site, the warm LED/ee359 interleaved replays.
