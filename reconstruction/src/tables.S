/* Recovered .data tables and the UI string region — [RECOVERED]
 * (sessions 39-42: armlink compressed-RW-data streams decompressed).
 *
 * .tables   = the RAM table image (RAM 0x20000000..0x20001730): key-action
 *             tables, menu string pointer arrays, CSS tone table, screen
 *             attributes — the original's initialized globals.
 * .uistrings= the flash region the table pointers target (linked
 *             0x08040000..0x08046C97): GB2312 + English UI strings, asset
 *             tables. Placed at the original's linked address so every
 *             pointer in .tables resolves unmodified.
 *
 * The 35 code-pointer words in .tables target the original's functions
 * (key-action handlers); the reconstruction's own dispatch uses its own
 * code — those words are reference data here.
 */
.section .tables, "a"
.global ui_ram_tables
ui_ram_tables:
    .incbin "data/ram_tables_recovered.bin"

.section .uistrings, "a"
.global ui_string_region
ui_string_region:
    .incbin "data/uistrings_region.bin"
