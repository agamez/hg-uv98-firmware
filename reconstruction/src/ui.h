/* Master UI task, screen-state machine, input policy, and gauges —
 * [RECOVERED] from the original image (sessions 18-30, REA-validated).
 * See ui.c header comments for provenance; ui.h for the public API. */
#ifndef UI_H
#define UI_H

#include <stdint.h>

/* ---- Screen-state ID space (sessions 28-30) --------------------------
 * The "state byte" at RAM 0x200050E8 = the CURRENT SCREEN ID. Values are
 * printable characters by design (original authors' debugging idiom):
 *   0x00          idle/off
 *   0x01..0x2F    the menu screens, entered via the menu dispatcher
 *                 (one ui_state_set site per menu item, in order)
 *   0x30..0x8D    the remaining screens/modes (~100 more)
 *   'g' 0x67      RX/RF-active (S-meter ring writes, rf_write)
 *   'h' 0x68 / 'i' 0x69 / 'j' 0x6A   beacon-data windows
 *   'M' 0x4D / 'N' 0x4E                 memory-mode screens
 *   'x' 0x78      refined menu state (from 'X')
 * Compared values (active per-pass logic): 18 of the ~145; the rest are
 * leaf screens served by the generic path. */

#define UI_STATE_IDLE     0x00
#define UI_STATE_RX       0x67   /* 'g' */
#define UI_STATE_BEACON_H 0x68   /* 'h' */
#define UI_STATE_BEACON_I 0x69   /* 'i' */
#define UI_STATE_BEACON_J 0x6A   /* 'j' */
#define UI_STATE_MEM_M    0x4D   /* 'M' */
#define UI_STATE_MEM_N    0x4E   /* 'N' */
#define UI_STATE_MENU_X   0x58   /* 'X' */
#define UI_STATE_MENU_X2  0x78   /* 'x' */
#define UI_STATE_MENU_SL  0x2F   /* '/' */

/* RAM anchors (session 20-29; addresses as in the research log). */
#define UI_STATE_ADDR    0x200050E8u  /* current screen id                */
#define UI_STATE_FLAG    0x200050EAu  /* companion flag (+2)              */
#define FLAG_BANK        0x20005094u  /* protocol/UI flags (+9 bit0=ready)*/
#define APP_STATE        0x2000469Cu  /* heartbeat +0x166, suspend +0x119 */
#define SMETER_RINGS     0x20004F94u  /* per-band 0x1E history rings      */
#define SMETER_SNAPSHOT  0x20004FD0u  /* active-ring snapshot             */
#define BAND_BYTE        0x20004E68u  /* +0x40 = programmable/band byte   */

/* Master UI task and screen machine (image labels; flash = label+0x4000). */
void ui_task(void *p_arg);                 /* 0x08029178, priority 3       */
void ui_state_set(uint8_t screen);         /* 0x08010530: screen switch    */
void ui_list_select(uint8_t a, uint8_t b, uint32_t entry); /* 0x08010D06 */

/* Input policy. */
void key_event(uint8_t key_id, uint8_t src, uint8_t pressed); /* 0x0801E044 */
int  key_state_test(uint8_t state_byte);                      /* 0x0801B28E */

/* Battery + S-meter chains. */
int  battery_level_read(void);             /* 0x080241A6 (th 0xCB5/0x6D7)  */
int  battery_bars(uint8_t level);          /* 0x080242FC                   */
uint16_t smeter_rssi_db(uint8_t chip);     /* rf_read(chip,0x1B)>>8        */

#endif /* UI_H */
