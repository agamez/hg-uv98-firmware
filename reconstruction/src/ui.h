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

/* Key cluster (session 56, Round N — the ADC-ladder keyscan state).
 * The TIM2 sampler pair and the three ladder groups share this bank. */
#define KEY_ADC_CH14     0x20005140u  /* last ch14 sample (group A)       */
#define KEY_ADC_CH15     0x20005142u  /* last ch15 sample (group B)       */
#define KEY_ADC_CH12     0x20005144u  /* last ch12 sample (group C)       */
#define KEY_VALID_A      0x20005160u  /* group A match flag (this pass)   */
#define KEY_VALID_B      0x20005161u  /* group B match flag               */
#define KEY_VALID_C      0x20005162u  /* group C match flag               */
#define KEY_CODE_BYTE    0x2000515Eu  /* the UI task's decoded keycode    */
#define KEY_CODE_COPY    0x2000515Fu  /* main-mode copy (0x8019ff2)       */
#define KEY_PC8_CACHE    0x20005163u  /* TIM2 sampler: last PC8 sample    */
#define KEY_PC7XOR8      0x20005164u  /* TIM2 sampler: PC7 ^ PC8 state    */
#define KEY_PC8_CHANGED  0x20005165u  /* TIM2 sampler: PC8 change flag    */
#define KEY_LADDER_A     0x20001684u  /* group A threshold table (8)      */
#define SETTINGS_CTX     0x20004E68u  /* radio context = the EE settings
                                       * image, 1:1 copy of the 0x0A00 page
                                       * (77 B) at boot [session 61]     */
#define SM_STEP_BASE     0x200046CAu  /* APP_STATE+0x2E: the u16 S-meter
                                       * step durations, per VFO [s61]   */
#define KEY_LADDER_B     0x200016A4u  /* group B threshold table (8)      */
#define KEY_LADDER_C     0x200016C4u  /* group C threshold table (3)      */
#define KEYLOCK_BYTE     0x20009458u  /* bit0 = keypad lock (0x80282fe)   */


/* Master UI task and screen machine (image labels; flash = label+0x4000). */
void ui_task(void *p_arg);                 /* 0x08029178, priority 3       */
void ui_state_set(uint8_t screen);         /* 0x08010530: screen switch    */
uint8_t key_get_code(void);                /* 0x080282FC: the Round N slice*/
void ui_list_select(uint8_t a, uint8_t b, uint32_t entry); /* 0x08010D06 */

/* Input policy. */
void key_event(uint8_t key_id, uint8_t src, uint8_t pressed); /* 0x0801E044 */
int  key_state_test(uint8_t state_byte);                      /* 0x0801B28E */

/* Battery + S-meter chains. */
int  battery_level_read(void);             /* 0x080241A6 (th 0xCB5/0x6D7)  */
int  battery_bars(uint8_t level);          /* 0x080242FC                   */
uint16_t smeter_rssi_db(uint8_t chip);     /* rf_read(chip,0x1B)>>8        */

#endif /* UI_H */
