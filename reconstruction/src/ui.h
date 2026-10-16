/* Master UI task, input policy, and UI state machine — recovered
 * mechanisms. See ui.c for the full provenance and state table. */
#ifndef UI_H
#define UI_H

#include <stdint.h>

/* Master UI state byte values at RAM 0x200050E8 [RECOVERED constants]:
 * printable characters used as enum values by the original authors.
 * 0x00 = idle/default; the full observed compare set (session 22). */
#define UI_STATE_IDLE    0x00
#define UI_STATE_G       0x67
#define UI_STATE_H       0x68
#define UI_STATE_I       0x69
#define UI_STATE_M       0x4D
#define UI_STATE_N       0x4E
#define UI_STATE_X       0x58

/* Master UI task (image labels; physical flash = label + 0x4000). */
void ui_task(void *p_arg);              /* 0x08029178, priority 3 */

/* Input policy (0x0801E044 family). */
void key_event(uint8_t key_id, uint8_t src, uint8_t pressed); /* 0x0801E044 */
int  key_state_test(uint8_t state_byte);                      /* 0x0801B28E */

/* Battery gauge chain. */
int  battery_level_read(void);          /* 0x080241A6: thresholds 0xCB5/0x6D7 */
int  battery_bars(uint8_t level);       /* 0x080242FC: level -> bar count    */

#endif /* UI_H */
