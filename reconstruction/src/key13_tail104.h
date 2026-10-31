/* [session 69 | DATA-DERIVED p3k13_short_orig.stream L191692..191795]
 * the 104-event PB9 tail that follows the 'S' cell in the key-13 press's
 * screen (the same 4-code transport as ui_led_stream[]). Differs from the
 * RSSI-slice tail (rssi_bus_tail.h) in 0 of 104 positions - extracted
 * verbatim from the key-13 capture. Replayed by key13_tail104_replay(). */
#ifndef KEY13_TAIL104_H
#define KEY13_TAIL104_H
#define KEY13_TAIL104_LEN 104u
static const uint8_t key13_tail104[KEY13_TAIL104_LEN] = {
    3,1,0,3,1,0,3,1,0,1,0,0,0,0,0,1,2,3,0,0,0,0,0,0,0,0,
    1,2,3,0,0,0,1,0,0,0,0,1,2,3,1,0,3,1,0,1,0,0,0,0,1,1,
    2,2,3,0,2,3,0,2,3,0,2,3,0,2,3,0,2,3,0,2,3,0,2,3,0,2,
    3,0,2,3,0,2,3,0,2,3,0,2,3,0,2,3,0,2,3,0,2,3,1,3,0,1,
};
#endif
