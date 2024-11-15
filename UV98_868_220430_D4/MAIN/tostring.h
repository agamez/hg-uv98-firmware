

#ifndef _tostring_H_
#define _tostring_H_
// Public function and public variable declarations


extern uchar wan, qian, bai, shi, ge;
extern void tostring(uint val); // Convert to 3 bytes

extern uchar str_txt[10];


extern void disp_Hex2Ascii2(uchar dat);

extern void READ_CPU_ID();	 // Read CPU serial number


extern void DEBUG_KISS(uchar *p, uint len);

extern uchar CPU_ID[16];	// Read CPU serial number

extern uchar TEMP_Call[10];	// Temporary call sign
extern void READ_TEMP_CALL(uint call_eerom_add, uint SSID_eerom_add);	 // Read out temporary call sign




extern void disp_Hex2Ascii3(uint temp)	 ;



#endif