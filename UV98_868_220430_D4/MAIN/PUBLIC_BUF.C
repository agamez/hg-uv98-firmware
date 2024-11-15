#include "STC8A8K64D4.H"
#include "PUBLIC_BUF.H"


 
 
unsigned char SN_RX_BUFFER[250]=0;	// Decoded KISS data, excluding C0 00 ..C0, length &lt; 128
 
unsigned char KISS_DATA[200];	// Decoded KISS data, excluding C0 00 ..C0, length &lt; 128
unsigned char KISS_LEN;			// Decoded KISS data length

 
uchar ASC_TEMP[300];		// Temporary text data
 
uint   TIME_1S;
 
uchar GPS_LINK;	   // Is GPS connected?

uchar A20_OUT_EN;
