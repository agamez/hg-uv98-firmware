
#ifndef _STC_EEPROM_H_
#define _STC_EEPROM_H_

/************* External function and variable declaration*****************/
 
/************************ Read 1 byte function********************/
extern void EEPROM_SectorErase(uint EE_address) ;

extern unsigned char EEPROM_read_one(uint EE_address);
/************************ Write 1 byte function********************/
extern void EEPROM_write_one(uint EE_address,uchar ndata);

extern void EEPROM_UPDATA();   // Update your settings
extern void DEMO_SETUP(uchar UV);		  // Check EEROM after power on, if there is any error, restore the default parameters 0=U segment 1=V segment

extern void   READ_SETUP();  	// Read settings into buffer

extern unsigned char Initial_check_erom();	  // Check if the MAC address is stored. If not, the TNC is not initialized.
extern unsigned char  EEPROM_Buffer[512];

extern void   POWER_READ_SETUP();  	// Read all parameter settings into the buffer at startup


extern unsigned char code	VER[];	// Firmware version date information


extern void   READ_VER(uchar UARTx);  	// Read the version number


/*****************************************/
#endif