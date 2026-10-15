#include "STC8A8K64D4.H"
#include "STC_EEPROM.H"
#include "UART1.H"
#include "UART2.H"
#include "ADC.H"
#include "DIGI.H"
#include "tostring.H"

#include  <INTRINS.H> // Keil library




#define IAP_EN			0x80

#define 	IAP_TRIG()	IAP_TRIG = 0x5A,	IAP_TRIG = 0xA5		// IAP trigger command

// 8A8K64D4 dedicated
// sfr IAP_TPS  =   0xF5;   // 8A8K64D4
// #define MAIN_Fosc 22118400L //Define the main clock (accurately calculate the 115200 baud rate)
#define     IAP_ENABLE()    IAP_CONTR = IAP_EN; IAP_TPS = FOSC / 1000000
#define     IAP_DISABLE()   IAP_CONTR = 0; IAP_CMD = 0; IAP_TRIG = 0; IAP_ADDRH = 0xff; IAP_ADDRL = 0xff





/************* External function and variable declaration*****************/
unsigned char code	VER[] = {"UV98_868_220430_D4"};	// Firmware version date information


unsigned char  EEPROM_Buffer[512];



void   READ_VER(uchar UARTx)  	// Read the version number
{
    READ_CPU_ID();	 // Read CPU serial number
    READ_ADC();		 // Read voltage value
 
    if (UARTx == 1)
    {
        UART1_SendString(" Ver: ");
        UART1_SendString(VER);
        UART1_SendString(" | ");	  // Report Version
        UART1_SendString("CPU ID: ");
        UART1_SendString(CPU_ID);
        UART1_SendString(" | ");	 // Report CPU serial number
        UART1_SendString("Voltage: ");
        UART1_SendString(DY);
        UART1_SendString(" V| ");	 // 
 
    }
    else
    {
        UART2_SendString(" Ver: ");
        UART2_SendString(VER);
        UART2_SendString(" | ");	  // Report Version
        UART2_SendString("CPU ID: ");
        UART2_SendString(CPU_ID);
        UART2_SendString(" | ");	 // Report CPU serial number
        UART2_SendString("Voltage: ");
        UART2_SendString(DY);
        UART2_SendString(" V| ");	 // 
 
    }
}
 

/************************ Read N bytes function, up to 255 bytes at a time********************/
/************************ Write N bytes function, up to 255 bytes at a time********************/
/******************** Sector erase function 512 bytes/sector********************/


/*********************************************************************/

void DisableEEPROM(void)		// The following statements can be omitted, just for safety reasons
{
    IAP_CONTR = 0;				// Prohibit IAP/IAP operation
    IAP_CMD   = 0;				// Remove IAP/IAP command
    IAP_TRIG  = 0;				// Prevent IAP/IAP command from being triggered by mistake
    IAP_ADDRH = 0xff;			// Point to non-EEPROM area to prevent misoperation
    IAP_ADDRL = 0xff;			// Point to non-EEPROM area to prevent misoperation
}


/************************ Read N bytes function, up to 255 bytes at a time********************/
// void EEPROM_read_n(uint EE_address,uchar *DataAddress,uchar length)
// {
// EA = 0; //Disable interrupts
// IAP_ENABLE(); //Macro call, set waiting time, allow IAP/IAP operation, send once is enough
// IAP_CMD =IAP_READ(); //Macro call, send byte read command, if the command does not need to be changed, no need to resend the command
// do
// {
// IAP_ADDRH = EE_address / 256; //Send the high byte of the address (the address needs to be resent only when it needs to be changed)
// IAP_ADDRL = EE_address % 256; //Send address low byte
// IAP_TRIG(); //Macro call, first send 5AH, then send A5H to IAP/IAP trigger register, this is required every time
// _nop_();
// *DataAddress = IAP_DATA; //The read data is sent to
// EE_address++;
// DataAddress++;
// }while(--lenth);
// 
// DisableEEPROM();
// EA = 1; //Re-enable interrupts
// }


/************************ Read 1 byte function********************/
unsigned char EEPROM_read_one(uint EE_address)
{
    unsigned char j;
// EA = 0; //Disable interrupts
    IAP_ENABLE();					// Macro call, set waiting time, allow IAP/IAP operation, send once is enough
    IAP_CMD = IAP_READ;						// Macro call, send byte read command, if the command does not need to be changed, no need to resend the command

    IAP_ADDRH = EE_address / 256;		// Send the high byte of the address (the address needs to be resent only when it needs to be changed)
    IAP_ADDRL = EE_address % 256;		// Send address low byte
    IAP_TRIG();							// Macro call, first send 5AH, then send A5H to IAP/IAP trigger register, this is required every time
    _nop_();
    j = IAP_DATA;			// The read data is sent to

    DisableEEPROM();
// EA = 1; //Re-enable interrupts

    return 	j;
}






/************************ Sector erase function **********************/
void EEPROM_SectorErase(uint EE_address)
{
// EA = 0; //Disable interrupts
    // Only sector erase, no byte erase, 512 bytes/sector.
    // Any byte address in a sector is a sector address.
    IAP_ADDRH = EE_address / 256;			// Send the high byte of the sector address (the address needs to be resent only when it needs to be changed)
    IAP_ADDRL = EE_address % 256;			// Send sector address low byte
    IAP_ENABLE();							// Set the waiting time, allow IAP/IAP operation, and send it once is enough
    IAP_CMD = IAP_ERASE;						// Macro call, send sector erase command, if the command does not need to be changed, no need to send the command again
    IAP_TRIG();								// Macro call, first send 5AH, then send A5H to IAP/IAP trigger register, this is required every time
    DisableEEPROM();
// EA = 1; //Re-enable interrupts
}

/************************ Write N bytes function, up to 255 bytes at a time********************/
void EEPROM_write_n(uint EE_address, uchar *DataAddress, uchar lenth)
{
// EA = 0; //Disable interrupts
    IAP_ENABLE();							// Set the waiting time, allow IAP/IAP operation, and send it once is enough
    IAP_CMD =	IAP_WRITE;						// Macro call, send byte write command, if the command does not need to be changed, no need to resend the command

    do
    {
        IAP_ADDRH = EE_address / 256;		// Send the high byte of the address (the address needs to be resent only when it needs to be changed)
        IAP_ADDRL = EE_address % 256;		// Send address low byte
        IAP_DATA  = *DataAddress;			// Send data to IAP_DATA, and only need to resend when the data changes
        IAP_TRIG();							// Macro call, first send 5AH, then send A5H to IAP/IAP trigger register, this is required every time
        _nop_();
        EE_address++;						// Next address
        DataAddress++;						// Next data
    }
    while(--lenth);						// Until the end

    DisableEEPROM();
// EA = 1; //Re-enable interrupts
}

/************************ Write 1 byte function********************/
void EEPROM_write_one(uint EE_address, uchar ndata)
{
// EA = 0; //Disable interrupts
    IAP_ENABLE();							// Set the waiting time, allow IAP/IAP operation, and send it once is enough
    IAP_CMD =	IAP_WRITE;						// Macro call, send byte write command, if the command does not need to be changed, no need to resend the command

    IAP_ADDRH = EE_address / 256;		// Send the high byte of the address (the address needs to be resent only when it needs to be changed)
    IAP_ADDRL = EE_address % 256;		// Send address low byte
    IAP_DATA  = ndata;			// Send data to IAP_DATA, and only need to resend when the data changes
    IAP_TRIG();							// Macro call, first send 5AH, then send A5H to IAP/IAP trigger register, this is required every time
    _nop_();
    // Until the end

    DisableEEPROM();
// EA = 1; //Re-enable interrupts
}
void EEPROM_write_String(uint EE_address, uchar *s)
{
    IAP_ENABLE();							// Set the waiting time, allow IAP/IAP operation, and send it once is enough
    IAP_CMD =	IAP_WRITE;						// Macro call, send byte write command, if the command does not need to be changed, no need to resend the command

    while (*s)                  // Detect the end of a string
    {
        IAP_ADDRH = EE_address / 256;		// Send the high byte of the address (the address needs to be resent only when it needs to be changed)
        IAP_ADDRL = EE_address % 256;		// Send address low byte
        IAP_DATA  = *s;			// Send data to IAP_DATA, and only need to resend when the data changes
        IAP_TRIG();
        _nop_();			// Macro call, first send 5AH, then send A5H to IAP/IAP trigger register, this is required every time
        EE_address++;			// Next address
        s++;					// Next data
    } 						// Until the end

    // Write 0x00 as the end symbol
    IAP_ADDRH = EE_address / 256;		// Send the high byte of the address (the address needs to be resent only when it needs to be changed)
    IAP_ADDRL = EE_address % 256;		// Send address low byte
    IAP_DATA  = 0x00;			// Send data to IAP_DATA, and only need to resend when the data changes
    IAP_TRIG();
    _nop_();			// Macro call, first send 5AH, then send A5H to IAP/IAP trigger register, this is required every time

    DisableEEPROM();
}

void   EEPROM_UPDATA()  	// Update your settings
{
    uint i;
    EEPROM_SectorErase(0x0000);	   		// Before writing, erase sector 1 (0-512 bytes)

    for(i = 0; i < 512; i++)
    {
        EEPROM_write_one(0x0000 + i, EEPROM_Buffer[i]);
    }
}



void   POWER_READ_SETUP()  	// Read all parameter settings into the buffer at startup
{
    uint i;

    for(i = 0; i < 512; i++)
    {
        EEPROM_Buffer[i] = EEPROM_read_one(0x0000 + i);    // Copy parameters to buffer
    }
}


// 0 = U segment 1 = V segment The default V segment is used for the first initialization
void DEMO_SETUP(uchar UV)	// Check EEROM after power on, if there is any error, restore default parameters
{
    unsigned char i;
    unsigned char code CALL[7] = {"NOCALL"};
    unsigned char code  WD[10] = {"3133.90N"};			// default
    unsigned char code  JD[10] = {"12022.80E"};			// default

    unsigned char code  MSG[30] = {"UV98"};			// default
// unsigned char code  IP_NAME[50]={"AT+CIPSTART=\"TCP\",\"202.141.176.2\",14580\r\n"};			//默认

    unsigned char code  IP_NAME[50] = {"202.141.176.2"};			// default

// unsigned char code  WIFI_NAME[]={"AT+CWJAP=\"HiWiFi_3452AE_2G\",\"13013684000\"\r\n"};			//默认

    unsigned char code  WIFI_NAME[] = {"HiWiFi_3452AE_2G"};			// default
    unsigned char code  WIFI_CODE[] = {"13013684000"};			// default


    unsigned char code  FREQ_U[35] = {"1,431.0400,431.0400,0,3,0 "};			// Default length: 27 bytes
    unsigned char code  FREQ_V[35] = {"1,144.6400,144.6400,0,3,0,0 "};			// Default length: 29 bytes


    unsigned char code  SOS[30] = {"SOS"};			// default

// //******************************************************

    EEPROM_SectorErase(0x0000);	   // Erase sector 1 (0-512 bytes)

    // Erase Write Restart

    // ******************************************************
    EEPROM_write_one(0x0000, 20 / 256);	// RF beacon time is set to 30
    EEPROM_write_one(0x0001, 20 % 256);

    EEPROM_write_one(0x0002, 0);		// Timing beacon ON/OFF
    EEPROM_write_one(0x0003, 0);		// PTT linkage beacon ON/OFF
    EEPROM_write_one(0x0004, 0);		// Smart Beacon 0-3
    EEPROM_write_one(0x0005, 0);		// Queue beacon ON/OFF
    EEPROM_write_one(0x0006, 0);		// Queue beacon timing
    // ******************************************************
    EEPROM_write_one(0x0007, 3);		// PTT delay 50MS step, 0-9, minimum 200MS, default 50x6=300ms
    // *****************************************************
    EEPROM_write_n(0x0008, CALL, 7);	// Clear call sign register
    EEPROM_write_one(0x000F, 7);	  	// SSID=7

    // *****************************************************

    EEPROM_write_one(0x0010, 1 ); // MICE ON/OFF
    EEPROM_write_one(0x0011, 0 ); // MICE Type
    // *****************************************************
    EEPROM_write_one(0x0012, '!' ); // type
    EEPROM_write_one(0x0013, '/' ); // Icon Sets
    EEPROM_write_one(0x0014, '[' ); // Car icon &gt; Relay icon # Gateway icon r Weather icon _ [
    // *****************************************************

    EEPROM_write_one(0x0015, 0 ); 	// Bluetooth output GPS 0=OFF, 1=ON
    EEPROM_write_one(0x0016, 2 ); 	// Bluetooth output 0=OFF 1=KISS hex, 2=UI data 3=waypoint GPS 4=KISS ASC

    // *****************************************************
    EEPROM_write_one(0x0017, 1 ); 	// Coordinate format 0 = degrees 1 = degrees minutes 2 = degrees minutes seconds
    EEPROM_write_one(0x0018, 0 ); 	// Speed unit 0 = kilometer 1 = nautical mile 2 = mile
    EEPROM_write_one(0x0019, 0 ); 	// Distance unit 0 = km 1 = nautical mile 2 = mile
    EEPROM_write_one(0x001a, 0 ); 	// Altitude unit 0 = meter 1 = foot

    EEPROM_write_one(0x001b, 0 ); 	// Temperature unit 0 = Celsius 1 = Fahrenheit
    EEPROM_write_one(0x001c, 0 ); 	// Rainfall unit 0=mm 1=inch
    EEPROM_write_one(0x001D, 0 ); 	// Wind speed unit 0=m/s 1=mph
    // *****************************************************
    EEPROM_write_one(0x001E, 1 ); 	// 
    // *****************************************************
    // *****************************************************


    // *****************************************************
    // 2A-2F 3A-3F A0-AF E0-FF Free
    EEPROM_write_one(0x002a, 0 ); 	// Coordinate selection, 0 = fixed coordinates/fixed station 1 = GPS coordinates/mobile station
    EEPROM_write_one(0x002b, 1 ); 	// GPS switch 0=OFF 1=ON
    EEPROM_write_one(0x002c, 21 ); 	// GPS time zone 0-26, corresponding to plus or minus 0-13-26 21-13=8
    EEPROM_write_one(0x002D, 0 ); 	// GPS power saving function 0=OFF 1=ON
    EEPROM_write_one(0x002E, 1 ); 	// Buzzer receiving prompt 0=OFF 1=ON
    EEPROM_write_one(0x002F, 1 ); 	// Buzzer sends reminder 0=OFF 1=ON

    EEPROM_write_one(0x003A, 1 ); 	// Bluetooth power switch
    EEPROM_write_one(0x003B, 0 ); 	// Report air pressure 0=OFF 1=ON
    EEPROM_write_one(0x003C, 0 ); 	// Report voltage 0=OFF 1=ON
    EEPROM_write_one(0x003D, 0 ); 	// Report temperature 0=OFF 1=ON
    EEPROM_write_one(0x003E, 0 ); 	// Report mileage 0=OFF 1=ON
    EEPROM_write_one(0x003F, 0 ); 	// Report number of satellites 0=OFF 1=ON


    // *****************************************************
    EEPROM_write_String(0x0020, WD);	 // latitude
    EEPROM_write_String(0x0030, JD);	 // longitude
    EEPROM_write_String(0x0040, MSG); // Custom information


    EEPROM_write_one(0x00A0, 0 ); // Internal module switch 0=OFF 1=ON 2=only TX
    EEPROM_write_one(0x00A1, 1 ); // UV module volume 1-9 levels
    EEPROM_write_one(0x00A2, 1 ); // MIC sensitivity 1-8
    EEPROM_write_one(0x00A3, 0 ); // MIC voice encryption 0 1-8

    EEPROM_write_one(0x00A4, 0 ); 	// Automatic shutdown time 0=OFF 1=0.5H
    EEPROM_write_one(0x00A5, 0 ); 	// Bluetooth/ISP port display analysis data
    EEPROM_write_one(0x00A6, 1 ); 	// 
    EEPROM_write_one(0x00A7, 0 ); 	// GPS beacon 0=RF+GPRS 1=GPRS 2=RF 3=AUTO 4=off
    EEPROM_write_one(0x00A8, 1 ); 	// 0 = Disable uploading of received beacons 1 = Enable
    EEPROM_write_one(0x00A9, 1 ); 	// When uploading a beacon, whether to insert your own call sign, 0 = do not insert 1 = insert



    EEPROM_write_one(0x00AA, 1 ); 	// 1 = CAR shows the heading of the other vehicle, 0 = N shows the relative north position
    EEPROM_write_one(0x00AB, 1 ); 	// Format 0=CSV 1=TXT 2=KISS 3=GPWPL

    EEPROM_write_one(0x00AC, 1 ); 	// Backlight brightness 0=30 1=50 2=100 3=AUTO (GPS time control)
    EEPROM_write_one(0x00AD, 1 );
    // Initial LCD interface 0 = Startup interface 1 = Main menu 2 = APRS detailed beacon 3 = ARS list interface
    // 4=GPS interface 5=Setting interface 6=Radar interface 7=Information interface

    EEPROM_write_one(0x00AE, 1 ); 	// Temperature sensor type 0 = DS18B20 1 = AM2320
    EEPROM_write_one(0x00AF, 0 ); 	// WIFI switch //0=WIFI OFF 1= WIFI ON

    // *****************************************************

    if (UV == 0)
    {
        EEPROM_write_String(0x0080, FREQ_U);
    }
    else
    {
        EEPROM_write_String(0x0080, FREQ_V);
    }

    // 0 =U 1=VU segment intercom module frequency // V segment intercom module frequency 27/29 bytes
    // *****************************************************

    EEPROM_write_one(0x00B0, 1 ); 	// 0=GPRS OFF  1= GPRS ON
    EEPROM_write_one(0x00B1, 1 ); 	// Login to send serial number
    EEPROM_write_one(0x00B2, 1 ); 	// APN 99=Taiwan
    EEPROM_write_one(0x00B3, 0 ); 	// GPRS connection mode 0=TCP 1=UDP
    EEPROM_write_one(0x00B4, 14580 / 256 ); 	// GPRS port numberPort number
    EEPROM_write_one(0x00B5, 14580 % 256 ); 	// Port Number


    EEPROM_write_one(0x00B6, 0); 	// 0 = Passive navigation 1 = Dynamic navigation


    EEPROM_write_one(0x00B7, 0); 	// Mileage unsigned long four bytes //0～4294967295
    EEPROM_write_one(0x00B8, 0);
    EEPROM_write_one(0x00B9, 0);
    EEPROM_write_one(0x00BA, 0);
    EEPROM_write_one(0x00BB, 1 ); 	// Accumulated mileage memory 0=OFF 1=ON



    EEPROM_write_one(0x0BC, '\\' ); // Icon Set 2
    EEPROM_write_one(0x0BD, 'P' ); // Icon 2 Cart &gt; Relay Icon# Gateway Iconr Weather Icon_

    EEPROM_write_one(0x00BE, 180 / 256 ); 	// Icon 2 conversion time
    EEPROM_write_one(0x00BF, 180 % 256 ); 	// 


// EEPROM_write_String(0x00B0,IP_NAME); //Server IP information 42 length AT+DSCADDR=0,&quot;TCP&quot;,&quot;202.141.176.2&quot;,14580
    for(i = 0; i < 32; i++)		 // Limit length to 32 bytes
    {
        EEPROM_write_one(0x00C0 + i, IP_NAME[i]); // Domain name or IP address

        if (IP_NAME[i] == 0x00)
        {
            break;
        }
    }

// EEPROM_write_String(0x0100,WIFI_NAME);	 //WIFI名称

    for(i = 0; i < 15; i++)		 // Limit length to 15 bytes
    {
        EEPROM_write_one(0x0100 + i, WIFI_NAME[i]); // WIFI NAME

        if (WIFI_NAME[i] == 0x00)
        {
            break;
        }
    }

    for(i = 0; i < 15; i++)		 // Limit length to 15 bytes
    {
        EEPROM_write_one(0x0110 + i, WIFI_CODE[i]); // WIFI PASSWORD

        if (WIFI_CODE[i] == 0x00)
        {
            break;
        }
    }

    // *****************************************************
    for(i = 0; i < 30; i++)		 // Limit length to 30 bytes
    {
        EEPROM_write_one(0x01B0 + i, SOS[i]);	// Custom information

        if (SOS[i] == 0x00)
        {
            break;
        }
    }

    EEPROM_write_one(0x01D0, 0 ); 	// Enable emergency beacon 0=OFF 1=ON
    EEPROM_write_one(0x01D1, 1 );   // 
    EEPROM_write_one(0x01D2, 1 );   // 
    EEPROM_write_one(0x01D3, 1 ); 	// CMX865AE TX level 0-7 0=-10.5DB 7=0dB
    EEPROM_write_one(0x01D4, 6 ); 	// CMX865AE RX level 0-7
    EEPROM_write_one(0x01D5, 0 ); 	// Relay forwarding waiting time
    EEPROM_write_one(0x01D6, 0 ); 	// Relay forwarding channel, 0=A, 1=B, 2=A+B
    EEPROM_write_one(0x01D7, 0 ); 	// Transmit channel, 0=A, 1=B, 2=A+B


    EEPROM_write_n(0x0120, "START1", 7);	// G5500 target call sign
    EEPROM_write_one(0x0127, 1);	  		// G5500 target call sign SSID
    EEPROM_write_one(0x0128, 0);	  		// G5500 Enable 0=OFF 1=ON
    EEPROM_write_one(0x0129, 100 / 256);	// Fixed station height
    EEPROM_write_one(0x012a, 100 % 256);

    EEPROM_write_one(0x012b, 1); // List direction display mode 0=English 1=0-12 2-0-36

    EEPROM_write_one(0x012c, 0); // 0 = Turn off LCD 1 = Turn on LCD
    EEPROM_write_one(0x012D, 0); // LCD display interface
    EEPROM_write_one(0x012E, 0); // Last Position Report
    EEPROM_write_one(0x012F, 0); // Allows 6 nautical miles of low-speed transmission, high-speed transmission is not allowed



    dig_Initial();

    EEPROM_write_one(0x0130, 0 ); 	// Alarm if no movement for 30 minutes
    EEPROM_write_one(0x0131, 0 ); 	// No movement for 60 minutesEMERGENCY

// EEPROM_write_one(0x0132, 1 ); // Enable WeChat
// EEPROM_write_one(0x0133, 1 ); // Enable APRS

    EEPROM_write_one(0x01D1, 0); // LCD brightness 0=Auto 1 //1=High Brightness //2=Normal

// LCD Brightness
// 0=Auto1
// 1=Automatic 2
// 1 = Highlight
// 2 = Normal
// 3 = Off
    EEPROM_write_one(0x01D2, 0); // Beacon popup 0 = disable 1 = enable
    EEPROM_write_one(0x01DF, 0); // LCD off time 0 = no off 1 = 1 minute


    EEPROM_write_one(0x001F, 0x03);	// Initialization mark


}


// System reset //0=U segment 1=V segment The default V segment is initialized for the first time
unsigned char Initial_check_erom()	  // Check if the MAC address is stored. If not, the TNC is not initialized.
{
    if ( EEPROM_read_one(0X001F) != 0x03)
    {
        DEMO_SETUP(1);
        IAP_CONTR = 0x20;
        return 0;
    }

    return 1;
}


