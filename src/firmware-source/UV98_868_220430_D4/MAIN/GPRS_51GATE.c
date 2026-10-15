/*------------------------------------------------------*/
/* ---  		       	     -------------------------------*/
/* --- MCU = STC8A8K64D4	22.1184M				 -------------*/
/* --- WeChat: (86)13013684000  ------------------------*/
/* ---   				----------------------------------------*/
/* --- Tel: 13013684000  15052205720--------------------*/
/* --- Web: BH4TDV.TAOBAO.com --------------------------*/
/*------------------------------------------------------*/

// The compiler automatically adds 0x00 to the end of a text string with unspecified length.
// 
// Note: For the n series chips, all PWM related IO ports are
// High impedance state, these ports need to be set to quasi-bidirectional ports or strong push-pull mode for normal use
// 

#include  <INTRINS.H> // Keil library
#include "STC8A8K64D4.H"
#include "STC_EEPROM.H"
#include "PUBLIC_BUF.H"

#include "UART1.H"
#include "UART2.H"
#include "UART3.H"
#include "UART4.H"

#include "DELAY.H"
#include "ADC.H"

#include "SENSOR\DS18B20.h"
#include "SENSOR\bmp280.h"
#include "IO.H"
#include "GPS2.H"
#include "DISP_GPS.H"
#include "CMX865A_DECODE.H"



#include "KISS_DECODE.H"


#include "KISS2ASC.H"

#include "BEACON.H"

#include "CMX865A_SPI.H"

#include "AT24C512.h"


#include "BT.H"

/************* Timing beacon related ******************/
void Timer0_init();			   					// Timer 0 initialization
unsigned int  time_a;	// 50MS time unit count
/************* General**************/
void  Initial_51G2();



uchar BT_LINK;
uint BT_LINK_TIME;


sbit TXD_1	= P3 ^ 1;	 // 
sbit RXD_1  = P3 ^ 0;	 // 


sbit RXD3	= P0 ^ 0;	 // 
sbit TXD3	= P0 ^ 1;


// #define SLEEP PCON= 0x02; //Power-down mode, current 0.1uA, after the interrupt is triggered, it starts from 0000H
// #define SHUT PCON= 0x01; //Suspend mode, current 2MA, after the interrupt is triggered, continue to the next instruction

// CMX865A_TX_TONE(0x01); CMX865A_TX_TONE(0);	 	BEACON_GPS_TXD();
// 
// void INT0_int (void) interrupt 0 //CPU cannot power down and sleep in interrupt
// {

// }
// 
// 
// IE.0 0 External interrupt 0
// IE.1 1 Timer 0 overflow
// IE.2 2 External interrupt 1
// IE.3 3 Timer 1 overflow
// IE.4 4 Serial port interrupt
// IE.5 5 Timer 2 overflow
// 

void INT0_int (void) interrupt 0	   // The CPU cannot be powered off and hibernated during an interrupt
{
    BT_LINK_TIME = 0;
}

void time0_init()
{

    TIME_1S = 0;

    time_a = 0;				// 40*25mS=1S
    /************* Initialize timer**************/
    TR0 = 0;					// 50ms interruption once
    TF0 = 0;

    TH0 = (65536 - 9216) / 256;	// Timing 5MS=5000US; 65536-22.1184M oscillation MHZ/12*5000US =65536-9216
    TL0 = (65536 - 9216) % 256;


    TR0 = 1;				// Start T0


    ET0 = 1;			// Enable T0 interrupt

}

void  Initial_uv98()
{
    uchar i;
    PTT = 0;

// Note: After the chip is powered on, all PWM-related IO ports are in high impedance state.
// These ports need to be set to quasi-bidirectional ports or strong push-pull mode for normal use.
// Related IO:
// P0.6/P0.7
// P1.6/P1.7/
// P2.1/P2.2/P2.3/P2.7
// P3.7
// P4.2/P4.4/P4.5
// In addition, the P1.0/P1.4 ports are strong push-pull outputs when powered on, and the CPU batch BUG
// When the program is initialized, these two ports must also be set to weak pull-up quasi-bidirectional port mode

// P3.0 P3.1 Power on = VCC/2
    P0M1 = 0x00;
    P0M0 = 0X00;	 // All P0 are set to weak pull-up quasi-bidirectional port mode
    P1M1 = 0x00;
    P1M0 = 0X00;	 // P1 is all set to weak pull-up quasi-bidirectional port mode
    P2M1 = 0x00;
    P2M0 = 0X00;	 // P2 is all set to weak pull-up quasi-bidirectional port mode
    P3M1 = 0x00;
    P3M0 = 0X00;	 // P3 is all set to weak pull-up quasi-bidirectional port mode
    P4M1 = 0x00;
    P4M0 = 0X00;	 // P4 is all set to weak pull-up quasi-bidirectional port mode
    P5M1 = 0x00;
    P5M0 = 0X00;	 // P5 is all set to weak pull-up quasi-bidirectional port mode

// Setting interrupt priority
// After the CPU is reset, each interrupt has the lowest priority 0
// IPH|=0X02; IP|=0X02; //Timer 0 is set to the highest value 3
// IPH|=0X10; //Set serial port 0 to priority 2

    P2M0 = 0X84;	 // P2.3 PTT push-pull output P2.7 GPS push-pull output
    P3M0 = 0X42;	 // P3.1 3.6 push-pull output txd

 

    for(i = 0; i < 2; i++)
    {
        LED_STU = 0;
// LED_GPRS=LED_GPS=LED_TX=LED_RX=LED_TX=0; //Light up all indicators, check LED, all light = white
        Delay_time_25ms(1);
        LED_STU = 1;
// LED_GPRS=LED_GPS=LED_TX=LED_RX=LED_TX=1;	 //关LED
// Delay_time_25ms(2);
    }

    Initial_check_erom();  // Check if the MAC address is stored. If not, the TNC is not initialized.

    POWER_READ_SETUP();  	// Read all parameter settings into the buffer at startup
    adc_Initial();		   // Initialize ADC

    UART1_Initial();	// Initialize serial port 1
    UART2_Initial();	// Initialize serial port 2
    UART3_Initial();	// Initialize serial port 3
    UART4_Initial();	// Initialize serial port 4

    /************* Other settings**************/
// ---------------------------------
// DS18B20_READTEMP() ; //Read the temperature several times after powering on to avoid displaying 85 degrees
// DS18B20_READTEMP() ;

    // ---------------------------------
    GPS_INIT();

    CMX865A_Init();
    CMX_RX_Initial();

    // ---------------------------------
    time0_init();


    EA  = 1;			// Enable global interrupts
}



// ========================================================================
// Remark:
// ========================================================================
void timer0 (void) interrupt 1
{
    CMX_RX_INT();		// Interrupt once every 5ms, and everything is normal if it is less than 6.5ms

    UART1_RX_TIME++;	// UARTx Receive Data Timing
    UART2_RX_TIME++;	// UARTx Receive Data Timing
    UART4_RX_TIME++;	// UARTx Receive Data Timing


    TIME_1S++;

    time_a++;

    if (time_a > (200 - 1))	 // 1 second
    {
        time_a = 0;

        BT_LINK_TIME++;

        UART3_OUTTIME++;   // Receive data count, used to determine whether GPS is disconnected
        GPS_WAIT_TIME++;   // Smart Beacon Time Base
        GPS_BEACON_TIME++; // GPS Timing Beacon Timing
    }
}

void main()
{
    Initial_uv98();	// Initializing the TNC

    if (EEPROM_Buffer[0X3A] == 0)
    {
        BL_PWR = 0;   // Initialize Bluetooth
    }
    else
    {
        BL_PWR = 1;
    }

    if (EEPROM_Buffer[0X3A] == 1)
    {
        SETUP_BL_NAME();   // Initialize Bluetooth name
    }

    READ_BMP280();
    UART4_TX_EEROM() ; 	// Delay_time_25ms(10);

    while (1)
    {
        UART1_FUN();    // Process the data received by serial port 1, set to 9600
        UART2_FUN();	// Processing Bluetooth data of serial port 2, 9600
        UART3_FUN();	// Processing of built-in GPS data of serial port 3, 9600
        UART4_FUN(); 	// Process GPRS G3524 data of serial port 4, 115200
        APRS_KISS_DECODE(); // RF Decoding KISS Data

        BECON_MODE();	// Timing Beacon

        if (TIME_1S > 200 - 1)
        {
            TIME_1S = 0;
            DISP_GPS_STU();
        }


    }
}

// $PCAS11,0*1D //Portable mode
// $PCAS11,1*1C //Static mode
// $PCAS11,2*1F //Walking mode
// $PCAS11,3*1E //Car mode
// $PCAS11,4*19 //Navigation mode
// $PCAS11,5*18 //Aviation mode &lt;1g
// $PCAS11,6*1B //Aviation mode &lt;2g
// $PCAS11,7*1A //Aviation mode &lt;4g
// $PCAS10,0*1C //Hot start





