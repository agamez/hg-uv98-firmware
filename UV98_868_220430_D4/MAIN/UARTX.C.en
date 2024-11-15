#include "STC8A8K64D4.H"
#include "STC_EEPROM.H"
#include "IO.H"

#include "UART1.H"
#include "UART2.H"
// #include "UART3.H"
#include "UART4.H"
#include "UARTx.H"

#include "BEACON.H"
#include "CMX865A_CODE.H"

#include "CMX865A_SPI.H"

#include "DELAY.H"

#include "tostring.H"

#include "FUN_B.h"
#include "FUN_C.h"
#include "AT24C512.h"

#include "GPS2.H"

#include "CHx.H"

#include "PUBLIC_BUF.H"




#include<string.h>




unsigned char  UARTx_BUF[600] ;		  // Serial port buffer size
uint UARTx_BUF_LENTH;	// Length of serial port data sent and received


uchar UARTx_TXD_KISS()	    	// Processing KISS data received by the serial port
{
    unsigned char i;

    if (UARTx_BUF[0] != 0xC0)
    {
        return 0;
    }

    if (UARTx_BUF[1] != 0x00)
    {
        return 0;
    }

    if (UARTx_BUF[UARTx_BUF_LENTH - 1] != 0xC0)
    {
        return 0;
    }

    if (UARTx_BUF_LENTH < 20)
    {
        return 0;   // Check whether it is KISS data //Judge whether it is KISS data C0 00 ... C0 format
    }

    KISS_LEN = 0;				 // Copy KISS

    for(i = 2; i < (UARTx_BUF_LENTH - 1); i++)
    {
        KISS_DATA[i - 2] =	UARTx_BUF[i];
        KISS_LEN++;
    }

// CMX865A_HDLC_TX(KISS_DATA,KISS_LEN); //RF sends data

    BEACON_TX_CHX(0);

// KISS_TO_ASCII(SN_RX_BUFFER,0); //Convert KISS data to ASCII UI format and get UI data length UI_DIGI_LEN
// UART1_SendString(SN_RX_BUFFER); //Serial port 1 monitoring
    return 1;
}
// 

// uchar UARTx_TXD_KISS(uchar *p,uint len) //Process the KISS data received by the serial port
// {	 unsigned char i;
// 
// if (*p !=0xC0){return 0;}
// if (*(p+1) !=0x00){return 0;}
// if (*(p+len-1) !=0xC0){return 0;}
// 
// if (len<20){return 0;}	 	//检查是否是KISS数据  //判断是否KISS数据C0 00 ...C0 格式
// 
// KISS_LEN=0; //Copy KISS
// for(i=2;i<(len-1);i++)   	 {    KISS_DATA[i-2] =	*(p+i);	KISS_LEN++;	  }
// 
// CMX865A_HDLC_TX(KISS_DATA,KISS_LEN); //RF sends data
// 
// KISS_TO_ASCII(SN_RX_BUFFER,0); //Convert KISS data to ASCII UI format and get UI data length UI_DIGI_LEN
// UART1_SendString(SN_RX_BUFFER); //Serial port 1 monitoring
// return 1;
// }




unsigned char CHECK_AT_CMD(  unsigned char *cmd)	   // Compare AT commands
{
    uchar *p;
    p = strstr(UARTx_BUF, cmd);

    if  (p != NULL)
    {
        return 1; 		   // Comparison completed
    }

    return 0;
}


void UART4_TX_EEROM()
{
    uint i;
// for(i=0;i<512;i++)		{EEPROM_Buffer[i]=EEPROM_read_one(0x0000+i); }	 //复制参数到缓冲区

    READ_CPU_ID();	 // Read CPU serial number
    UART4_SendString("$A00,");
    UART4_SendString(CPU_ID);
    UART4_SendString(",");

    for(i = 0; i < 512; i++)
    {
        UART4_SendData(EEPROM_Buffer[i]);     // Copy parameters to buffer
    }

    UART4_SendString("\r\n");

// $A00, 14-byte serial number, 512-byte settings, 0x0d, 0x0a
// for(i=0;i<128;i++){UART4_SendData(EEPROM_Buffer[i]);  }	 //复制参数到缓冲区
}
void UART4_RX_EEROM()
{
    uint i;

    if (UARTx_BUF_LENTH == 519)			// Update your settings
    {
        for(i = 0; i < 512; i++)
        {
            EEPROM_Buffer[i] = UARTx_BUF[5 + i];
        }

        EEPROM_UPDATA();  	// Update your settings
// UART4_SendString(&quot;$A01,OK\r\n&quot;);

        CMX_RX_ON();

        if (EEPROM_Buffer[0x002B] == 1)
        {
            GPS_EN = 1;   // GPS on or off
        }
        else
        {
            GPS_EN = 0;
        }

        if (EEPROM_Buffer[0X3A] == 0)
        {
            BL_PWR = 0;   // Initialize Bluetooth
        }
        else
        {
            BL_PWR = 1;
        }

// DEBUG_KISS(EEPROM_Buffer,48) ;

// for(i=0;i<512;i++)		{UART2_SendData(EEPROM_Buffer[i]);  }	 //蓝牙临时输出参数

        UART2_SendString("HELLO");

        for(i = 0; i < 512; i++)
        {
            UART2_SendData(EEPROM_Buffer[i]);	 	   // Copy parameters to buffer
        }



    }
}


uchar FUN_UART_2_4()
{
    // ==============================================
    // ==============================================
    // $A00, 14-byte serial number, 512-byte settings, 0x0d, 0x0a
    if (CHECK_AT_CMD("$A00\r\n") == 1)
    {
        UART4_TX_EEROM();
        return 1;
    }

    // ==============================================
    // Receiving Settings
    if (CHECK_AT_CMD("$A01,") == 1)
    {
        UART4_RX_EEROM();
        return 1;
    }

    // ==============================================
    // Side button to transmit GPS or fixed station beacon
    if (CHECK_AT_CMD("$A09\r\n") == 1)
    {
        BEACON_GPS_TXD();
        return 1;
    }

    // ==============================================
    // When the alarm button is pressed, GPS or fixed station beacon is transmitted
    if (CHECK_AT_CMD("$A10\r\n") == 1)
    {
        BEACON_GPS_TXD();
        return 1;
    }

    // PTT button is pressed to transmit GPS or fixed station beacon
    if (CHECK_AT_CMD("$A11\r\n") == 1)
    {
        if (EEPROM_Buffer[0X03] == 1)
        {
            BEACON_GPS_TXD();
        }

        return 1;
        // After detecting that PTT is pressed and waiting for release, a beacon is transmitted
// UART4_SendString(&quot;$A11,OK\r\n&quot;);
    }






    // ==============================================
    // Read beacon list
    if (CHECK_AT_CMD("$B") == 1)
    {
        if (UARTx_BUF_LENTH == 6)
        {
            Delay_time_25ms(4);
            FUN_B((UARTx_BUF[2] - 0X30) * 10 + (UARTx_BUF[3] - 0X30));
        }

        return 1;
    }

    // ==============================================
    // Read detailed beacon
    if (CHECK_AT_CMD("$C") == 1)
    {
        if (UARTx_BUF_LENTH == 6)
        {
            Delay_time_25ms(4);
            FUN_C((UARTx_BUF[2] - 0X30) * 10 + (UARTx_BUF[3] - 0X30));
        }

        return 1;
    }

    // ==============================================
    // Clear all records 0-511, clear index
    if (CHECK_AT_CMD("$D00\r\n") == 1)
    {
        AT2C512_CLEAN();
        UART4_SendString("$D00,OK\r\n");
        return 1;  // The timing is cleared to 0 to prevent continuous output of GPS status
    }

    // ==============================================
    // Restore factory settings and re-read all parameter settings into the buffer
    if (CHECK_AT_CMD("$F00\r\n") == 1)
    {
        DEMO_SETUP(1);
        POWER_READ_SETUP();
        UART4_SendString("$F00,OK\r\n");
        return 1;
    }

    // ==============================================
    // Bass Test
    if (CHECK_AT_CMD("AT+TONE=1200\r\n") == 1)
    {
        TONE1200();
        return 1;
    }

    // ==============================================
    // Bass Test
    if (CHECK_AT_CMD("AT+TONE=2200\r\n") == 1)
    {
        TONE2200();
        return 1;
    }

    // ==============================================
    // High and low frequency test off
    if (CHECK_AT_CMD("AT+TONE=OFF\r\n") == 1)
    {
        TONE_OFF();
        return 1;
    }

    // ==============================================



// // Mileage cleared to 0
// if (CHECK_AT_CMD("AT+LC=CLEAN\r\n")==1) 	{  	LC_CLEAN();	  	return 1; 	}
// if (CHECK_AT_CMD("$A30\r\n")==1) 	{  	LC_CLEAN();	  	return 1; 	}
    // ==============================================

    // Quick mileage reset
    if (CHECK_AT_CMD("$A30\r\n") == 1)
    {
        LC_CLEAN();
        return 1;
    }

    // Quickly set up fixed stations
    if (CHECK_AT_CMD("$A31\r\n") == 1)
    {
        QUICK_SET_EN = 1;
        return 1;
    }


    // ==============================================

    // Read the version number
    if (CHECK_AT_CMD("$A50\r\n") == 1)
    {
        UART4_SendString("$A50,");
        UART4_SendString(VER);
        UART4_SendString("\r\n");
        return 1;
    }

    // ==============================================
    // Read CPU serial number
    if (CHECK_AT_CMD("$A51\r\n") == 1)
    {
        READ_CPU_ID();
        UART4_SendString("$A51,");
        UART4_SendString(CPU_ID);
        UART4_SendString("\r\n");
        return 1;
    }

    // ==============================================
    return 0;
}



uchar FUN_UART_1_2( uchar UARTx)		   // Compare AT commands
{
    uint i;

// for(i=0;i<256;i++)		{EEPROM_Buffer[i]=EEPROM_read_one(0x0000+i); }	 //复制参数到缓冲区
    // 41 54 2B 49 53 50 3D 4F 4E 0D 0A //Firmware upgrade and debugging without power failure
    if (UARTx == 1)
    {
        if (CHECK_AT_CMD( "AT+ISP=ON") == 1)
        {
            IAP_CONTR = 0x60;    // Upgrade and debug
            return 1;
        }
    }

    if (CHECK_AT_CMD( "AT+RST=1") == 1)
    {
        IAP_CONTR = 0x20;
        return 1;
    }


// if (CHECK_AT_CMD("$A20\r\n")==1)   	{ 	 A20_OUT_EN=0;		return 1;}
// 
// if (CHECK_AT_CMD("$A21\r\n")==1)   	{ 	 A20_OUT_EN=1;		return 1;}



    if (UARTx == 2)
    {
        if (UARTx_TXD_KISS() == 1)
        {
            return 1;   // Processing KISS data received by the serial port
        }

// if (UARTx_TXD_KISS(UARTx_BUF,UARTx_BUF_LENTH)==1){return 1;} //Process the serial port receiving KISS data

    }

    if (CHECK_AT_CMD( "AT+SET=READ") == 1)	 // Read Settings
    {
// Delay_time_25ms(40);

        if (UARTx == 1)
        {
            UART1_SendString("HELLO");
        }
        else
        {
            UART2_SendString("HELLO");
        }

        for(i = 0; i < 512; i++)
        {
            if (UARTx == 1)
            {
                UART1_SendData(EEPROM_Buffer[i]);
            }
            else
            {
                UART2_SendData(EEPROM_Buffer[i]);
            }
        }  // Copy parameters to buffer


// DEBUG_KISS(EEPROM_Buffer,48) ;


// Delay_time_25ms(20);
        return 1;
    }

    if (CHECK_AT_CMD( "AT+SET=WRITE") == 1)	 // Write Settings
    {
        if (UARTx_BUF_LENTH == (12 + 512))
        {
            for(i = 0; i < 512; i++)
            {
                EEPROM_Buffer[i] = UARTx_BUF[i + 12];
            }

            EEPROM_UPDATA();


            UART4_TX_EEROM();

            CMX_RX_ON();

            if (EEPROM_Buffer[0x002B] == 1)
            {
                GPS_EN = 1;   // GPS on or off
            }
            else
            {
                GPS_EN = 0;
            }

            if (EEPROM_Buffer[0X3A] == 0)
            {
                BL_PWR = 0;   // Initialize Bluetooth
            }
            else
            {
                BL_PWR = 1;
            }

// CMX865A_Init(); //Initialize CMX86X
// UART4_TX_EEROM() ;

// Delay_time_25ms(10);

        }
    }

    if (CHECK_AT_CMD( "AT+VER=?") == 1)	 // Returns the version number
    {
// Delay_time_25ms(20);
        READ_VER(UARTx);
// Delay_time_25ms(20);
        return 2;
    }

    if (CHECK_AT_CMD( "AT+TX=ON") == 1)
    {
        BEACON_GPS_TXD(); 	   // Command transmission test
        return 2;
    }

    // Set the default value of the U segment parameters //0 =U 1=V
    if (CHECK_AT_CMD( "AT+DEMO=ON") == 1)
    {
        DEMO_SETUP(1);
        IAP_CONTR = 0x20;
        return 1;
    }

    if (CHECK_AT_CMD( "AT+TONE=1200") == 1)
    {
        TONE1200();	   // Bass Test
        return 1;
    }

    if (CHECK_AT_CMD( "AT+TONE=2200") == 1)
    {
        TONE2200();	   // Treble test
        return 1;
    }

    if (CHECK_AT_CMD( "AT+TONE=OFF") == 1)
    {
        TONE_OFF();	   // High and low frequency test off
        return 1;
    }

    // Mileage reset to 0
    if (CHECK_AT_CMD("AT+LC=CLEAN\r\n") == 1)
    {
        LC_CLEAN();
        return 1;
    }


    return 0;
}








uchar UART_X_CMD(uchar UARTx)		   // Compare AT commands
{
    if (UARTx == 1)
    {
        FUN_UART_1_2(1);
        return 0;
    }

    if (UARTx == 2)
    {
        FUN_UART_1_2(2);
        return 0;
    }

    if (UARTx == 4)
    {
        FUN_UART_2_4();
        return 0;
    }

    return 0;
}


