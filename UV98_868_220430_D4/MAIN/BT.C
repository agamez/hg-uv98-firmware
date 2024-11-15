#include "STC8A8K64D4.H"
#include "STC_EEPROM.H"

#include "KISS2GAOMIN.H"
#include "KISS2ASC.H"
// #include "UART1.H"
#include "UART2.H"
// #include "UART4.H"

#include "BEACON.H"
#include "BT.H"
// #include "MICE_DECODE.H"

#include "tostring.H"

#include "KISS_Analysis.H"
// #include "tostring.H"
#include "DELAY.H"

#include "PUBLIC_BUF.H"


#include <string.h>



// Bluetooth output 0=waypoint, 1=KISS, 2=UI data 3=GPS 4=GPS+UI 5=OFF




void Disp_Analytic_data()		 // Bluetooth/ISP port displays additional information
{
// uchar SN_RX_BUFFER[200];
    uint  k ;
    uchar temp[20];

// return;

    if (EEPROM_Buffer[0x00A5] != 1)
    {
        return;
    }


    SN_RX_BUFFER[0] = 0;
    // ==============================================
    strcat(SN_RX_BUFFER, "{alt: ");

    if (UI_ALT[0] == 0)
    {
        strcat(SN_RX_BUFFER, "------");
    }
    else
    {
        strcat(SN_RX_BUFFER, UI_ALT);
    }

    // Write altitude, up to 6 bytes
    strcat(SN_RX_BUFFER, " || speed: ");

    if (UI_SPEED[0] == 0)
    {
        strcat(SN_RX_BUFFER, "---.-");
    }
    else
    {
        strcat(SN_RX_BUFFER, UI_SPEED);
    }

    // Write speed, up to 6 bytes

    // ==============================================
    strcat(SN_RX_BUFFER, " || course: ");

    if (UI_DIR[0] == 0)
    {
        strcat(SN_RX_BUFFER, "---");
    }
    else
    {
        strcat(SN_RX_BUFFER, UI_DIR);
    }

    // Write heading, up to 3 bytes
    // --------------------------------------------------------
    strcat(SN_RX_BUFFER, " || Dir(north): ");
    k = 0;
    tostring(UI_Angle_N);	   // Relative north direction, up to 3 bytes
    temp[k++] = bai;
    temp[k++] = shi;
    temp[k++] = ge;
    temp[k++] = 0x00;

    strcat(SN_RX_BUFFER, temp);
    // --------------------------------------------------------
    strcat(SN_RX_BUFFER, " || Dir(Relative): ");
    k = 0;
    tostring(UI_Angle_CAR);		 // Relative vehicle head position, up to 3 bytes
    temp[k++] = bai;
    temp[k++] = shi;
    temp[k++] = ge;
    temp[k++] = 0x00;

    strcat(SN_RX_BUFFER, temp);

    // --------------------------------------------------------
    strcat(SN_RX_BUFFER, " || distance: ");		  // distance





    switch (UI_JULI[0])
    {
        case 0:
            strcat(SN_RX_BUFFER, "----.-");
            strcat(SN_RX_BUFFER, " Km ");
            break;

        case '*':	  // Less than 10,000 meters
            strcat(SN_RX_BUFFER, UI_JULI + 1);
            strcat(SN_RX_BUFFER, " Km ");
            break;

        default:
            strcat(SN_RX_BUFFER, UI_JULI);
            strcat(SN_RX_BUFFER, " Km ");
            break;
    }




// k=0;
// tostring((uint)UI_JULI);
// temp[k++]=bai;  		temp[k++]=shi;   	temp[k++]=ge;  	temp[k++]=0x00;
// strcat(SN_RX_BUFFER,temp);
// strcat(SN_RX_BUFFER," Km ");

    // --------------------------------------------------------

    strcat(SN_RX_BUFFER, " || elevation: ");	   // Relative elevation angle

    k = 0;
    tostring(Elevation_angle);
    temp[k++] = bai;
    temp[k++] = shi;
    temp[k++] = ge;
    temp[k++] = 0x00;
    strcat(SN_RX_BUFFER, temp);

    strcat(SN_RX_BUFFER, " }\r\n");
    // --------------------------------------------------------
    UART2_SendString(SN_RX_BUFFER);

    // --------------------------------------------------------
    SN_RX_BUFFER[0] = 0;

    strcat(SN_RX_BUFFER, "{GS-232B/G5500 CMD: ");
    strcat(SN_RX_BUFFER, "W");

    k = 0;
    tostring(UI_Angle_N);	   // Relative north direction, up to 3 bytes
    temp[k++] = bai;
    temp[k++] = shi;
    temp[k++] = ge;
    temp[k++] = 0x00;

    strcat(SN_RX_BUFFER, temp);
    // --------------------------------------------------------
    strcat(SN_RX_BUFFER, " ");
    // --------------------------------------------------------
    k = 0;						 // Elevation
    tostring(Elevation_angle);
    temp[k++] = bai;
    temp[k++] = shi;
    temp[k++] = ge;
    temp[k++] = 0x00;
    strcat(SN_RX_BUFFER, temp);

    strcat(SN_RX_BUFFER, " }\r\n");

    UART2_SendString(SN_RX_BUFFER);
}


void G5500_OUT( )	// Bluetooth output data 0 = output the beacon sent by itself 1 = output the decoded beacon
{
    uchar *p;
    uint  k ;
    uchar temp[20];

// float GS232B[20];
// if (EEPROM_Buffer[0x0128]==0) {return;}
// -----------------------------------------------------
    if(EEPROM_Buffer[0X15] != 2)
    {
        return;    // Whether to enable G5500 control
    }

    READ_TEMP_CALL(0x0120, 0x0127);	 // Read the call sign-SSID of the tracking target

    p = strstr(UI_CALL, TEMP_Call);	// Compare with received call sign

    if  (p != NULL) 						// Comparison success
    {
        SN_RX_BUFFER[0] = 0;

        strcat(SN_RX_BUFFER, "W");

        k = 0;
        tostring(UI_Angle_N);	   // Relative north direction, up to 3 bytes
        temp[k++] = bai;
        temp[k++] = shi;
        temp[k++] = ge;
        temp[k++] = 0x00;

        strcat(SN_RX_BUFFER, temp);
        // --------------------------------------------------------
        strcat(SN_RX_BUFFER, " ");
        // --------------------------------------------------------
        k = 0;						 // Elevation
        tostring(Elevation_angle);
        temp[k++] = bai;
        temp[k++] = shi;
        temp[k++] = ge;
        temp[k++] = 0x00;
        strcat(SN_RX_BUFFER, temp);

        strcat(SN_RX_BUFFER, "\r\n");

        UART2_SendString(SN_RX_BUFFER);
    }

}




void SETUP_BL_NAME()
{
    Delay_time_25ms(20);

    READ_TEMP_CALL(0X0008, 0X000F);
    UART2_SendString("AT+NAME");	 // Initialize Bluetooth name settings
    UART2_SendString(TEMP_Call);	 // Initialize the Bluetooth name
// UART2_SendString(&quot;\r\n&quot;); //No need to add carriage return
    Delay_time_25ms(20); // It needs to be delayed to wait for the Bluetooth setting to be completed, and it will be effective next time the power is turned on
}




void BT_OUT(unsigned char STU)	// STU=0 unknown format, 1=valid beacon received 2=self-sent beacon
{
    uint i;

    if (EEPROM_Buffer[0x0016] == 0)
    {
        return;
    }

    if (EEPROM_Buffer[0x0016] == 3) 	// Waypoint Format
    {
        if (STU == 0)
        {
            return;   // Unknown format data
        }

        if (STU == 2)
        {
            // Output the beacon you send yourself in GPWPL format

            MY_BEACON_TO_GPWPL();	// Convert your own beacon directly into GPWPL waypoint format

            UART2_SendString(GPWPL_BUF );  // Serial port 2 Bluetooth output waypoint data
// UART1_SendString(GPWPL_BUF); //Serial port 2 Bluetooth output waypoint data
            return;						   // Beacons sent by yourself do not need to display analysis data
        }

        if (STU == 1)
        {
            UI_TO_GPWPL();			       // Received beacons, parsed into waypoints
            UART2_SendString(GPWPL_BUF );  // Serial port 2 Bluetooth output waypoint data
// UART1_SendString(GPWPL_BUF); //Serial port 2 Bluetooth output waypoint data

            Disp_Analytic_data();		   // Display and analyze the received data

            return;
        }
    }

    if (EEPROM_Buffer[0x0016] == 1) 		 	// KISS HEX
    {
        UART2_SendData(0xC0);
        UART2_SendData(0x00); 		// Bluetooth serial port outputs KISS data

        for (i = 0; i < KISS_LEN; i++)
        {
            UART2_SendData(KISS_DATA[i]);
        }

        UART2_SendData(0xC0);

// UART1_SendData(0xC0); UART1_SendData(0x00); //Bluetooth serial port outputs KISS data
// for (i=0;i<KISS_LEN;i++)    {  UART1_SendData(KISS_DATA[i]);}
// UART1_SendData(0xC0);

        return;
    }

    if (EEPROM_Buffer[0x0016] == 2) 		 	 // 2=UI
    {
        KISS_TO_ASCII(SN_RX_BUFFER, 0);	 	 // The KISS data received by the radio is converted to ASCII UI format after RF decoding, and the UI data length UI_DIGI_LEN is obtained.
        UART2_SendString(SN_RX_BUFFER);	     // Serial port 1 monitoring
// UART1_SendString(SN_RX_BUFFER); //Serial port 1 monitoring

        if (STU == 0)
        {
            return;   // Unknown format data
        }

        if (STU == 2)
        {
            return;   // Beacons sent by yourself do not need to display analysis data
        }

        if (STU == 1)
        {
            Disp_Analytic_data();    // Display and analyze the received data
        }

        return;
    }

    if (EEPROM_Buffer[0x0016] == 4) 	 		// 4=KISS ASC
    {
        DEBUG_KISS(KISS_DATA, KISS_LEN);

        if (STU == 0)
        {
            return;   // Unknown format data
        }

        if (STU == 2)
        {
            return;   // Beacons sent by yourself do not need to display analysis data
        }

        if (STU == 1)
        {
            Disp_Analytic_data();   // Display and analyze the received data
        }

        return;
    }
}

