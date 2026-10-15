#include "STC8A8K64D4.H"
#include "IO.H"

#include "DIGI.H"
#include "CMX865A_DECODE.H"
#include "KISS_Analysis.H"
#include "FUN_C.h"

#include "UART2.H"

#include "DELAY.H"

#include "FUN_B.h"

#include "APRS_RF.H"

#include "PUBLIC_BUF.H"
#include "tostring.H"
#include "BT.H"

#include <string.h>




void APRS_KISS_DECODE()			 // APRS decoding, serial port output KISS, network output UI
{
// flying stu;
    if (CMX865A_HDLC_RX_2() != 5)
    {
        return;
    }

// if (CMX865A_HDLC_RX()!=1) 	 { return;  }
// if (RF_DECODE()!=6) { return; } //RF decodes KISS data
// 
// stu=RF_DECODE();
// 
// if (stu==0 ) { return; } //RF decodes KISS data
// 
// UART2_SendString("RX COUNT:  ");	UART2_DEBUG(stu);
// 
// DEBUG_KISS(KISS_DATA,KISS_LEN);
// 
// if (stu!=6 ) { return; } //RF decodes KISS data

// LED_STU=0;
    DISP_KISS_DATA() ;  // Analyze and display the other party&#39;s positioning data, and display

    DISP_A08();
    Delay_time_25ms(2);  // Refresh the real-time beacon and delay appropriately after refreshing, otherwise DIGI PTT cannot detect it

// FUN_B(0); //Refresh beacon list
// LED_STU=1; //

    G5500_OUT( );

    // ------------------------------------------------------
    DIGI_FUN();	   // Digital relay
}




