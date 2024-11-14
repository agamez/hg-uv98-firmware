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




void APRS_KISS_DECODE()			 //APRS解码，串口输出KISS，网络输出UI
{
//	uchar stu;
    if (CMX865A_HDLC_RX_2() != 5)
    {
        return;
    }

//	if (CMX865A_HDLC_RX()!=1) 	 { return;  }
//	if (RF_DECODE()!=6) 	 { return;  }	     //RF解码KISS数据
//
//	stu=RF_DECODE();
//
//	if (stu==0	) { return;  }	     //RF解码KISS数据
//
//	UART2_SendString("RX COUNT:  ");	UART2_DEBUG(stu);
//
//	DEBUG_KISS(KISS_DATA,KISS_LEN);
//
//	if (stu!=6	) { return;  }	     //RF解码KISS数据

// 	LED_STU=0;
    DISP_KISS_DATA() ;  //解析并显示对方的定位数据,并显示

    DISP_A08();
    Delay_time_25ms(2);  //刷新实时信标，刷新后适当延时，否则DIGI PTT无法检测到

// 	FUN_B(0);	  	//刷新信标列表
//	LED_STU=1;	 	//

    G5500_OUT( );

    //------------------------------------------------------
    DIGI_FUN();	   //数字中继
}




