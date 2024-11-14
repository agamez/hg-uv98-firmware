
#include "STC8A8K64D4.H"
#include "STC_EEPROM.H"
#include "DELAY.H"
#include "CHx.H"

#include "UART4.H"
#include "UART2.H"
//#include "BEACON.H"
#include "CMX865A_CODE.H"
#include "io.H"
#include "BT.H"

#include "PUBLIC_BUF.H"



//ptt 方式修改下
//不用控制PTT线
//通过指令控制发射

//$E01\r\n  CH A  PTT接通
//$E02\r\n  CH B  PTT接通
//$E00\r\n  PTT断开

//#define   PTT_CH_A_ON  UART2_SendString("$E01\r\n");	UART4_SendString("$E01\r\n"); CMX865A_HDLC_TX(KISS_DATA,KISS_LEN);
//#define   PTT_CH_B_ON   UART2_SendString("$E02\r\n");   UART4_SendString("$E02\r\n"); CMX865A_HDLC_TX(KISS_DATA,KISS_LEN);

#define   PTT_CH_A_ON  	PTT=1; UART4_SendString("$E01\r\n"); CMX865A_HDLC_TX(KISS_DATA,KISS_LEN); 	UART4_SendString("$E00\r\n"); PTT=0;
#define   PTT_CH_B_ON   PTT=1; UART4_SendString("$E02\r\n"); CMX865A_HDLC_TX(KISS_DATA,KISS_LEN);	UART4_SendString("$E00\r\n");PTT=0;

//#define   PTT_OFF  		UART2_SendString("$E00\r\n");	UART4_SendString("$E00\r\n");


void BEACON_TX_CHX(uchar mode)	 //发射自己的信标 MODE=0   中继转发MODE=1
{
    uchar CHx;

    Delay_time_25ms(1);//延时50ms,给大板处理时间


    if (mode == 0)
    {
        CHx = EEPROM_Buffer[0X1D7];
    }
    else
    {
        CHx = EEPROM_Buffer[0X1D6];
    }

    //CH A发送RF信标
    if (CHx == 0)
    {
        PTT_CH_A_ON	  	BT_OUT(2);	   //蓝牙输出
        return;
    }

    //CH B发送RF信标
    if (CHx == 1)
    {
        PTT_CH_B_ON	 	BT_OUT(2);	   //蓝牙输出
        return;
    }

    //CH A+B发送RF信标
    if (CHx == 2)
    {
        PTT_CH_A_ON
        Delay_time_25ms(40);
        PTT_CH_B_ON
        BT_OUT(2);
        return; //蓝牙输出
    }


    //仅蓝牙发送数据，CH A+B均不发送RF信标
    if (CHx == 3)
    {
        BT_OUT(2);
        UART4_SendString("$F01\r\n");
        return; //仅蓝牙输出
    }



}

