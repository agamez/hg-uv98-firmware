#include "STC8A8K64D4.H"
#include "STC_EEPROM.H"
//#include "DELAY.H"

#include "UART1.H"
#include "UART2.H"
#include "UART3.H"
//#include "UART4.H"

#include "GPS2.H"
#include "IO.H"


//#include "BEACON.H"
//#include "bmp280.h"
#include "PUBLIC_BUF.H"


#include<string.h>


#define BAUD    9600			   //GPS速率9600

#define S3RI  0x01              //S3CON.0
#define S3TI  0x02              //S3CON.1
//#define S3RB8 0x04              //S3CON.2
//#define S3TB8 0x08              //S3CON.3
#define S3_S0 0x02              //P_SW2.1

unsigned char   UART3_GPRMC_DATA[128] ;		  //串口缓冲尺寸
unsigned char   UART3_GPGGA_DATA[128] ;		  //串口缓冲尺寸

unsigned int 	UART3_BUF_LENTH;	//串口收发数据长度

unsigned char UART3_RX_BUSY;	    //串口3接收数据等待处理
bit UART3_TX_BUSY;		//串口3发送空闲



uchar UART3_OUTTIME; //接收数据计数，用于判断GPS是否断开


unsigned char UART3_CHECK_FLAG(uchar *p, uchar *p1)			//对比字符串
{
//GPS  "$GPRMC"  "$GBRMC"
//BD   "$GPGGA"  "$GBGGA"

    if(*p != *p1)
    {
        return 0;
    }

    p++;
    p1++;	  	//对比 $

    if(*p != *p1)
    {
        return 0;
    }

    p++;
    p1++;		//对比 G

    p++;
    p1++;				   	//忽略P 或 B

    if(*p != *p1)
    {
        return 0;
    }

    p++;
    p1++;		//对比 RMC或GGA

    if(*p != *p1)
    {
        return 0;
    }

    p++;
    p1++;		//对比 $

    if(*p != *p1)
    {
        return 0;
    }

    p++;
    p1++;		//对比 $

    return 1;
}
//
///*----------------------------
//发送串口数据
//----------------------------*/
//void UART3_SendData(uchar dat)
//{
//    while (UART3_TX_BUSY);      //等待前面的数据发送完成
//    UART3_TX_BUSY = 1;
//    S3BUF = dat;                //写数据到UART2数据寄存器
//}
//
///*----------------------------
//发送字符串
//----------------------------*/
//void UART3_SendString(char *s)
//{
//    while (*s)                  //检测字符串结束标志
//    {
//        UART3_SendData(*s++);   //发送当前字符
//    }
//}
//



//串口3中断接收数据
void UART3() interrupt 17 using 1
{
    //发送完毕  //清除S3TI位 //清忙标志
    if (S3CON & S3TI)
    {
        S3CON &= ~S3TI;
        UART3_TX_BUSY = 0;
        return;
    }

    if (S3CON & S3RI)
    {
        S3CON &= ~S3RI;         //清除S3RI位

        if (UART3_RX_BUSY == 2)
        {
            return ;   //如果串口3数据的未处理，则不接收新的数据
        }

        //接收GPS 数据
        if (UART3_BUF_LENTH > 120)
        {
            UART3_BUF_LENTH = 0;	   //接收数据超长，重新接收
            return;
        }

        //接收数据超长，重新接收
        if ( UART3_RX_BUSY == 0  	)
        {
            UART3_GPRMC_DATA[UART3_BUF_LENTH] = S3BUF;
            UART3_BUF_LENTH++;		 //接收1个字节数据

            if (S3BUF == 0x0A)	 //收到1行数据		//接收GPS数据,数据帧中检索指定的数据行
            {
                UART3_GPRMC_DATA[UART3_BUF_LENTH] = 0x00;		//	补结束符号


                if (UART3_CHECK_FLAG(UART3_GPRMC_DATA, "$GPRMC") == 1)	 	//检查是否是GPRMC数据
                {
                    UART3_RX_BUSY = 1;
                }	//	串口收到GPRMC,下次接收GPGGA

                UART3_BUF_LENTH = 0;
                return ; 		//清除长度标记，准备下次接收
            }
        }

        //-------------------------------------------------

        if ( UART3_RX_BUSY == 1  	)
        {
            UART3_GPGGA_DATA[UART3_BUF_LENTH] = S3BUF;
            UART3_BUF_LENTH++;		 //接收1个字节数据

            if (S3BUF == 0x0A)	 //收到1行数据		//接收GPS数据,数据帧中检索指定的数据行
            {
                UART3_GPGGA_DATA[UART3_BUF_LENTH] = 0x00;		//	补结束符号

                if (UART3_CHECK_FLAG(UART3_GPGGA_DATA, "$GPGGA") == 1)	//检查是否是GPRMC数据
                {
                    UART3_RX_BUSY = 2;	 //串口又收到GPRMC,等待处理
                }

                UART3_BUF_LENTH = 0;
                return ; 		//清除长度标记，准备下次接收
            }
        }
    }
}

//北斗
//$GBRMC,,V,,,,,,,,,,N,V*3B
//$GBVTG,,,,,,,,,N*22
//$GBGGA,,,,,,0,00,99.99,,,,,,*5A
//$GBGSA,A,1,,,,,,,,,,,,,99.99,99.99,99.99,4*3A
//$GBGSV,1,1,00,0*77
//$GBGLL,,,,,,V,N*76



void UART3_FUN()
{
    if (UART3_RX_BUSY == 2)	 //串口3收到GPRMC GPGGA数据标志
    {
        if (GPS_EN == 1)
        {
            if(EEPROM_Buffer[0X15] == 1)
            {
                UART2_SendString(UART3_GPRMC_DATA);	 //调试输出GPRMC
                UART2_SendString(UART3_GPGGA_DATA);	 //调试输出GPRMC
//			UART1_SendString(UART3_GPRMC_DATA);	 //调试输出GPRMC
//			UART1_SendString(UART3_GPGGA_DATA);	 //调试输出GPRMC
            }

            UART3_RX_GPS();	    //	处理GPS数据
        }

        UART3_OUTTIME = 0;
        UART3_RX_BUSY = 0;	//	//超时清0串口3,重新接收
        GPS_LINK = 1; //GPS已连接
    }

    //4秒后还没收到GPS数据，判定为GPS没链接		   	//关GPS灯 	 //计时清0
    if (UART3_OUTTIME > 5)
    {
        UART3_OUTTIME = 0;
        GPS_LOCKED = 0;
        LED_STU = 1;
        GPS_LINK = 0; //GPS已断开
    }
}



void UART3_Initial()
{
    P_SW2 &= ~S3_S0;            //S3_S0=0 (P0.0/RxD3, P0.1/TxD3)
//  P_SW2 |= S3_S0;             //S3_S0=1 (P5.0/RxD3_2, P5.1/TxD3_2)

    S3CON = 0x50;               //8位可变波特率
//	定时器3用作串口3的波特率发生器
    T3L = (65536 - (FOSC / 4 / BAUD)); //设置波特率重装值
    T3H = (65536 - (FOSC / 4 / BAUD)) >> 8;
    T4T3M |= 0x02;              //定时器3为1T模式
    T4T3M |= 0x08;              //定时器3开始计时

    UART3_TX_BUSY = 0;
    UART3_RX_BUSY = 0;
    UART3_BUF_LENTH = 0;	//数据长度重置=0	  //清除串口2缓冲数据

    IE2 |= 0x08;                 //使能串口3中断
//	EA = 1;
}

