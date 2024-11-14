#include "STC8A8K64D4.H"
#include "STC_EEPROM.H"
#include "IO.H"

#include "UART1.H"
#include "UART2.H"
//#include "UART3.H"
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




unsigned char  UARTx_BUF[600] ;		  //串口缓冲尺寸
uint UARTx_BUF_LENTH;	//串口收发数据长度


uchar UARTx_TXD_KISS()	    	//处理串口接收到KISS数据
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
        return 0;   //检查是否是KISS数据  //判断是否KISS数据C0 00 ...C0 格式
    }

    KISS_LEN = 0;				 //复制KISS

    for(i = 2; i < (UARTx_BUF_LENTH - 1); i++)
    {
        KISS_DATA[i - 2] =	UARTx_BUF[i];
        KISS_LEN++;
    }

//	CMX865A_HDLC_TX(KISS_DATA,KISS_LEN);   //RF发送数据

    BEACON_TX_CHX(0);

//	KISS_TO_ASCII(SN_RX_BUFFER,0);	//KISS数据转换ASCII UI格式,并取得UI数据长度	UI_DIGI_LEN
//	UART1_SendString(SN_RX_BUFFER ); //串口1监控
    return 1;
}
//

//uchar UARTx_TXD_KISS(uchar *p,uint len)	    	//处理串口接收到KISS数据
//{	 unsigned char i;
//
//	if (*p !=0xC0){return 0;}
//   	if (*(p+1) !=0x00){return 0;}
//  	if (*(p+len-1) !=0xC0){return 0;}
//
//	if (len<20){return 0;}	 	//检查是否是KISS数据  //判断是否KISS数据C0 00 ...C0 格式
//
//	KISS_LEN=0;				  //复制KISS
//	for(i=2;i<(len-1);i++)   	 {    KISS_DATA[i-2] =	*(p+i);	KISS_LEN++;	  }
//
//	CMX865A_HDLC_TX(KISS_DATA,KISS_LEN);   //RF发送数据
//
////	KISS_TO_ASCII(SN_RX_BUFFER,0);	//KISS数据转换ASCII UI格式,并取得UI数据长度	UI_DIGI_LEN
////	UART1_SendString(SN_RX_BUFFER ); //串口1监控
//	return 1;
//}




unsigned char CHECK_AT_CMD(  unsigned char *cmd)	   //对比AT指令
{
    uchar *p;
    p = strstr(UARTx_BUF, cmd);

    if  (p != NULL)
    {
        return 1; 		   //对比完毕
    }

    return 0;
}


void UART4_TX_EEROM()
{
    uint i;
// 	for(i=0;i<512;i++)		{EEPROM_Buffer[i]=EEPROM_read_one(0x0000+i); }	 //复制参数到缓冲区

    READ_CPU_ID();	 //读取CPU序列号
    UART4_SendString("$A00,");
    UART4_SendString(CPU_ID);
    UART4_SendString(",");

    for(i = 0; i < 512; i++)
    {
        UART4_SendData(EEPROM_Buffer[i]);     //复制参数到缓冲区
    }

    UART4_SendString("\r\n");

//$A00,14字节序列号，512字节设置，0x0d,0x0a
//for(i=0;i<128;i++){UART4_SendData(EEPROM_Buffer[i]);  }	 //复制参数到缓冲区
}
void UART4_RX_EEROM()
{
    uint i;

    if (UARTx_BUF_LENTH == 519)			//更新设置
    {
        for(i = 0; i < 512; i++)
        {
            EEPROM_Buffer[i] = UARTx_BUF[5 + i];
        }

        EEPROM_UPDATA();  	//更新设置
//		UART4_SendString("$A01,OK\r\n");

        CMX_RX_ON();

        if (EEPROM_Buffer[0x002B] == 1)
        {
            GPS_EN = 1;   //GPS打开或关闭
        }
        else
        {
            GPS_EN = 0;
        }

        if (EEPROM_Buffer[0X3A] == 0)
        {
            BL_PWR = 0;   //初始化蓝牙
        }
        else
        {
            BL_PWR = 1;
        }

//			DEBUG_KISS(EEPROM_Buffer,48) ;

//		for(i=0;i<512;i++)		{UART2_SendData(EEPROM_Buffer[i]);  }	 //蓝牙临时输出参数

        UART2_SendString("HELLO");

        for(i = 0; i < 512; i++)
        {
            UART2_SendData(EEPROM_Buffer[i]);	 	   //复制参数到缓冲区
        }



    }
}


uchar FUN_UART_2_4()
{
    //==============================================
    //==============================================
    //$A00,14字节序列号，512字节设置，0x0d,0x0a
    if (CHECK_AT_CMD("$A00\r\n") == 1)
    {
        UART4_TX_EEROM();
        return 1;
    }

    //==============================================
    //接收设置
    if (CHECK_AT_CMD("$A01,") == 1)
    {
        UART4_RX_EEROM();
        return 1;
    }

    //==============================================
    //侧键发射GPS或固定台站信标
    if (CHECK_AT_CMD("$A09\r\n") == 1)
    {
        BEACON_GPS_TXD();
        return 1;
    }

    //==============================================
    //报警键按下,发射GPS或固定台站信标
    if (CHECK_AT_CMD("$A10\r\n") == 1)
    {
        BEACON_GPS_TXD();
        return 1;
    }

    //PTT键按下发射GPS或固定台站信标
    if (CHECK_AT_CMD("$A11\r\n") == 1)
    {
        if (EEPROM_Buffer[0X03] == 1)
        {
            BEACON_GPS_TXD();
        }

        return 1;
        //检测PTT按下  等待松开后，发射一次信标
//	UART4_SendString("$A11,OK\r\n");
    }






    //==============================================
    //读取信标列表
    if (CHECK_AT_CMD("$B") == 1)
    {
        if (UARTx_BUF_LENTH == 6)
        {
            Delay_time_25ms(4);
            FUN_B((UARTx_BUF[2] - 0X30) * 10 + (UARTx_BUF[3] - 0X30));
        }

        return 1;
    }

    //==============================================
    //读取详细信标
    if (CHECK_AT_CMD("$C") == 1)
    {
        if (UARTx_BUF_LENTH == 6)
        {
            Delay_time_25ms(4);
            FUN_C((UARTx_BUF[2] - 0X30) * 10 + (UARTx_BUF[3] - 0X30));
        }

        return 1;
    }

    //==============================================
    //清除全部记录0-511,清除索引
    if (CHECK_AT_CMD("$D00\r\n") == 1)
    {
        AT2C512_CLEAN();
        UART4_SendString("$D00,OK\r\n");
        return 1;  //计时清0，防止GPS状态连续输出
    }

    //==============================================
    //恢复出厂设置并重新读取全部参数设置到缓冲
    if (CHECK_AT_CMD("$F00\r\n") == 1)
    {
        DEMO_SETUP(1);
        POWER_READ_SETUP();
        UART4_SendString("$F00,OK\r\n");
        return 1;
    }

    //==============================================
    //低音测试
    if (CHECK_AT_CMD("AT+TONE=1200\r\n") == 1)
    {
        TONE1200();
        return 1;
    }

    //==============================================
    //低音测试
    if (CHECK_AT_CMD("AT+TONE=2200\r\n") == 1)
    {
        TONE2200();
        return 1;
    }

    //==============================================
    //高低音测试关闭
    if (CHECK_AT_CMD("AT+TONE=OFF\r\n") == 1)
    {
        TONE_OFF();
        return 1;
    }

    //==============================================



//	//里程清0
////	if (CHECK_AT_CMD("AT+LC=CLEAN\r\n")==1) 	{  	LC_CLEAN();	  	return 1; 	}
//	if (CHECK_AT_CMD("$A30\r\n")==1) 	{  	LC_CLEAN();	  	return 1; 	}
    //==============================================

    //快速里程清0
    if (CHECK_AT_CMD("$A30\r\n") == 1)
    {
        LC_CLEAN();
        return 1;
    }

    //快速设置固定站
    if (CHECK_AT_CMD("$A31\r\n") == 1)
    {
        QUICK_SET_EN = 1;
        return 1;
    }


    //==============================================

    //读取版本号
    if (CHECK_AT_CMD("$A50\r\n") == 1)
    {
        UART4_SendString("$A50,");
        UART4_SendString(VER);
        UART4_SendString("\r\n");
        return 1;
    }

    //==============================================
    //读取CPU序列号
    if (CHECK_AT_CMD("$A51\r\n") == 1)
    {
        READ_CPU_ID();
        UART4_SendString("$A51,");
        UART4_SendString(CPU_ID);
        UART4_SendString("\r\n");
        return 1;
    }

    //==============================================
    return 0;
}



uchar FUN_UART_1_2( uchar UARTx)		   //对比AT指令
{
    uint i;

// 	for(i=0;i<256;i++)		{EEPROM_Buffer[i]=EEPROM_read_one(0x0000+i); }	 //复制参数到缓冲区
    //   41 54 2B 49 53 50 3D 4F 4E 0D 0A	 //固件不掉电升级调试
    if (UARTx == 1)
    {
        if (CHECK_AT_CMD( "AT+ISP=ON") == 1)
        {
            IAP_CONTR = 0x60;    //升级调试
            return 1;
        }
    }

    if (CHECK_AT_CMD( "AT+RST=1") == 1)
    {
        IAP_CONTR = 0x20;
        return 1;
    }


//	if (CHECK_AT_CMD("$A20\r\n")==1)   	{ 	 A20_OUT_EN=0;		return 1;}
//
//	if (CHECK_AT_CMD("$A21\r\n")==1)   	{ 	 A20_OUT_EN=1;		return 1;}



    if (UARTx == 2)
    {
        if (UARTx_TXD_KISS() == 1)
        {
            return 1;   //处理串口接收到KISS数据
        }

//	if (UARTx_TXD_KISS(UARTx_BUF,UARTx_BUF_LENTH)==1){return 1;} 	//处理串口接收到KISS数据

    }

    if (CHECK_AT_CMD( "AT+SET=READ") == 1)	 //读设置
    {
//		Delay_time_25ms(40);

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
        }  //复制参数到缓冲区


//		DEBUG_KISS(EEPROM_Buffer,48) ;


//	   	Delay_time_25ms(20);
        return 1;
    }

    if (CHECK_AT_CMD( "AT+SET=WRITE") == 1)	 //写设置
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
                GPS_EN = 1;   //GPS打开或关闭
            }
            else
            {
                GPS_EN = 0;
            }

            if (EEPROM_Buffer[0X3A] == 0)
            {
                BL_PWR = 0;   //初始化蓝牙
            }
            else
            {
                BL_PWR = 1;
            }

//			CMX865A_Init();	//初始化CMX86X
//		   	UART4_TX_EEROM() ;

//			Delay_time_25ms(10);

        }
    }

    if (CHECK_AT_CMD( "AT+VER=?") == 1)	 //返回版本号
    {
//	Delay_time_25ms(20);
        READ_VER(UARTx);
//		Delay_time_25ms(20);
        return 2;
    }

    if (CHECK_AT_CMD( "AT+TX=ON") == 1)
    {
        BEACON_GPS_TXD(); 	   //指令发射测试
        return 2;
    }

    //设置U段参数默认值	//0 =U 1=V
    if (CHECK_AT_CMD( "AT+DEMO=ON") == 1)
    {
        DEMO_SETUP(1);
        IAP_CONTR = 0x20;
        return 1;
    }

    if (CHECK_AT_CMD( "AT+TONE=1200") == 1)
    {
        TONE1200();	   //低音测试
        return 1;
    }

    if (CHECK_AT_CMD( "AT+TONE=2200") == 1)
    {
        TONE2200();	   //高音测试
        return 1;
    }

    if (CHECK_AT_CMD( "AT+TONE=OFF") == 1)
    {
        TONE_OFF();	   //高低音测试关闭
        return 1;
    }

    //里程清0
    if (CHECK_AT_CMD("AT+LC=CLEAN\r\n") == 1)
    {
        LC_CLEAN();
        return 1;
    }


    return 0;
}








uchar UART_X_CMD(uchar UARTx)		   //对比AT指令
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


