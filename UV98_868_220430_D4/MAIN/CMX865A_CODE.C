#include "STC8A8K64D4.H"
#include "STC_EEPROM.H"

//#include "UART1.H"
#include "DELAY.H"
#include "io.H"
#include "BEACON.H"
#include "BT.H"


#include "CRC16.H"
#include "CMX865A_CODE.H"
#include "CMX865A_SPI.H"

//#include "tostring.H"

#include "UART2.H"
#include "UART4.H"

#include "PUBLIC_BUF.H"


#include  <INTRINS.H> //Keil library 


//************* 发送寄存器
bit HDLC_TX_BIT;
uchar HDLC_TX_COUNT;			//重置连续5个1的计数
uchar HDLC_TX_BUF[300];
uint HDLC_TX_IDX;	  //当前位索引

//************* 接收寄存器
//uchar HDLC_RX_BUF[300];

//uchar HDLC_TX_BIT_OLD;//上次位的状态
//uchar BIT_1_COUNT; //连续1的计数
//uchar END_7E; 	   //7E结束标记
//uchar HDLC_TEMP;   //临时解码数据
//uint  TOTAL_IDX;   //总的位数

//************* 输出KISS数据寄存器

//uchar APRS_KISS_BUF[300];


//每个帧前、后均有一标志码01111110，用作帧的起始、终止指示及帧的同步。
//标志码不允许在帧的内部出现，以免引起歧义。
//为保证标志码的唯一性但又兼顾帧内数据的透明性，可以采用“0比特插入法”来解决。
//该法在发送端监视除标志码以外的所有字段，
//当发现有连续5个“1”出现时，便在其后添插一个“0”，然后继续发后继的比特流。
//在接收端，同样监除起始标志码以外的所有字段。
//当连续发现5个“1”出现后，若其后一个比特“0”则自动删除它，以恢复原来的比特流；
//若发现连续6个“1”，则可能是插入的“0”发生差错变成的“1”，也可能是收到了帧的终止标志码。
//后两种情况，可以进一步通过帧中的帧检验序列来加以区分。
//“0比特插入法”原理简单，很适合于硬件实现。


//uchar HDLC_SendByte (uchar inbyte,uchar aprs_flag)  //aprs_flag 1= 前后标志7E，0=数据7E
//{
//	uchar k, bt; uchar HDLC_TEMP;
//
//	for (k=0;k<8;k++)
//	{                                                         //do the following for each of the 8 bits in the byte
//		bt = inbyte & 0x01;    //取一位
//		if (bt == 0)
//		{
//			HDLC_TX_BIT=!HDLC_TX_BIT;  	//输出取反
//			HDLC_TX_COUNT=0;			//重置连续5个1的计数
//		}
//		else
//		{			HDLC_TX_COUNT++; 	 	}
//
//		HDLC_TEMP<<=1; 		HDLC_TEMP|=HDLC_TX_BIT;
//
//		if ((HDLC_TX_COUNT == 5)&&(aprs_flag == 0) )		  //连续5个“1”，并且不是aprs_flag前后标识码，则插入1个“0”
//		{
//			HDLC_TX_BIT=!HDLC_TX_BIT;  	//连续5个“1”，输出取反,即插入1个0
//			HDLC_TEMP<<=1; 	   	HDLC_TEMP|=HDLC_TX_BIT;
//			HDLC_TX_COUNT=0;	 	   //重置连续5个1的计数
//		}
//
//	  	inbyte = inbyte>>1;   //取下一位
//	}
//
//	CMX865A_TX_DATA(HDLC_TEMP);	   FX604_TX=!FX604_TX;
//	return 	HDLC_TEMP;
//}

//  C0 00
//  82 A0 6A 62 90 62 60
//  9C 9E 86 82 98 98 73
//  03 F0
//  21 	  //21F0  20
//  33 31 33 34 2E 38 30 4E
//  2F
//  31 32 30 32 30 2E 39 39 45
//  3E 	 //3E45   1E
//  36 6F
//  C0

void HDLC_WIRITE_TX_BIT(uchar dat)	   // 写入一位
{
    uint byte_idx;
    uchar bit_idx;

    HDLC_TX_IDX++;
    byte_idx = HDLC_TX_IDX / 8;
    bit_idx = HDLC_TX_IDX % 8;

    if (dat == 1)
    {
        HDLC_TX_BUF[byte_idx] |= (0x80 >> bit_idx);
    }
}

void HDLC_SendByte2 (uchar inbyte, uchar aprs_flag) //aprs_flag 1= 前后标志7E，0=数据7E或其他数据
{
    uchar k, bt;

    for (k = 0; k < 8; k++)
    {
        //do the following for each of the 8 bits in the byte
        bt = inbyte & 0x01;    //取一位

        if (bt == 0)
        {
            HDLC_TX_BIT = !HDLC_TX_BIT;  	//输出取反
            HDLC_TX_COUNT = 0;			//重置连续5个1的计数
        }
        else
        {
            HDLC_TX_COUNT++;
        }


        HDLC_WIRITE_TX_BIT(HDLC_TX_BIT);

        if ((HDLC_TX_COUNT == 5) && (aprs_flag == 0) )		 //连续5个“1”，并且不是aprs_flag前后标识码，则插入1个“0”
        {
            HDLC_TX_COUNT = 0;	 	 //重置连续5个1的计数
            HDLC_TX_BIT = !HDLC_TX_BIT;  //连续5个“1”，输出取反,即插入1个0
            HDLC_WIRITE_TX_BIT(HDLC_TX_BIT);
        }

        inbyte = inbyte >> 1; //取下一位
    }
}

//void DISP_HDLC(uchar dat)	//转成高低符号显示,调试用
//{	  uchar i;
//	for (i=0;i<8;i++)
//	{
//	if ((dat&0x80)==0x80){UART1_SendData('-');}else{UART1_SendData('_');UART1_SendData(' ');}
//	dat<<=1;   //取下一位
//	}
////		UART1_SendData('|');
//}

//
//void CMX865A_HDLC_TX(uchar *pData,uchar nlen)	   //HDLC编码
//{	uint i;	  uint HDLC_LEN;	  //缓存长度
//
////	KISS_TO_ASCII(SN_RX_BUFFER,0);	 	 //电台接收到的 KISS数据,RF解码后,转换ASCII UI格式,并取得UI数据长度	UI_DIGI_LEN
////  UART4_SendString(SN_RX_BUFFER );
////	UART1_SendString(SN_RX_BUFFER );	 //串口1监控
//
//	BT_OUT();	 //蓝牙输出
//
//	Delay_time_25ms(2);//延时50ms,给大板处理时间
//
//	for (i=0;i<250;i++)   {HDLC_TX_BUF[i]=0;   }	  //
//	HDLC_TX_IDX=0;  HDLC_TX_BIT=0;
//
//	GetCrc16_LEN(pData,nlen);	//计算数据的校验值	   //	SendData(FCS_LO)	;  	   //58	 //SendData( FCS_HI)	; 	   //D5
//
//
//	PTT=1;	   LED_STU=0; 	Delay_time_1ms(10);
//
//	CMX865A_TX_ON();  	//启动PTT
//
//
//	//编码
//	for (i=0;i<(8*EEPROM_Buffer[0X07]);i++) {HDLC_SendByte2(0x7E,1);  }//发送50个前置FLAG标志数据 约等于PTT延时8x6.6=53ms
////	for (i=0;i<(8*8);i++) {HDLC_SendByte2(0x7E,1);  }//发送50个前置FLAG标志数据 约等于PTT延时8x6.6=53ms
//
//
//	for (i=0;i<nlen;i++) {HDLC_SendByte2(*pData,0); pData++;}	//编码
//	HDLC_SendByte2(FCS_LO,0);	HDLC_SendByte2(FCS_HI,0);	   	//发送校验值数据，先低后高
//	for (i=0;i<2;i++)   {HDLC_SendByte2(0x7E,1);   }	  //发送后置FLAG标志数据
//
//	HDLC_LEN=  HDLC_TX_IDX/8+1; //必须+2以上
//	for (i=0;i<HDLC_LEN;i++)   {CMX865A_TX_DATA(HDLC_TX_BUF[i]);  }	//发送
//
////   	UART1_DEBUG(HDLC_LEN);
//
//	CMX865A_TX_OFF();
//	PTT=0;	  LED_STU=1;   //关闭PTT
//
//
////	for (i=0;i<HDLC_LEN;i++) 	{	DISP_HDLC(HDLC_TX_BUF[i]);  	}
////
////   	UART1_SendString("TATAL:  ");	UART1_DEBUG(HDLC_LEN);
//
//
//
////UART1_SendData(0xC0);	 UART1_SendData(0x00);
////for (i=0;i<HDLC_LEN;i++)   {UART1_SendData(HDLC_TX_BUF[i]);   }// 调试
////UART1_SendData(FCS_LO);	  	UART1_SendData(FCS_HI);
////UART1_SendData(0xC0);
//
//
//
////	UART1_SendData(0xC0);	UART1_SendData(0x00); 		//蓝牙串口输出KISS数据
////	for (i=0;i<KISS_LEN;i++)    {  UART1_SendData(KISS_DATA[i]);}
////  UART1_SendData(FCS_LO);  	UART1_SendData(FCS_HI);
////	UART1_SendData(0xC0);
//
////	KISS_TO_ASCII(SN_RX_BUFFER,0);	//KISS数据转换ASCII UI格式,并取得UI数据长度	UI_DIGI_LEN
////  KISS_TO_ASCII(SN_RX_BUFFER,1);	 	 //电台接收到的 KISS数据,RF解码后,转换ASCII UI格式,并取得UI数据长度	UI_DIGI_LEN
//// 	UART1_SendString(SN_RX_BUFFER );	 //串口1监控
//
//}





void CMX865A_HDLC_TX(uchar *pData, uchar nlen)	  //HDLC编码
{
    uint i;
    uint HDLC_LEN;	  //缓存长度

//	Delay_time_25ms(1);//延时50ms,给大板处理时间
//	CMX865A_Init();


    for (i = 0; i < 250; i++)
    {
        HDLC_TX_BUF[i] = 0;      //
    }

    HDLC_TX_IDX = 0;
    HDLC_TX_BIT = 1;

    GetCrc16_LEN(pData, nlen);	//计算数据的校验值	   //	SendData(FCS_LO)	;  	   //58	 //SendData( FCS_HI)	; 	   //D5


//	PTT=1;
//	LED_STU=0;	 //Delay_time_1ms(10);
//		CMX_RX_OFF(); //关闭接收
    CMX865A_TX_ON();  	//启动PTT

    //编码
    for (i = 0; i < (8 * EEPROM_Buffer[0X07] + 30); i++)
    {
        HDLC_SendByte2(0x7E, 1);     //发送50个前置FLAG标志数据 约等于PTT延时8x6.6=53ms
    }

    for (i = 0; i < nlen; i++)
    {
        HDLC_SendByte2(*pData, 0);
        pData++;
    }

    HDLC_SendByte2(FCS_LO, 0);
    HDLC_SendByte2(FCS_HI, 0);	   	//发送校验值数据，先低后高

    for (i = 0; i < 2; i++)
    {
        HDLC_SendByte2(0x7E, 1);      //发送后置FLAG标志数据
    }

    //发送
    HDLC_LEN =  HDLC_TX_IDX / 8 + 1; //必须+2以上

    for (i = 0; i < HDLC_LEN; i++)
    {
        CMX865A_TX_DATA(HDLC_TX_BUF[i]);     //发送数据
    }

//   	UART1_DEBUG(HDLC_LEN);

    CMX865A_TX_OFF();
//	CMX_RX_ON();

//	PTT=0;	//  LED_STU=1;   //关闭PTT




//	UART2_SendString("$E00\r\n");	UART4_SendString("$E00\r\n"); 	    	Delay_time_25ms(1);
//	UART2_SendString("$E00\r\n");	UART4_SendString("$E00\r\n"); 	    	Delay_time_25ms(1);

}

