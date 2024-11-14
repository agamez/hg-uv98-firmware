#include "STC8A8K64D4.H"
#include "STC_EEPROM.H"


#include "UART1.H"
#include "UART2.H"

#include "DELAY.H"
#include "io.H"

#include "CMX865A_SPI.H"
#include "CMX865A_DECODE.H"
#include "CRC16.H"

#include "tostring.H"

//#include "BEACON.H"
#include "KISS_Analysis.H"
#include "PUBLIC_BUF.H"



#define MAX_LEN 200	 //允许接收的最大KISS数据长度

#include  <INTRINS.H> //Keil library 


bit HDLC_RX_BIT;
uchar HDLC_RX_COUNT;			//重置连续5个1的计数

//************* 接收寄存器
uchar HDLC_RX_BUF[400];
//uchar APRS_KISS_BUF[300];

uchar HDLC_RX_BIT_OLD;//上次位的状态
uchar BIT_1_COUNT; //连续1的计数
uchar END_7E; 	   //7E结束标记
uchar HDLC_RX_TEMP;   //临时解码数据
uint  TOTAL_IDX;   //总的位数

//************* 输出KISS数据寄存器
uint HDLC_RX_IDX;	  //当前位索引
uint HDLC_RX_LEN;	  //缓存长度


uchar CMX_RX_BUSY;


uint RX_OK_COUNT;

sbit sig_in  = P1 ^ 6;


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




uchar HDLC_READ_BIT()	 // 读出一位  0或者 1
{
    uint byte_idx;
    uchar bit_idx;
    HDLC_RX_IDX++;
    byte_idx = HDLC_RX_IDX / 8;
    bit_idx = HDLC_RX_IDX % 8;

    if ((HDLC_RX_BUF[byte_idx] & (0x80 >> bit_idx)) != 0)
    {
        return 1;
    }

    return  0 ;
}

//检索首个FLAG=7E
uchar HDLC_START_7E()
{
    uchar COUNT_7E;
    uchar BIT_STU;//7E计数

    HDLC_RX_TEMP = 0x00;
    HDLC_RX_BIT_OLD = HDLC_READ_BIT();	 //记录起始输入电平

//UART1_SendString("7E start IDX:  ");UART1_DEBUG(HDLC_RX_IDX);
//if (HDLC_RX_BIT_OLD==1){	UART1_SendString("srat 1\r\n"); }else{	UART1_SendString("srat 0\r\n");}

    BIT_1_COUNT = 0;	//连续1的计数
    COUNT_7E = 0;	  //7E计数

    while (1)
    {
        if (HDLC_RX_IDX > (TOTAL_IDX - 2))
        {
            return 0x00;   //检索超长
        }

        BIT_STU	= HDLC_READ_BIT();

        if (BIT_STU != HDLC_RX_BIT_OLD) //发生电平翻转
        {
            HDLC_RX_BIT_OLD = BIT_STU; //记录输入电平
            HDLC_RX_TEMP  >>= 1;			  //右移1位,默认写0

            if  (BIT_1_COUNT == 6)		 // ;当前=0，前面6个1，发现到首个FLAG=7E
            {
                COUNT_7E++;		 	//连续检测到3个FLAG=7E

                if (COUNT_7E > 2)
                {
                    BIT_1_COUNT = 0;	   //检索成功，返回
                    return 0x01;
                }

//		 	UART1_SendString("7E DAT: ");   UART1_SendData(Hex2Ascii2(HDLC_RX_TEMP>>4)); UART1_SendData(Hex2Ascii2(HDLC_RX_TEMP));	  UART1_SendString("\r\n");
//			BIT_1_COUNT=0;	return 0x01;
            }

            BIT_1_COUNT = 0;	 //连续1的计数
        }
        else			 //没翻转，写1
        {
            HDLC_RX_TEMP  >>= 1;	//先右移1位
            HDLC_RX_TEMP |= 0x80; 	  //写1

            BIT_1_COUNT++;		  //连续1的计数+1
//			if (BIT_1_COUNT==7)	{  return 0x05;}  		//发生错误，连续7个"1",可能是长音或非APRS数据	 	// 重新检测
        }
    }

    return 0x00;
}



uchar HDLC_RX_BYTE()
{
    unsigned char bit_count;
    uchar BIT_STU;

    END_7E = 0;	 //数据7E标志清0
    bit_count = 0; //接收8位

    while (bit_count < 8)
    {
        if (HDLC_RX_IDX > (TOTAL_IDX - 2))
        {
            return 0x00;   //检索超长
        }

        BIT_STU	= HDLC_READ_BIT();

        if (BIT_STU != HDLC_RX_BIT_OLD) //发生电平翻转
        {
            HDLC_RX_BIT_OLD = BIT_STU; //记录输入电平

            if (BIT_1_COUNT != 5)	//不是连续5个1，则接收一位，前面连续5个“1”，则忽略此位
            {
                if (BIT_1_COUNT == 6)
                {
                    END_7E = 1;	   //检测到7E标志
                }

                HDLC_RX_TEMP  >>= 1;	//右移1位,默认写0
                bit_count++;
            }

            BIT_1_COUNT = 0;	  //连续1的计数
        }
        else			        //没翻转，写1
        {
            HDLC_RX_TEMP  >>= 1;	//先右移1位
            HDLC_RX_TEMP |= 0x80; 	//写1

            BIT_1_COUNT++;		  //连续1的计数+1

            if (BIT_1_COUNT == 7)
            {
                return 0x05;   //发生错误，连续7个"1",可能是长音或非APRS数据	 	// 重新检测
            }

            bit_count++;
        }
    }

    return 	1;
}

//uchar HDLC_CRC()		 //校验数据最后2位
//{
//	GetCrc16_LEN(KISS_DATA,HDLC_RX_LEN-2);	  //计算数据的校验值
////   	disp_Hex2Ascii2(FCS_LO);   disp_Hex2Ascii2(FCS_HI);
//
//	if (FCS_LO!=KISS_DATA[HDLC_RX_LEN-2])	  { return 0;   }
//	if (FCS_HI!=KISS_DATA[HDLC_RX_LEN-1])	  { return 0;   }
//	KISS_LEN=HDLC_RX_LEN-2;
//	return 1;	//	 对比校验码正确，解码成功
//}
//
//
//uchar HDLC_DECODE( )
//{      //uint i;	//uchar stu;
//
//	if (HDLC_START_7E()!=0x01)	{   return 0; } //寻找数据包前部首个FLAG=7E，没有检索到7E起始标记，则退出
//
//	while (1)		//等待数据包头部的7E全部接收完毕
//	{
// 		if (HDLC_RX_BYTE()!=0x01)	{ return 0;  }		   //如果断线、杂音、长音等异常错误跳出
//		if (HDLC_RX_TEMP!=0x7e)	{break; 	}
//	}
//
//	HDLC_RX_LEN=0;				//接收首个KISS数据
//
// 	while (1)		//数据包中部
//	{
//		KISS_DATA[HDLC_RX_LEN++]=HDLC_RX_TEMP;
//		if (HDLC_RX_BYTE()!=0x01)	{ return 0;   }		   //如果断线、杂音、长音等异常错误跳出
//		if (HDLC_RX_LEN>120)  {  return 0;}		           //接收长度超长，如同时2个电台在发射，数据部分重叠，异常错误跳出
//		if (END_7E==1){break ;}   //结尾标志7E
//	}
//
//	if 	(HDLC_CRC()==1)  { return 1;}	  //校验数据最后2位		   //校验数据错误
//	return 0;
//}
//


uchar HDLC_CRC()		 //校验数据最后2位
{
    GetCrc16_LEN(KISS_DATA, KISS_LEN - 2);	 //计算数据的校验值
//   	disp_Hex2Ascii2(FCS_LO);   disp_Hex2Ascii2(FCS_HI);

    if (FCS_LO != KISS_DATA[KISS_LEN - 2])
    {
        return 0;
    }

    if (FCS_HI != KISS_DATA[KISS_LEN - 1])
    {
        return 0;
    }

    KISS_LEN = KISS_LEN - 2;
    return 1;	//	 对比校验码正确，解码成功
}


uchar HDLC_DECODE( )
{
    if (HDLC_START_7E() != 0x01)
    {
        return 0;    //寻找数据包前部首个FLAG=7E，没有检索到7E起始标记，则退出
    }

    while (1)		//等待数据包头部的7E全部接收完毕
    {
        if (HDLC_RX_BYTE() != 0x01)
        {
            return 1;     //如果断线、杂音、长音等异常错误跳出
        }

        if (HDLC_RX_TEMP != 0x7e)
        {
            break;
        }
    }

    KISS_LEN = 0;		//接收首个KISS数据

    while (1)		//数据包中部
    {
        KISS_DATA[KISS_LEN++] = HDLC_RX_TEMP;

        if (HDLC_RX_BYTE() != 0x01)
        {
            return 2;      //如果断线、杂音、长音等异常错误跳出
        }

        if (KISS_LEN > 190)
        {
            return 3;   //接收长度超长，如同时2个电台在发射，数据部分重叠，异常错误跳出
        }

        if (END_7E == 1)
        {
            break ;   //结尾标志7E
        }
    }

    if 	(HDLC_CRC() == 1)
    {
        return 5;   //校验数据最后2位		   //校验数据错误
    }

//	UART1_SendString("KISS RX:  ");		DEBUG_KISS(KISS_DATA,KISS_LEN);

    return 4;
}

void DISP_HDLC(uchar dat)	//转成高低符号显示,调试用
{
    uchar i;

    for (i = 0; i < 8; i++)
    {
        if ((dat & 0x80) == 0x80)
        {
            UART2_SendData('-');
        }
        else
        {
            UART2_SendData('_');
            UART2_SendData(' ');
        }

        dat <<= 1; //取下一位
    }
}





uchar CMX865A_HDLC_RX()			// 独占解码方式
{
    uchar DCD;
    uint i;
    uint stu;
    uint over_err;
    uint fram_err;

//    DCD=0;
// 	HDLC_RX_LEN=0;
//	while (CMX865A_DET()==1)   //检测到信号，开始记录数据
//	{
//		DCD=1;
//		HDLC_RX_BUF[HDLC_RX_LEN++]=CMX865A_RX_DATA();
//		if (HDLC_RX_LEN>295){break;}	   //限制数据，最多300字节，超长跳出
//	}
//
//	if (DCD==0){return 0;}	 //没信号跳出



    stu = CMX865A_READ_E6() ;		  //E6  //新的数据 ，B10=1 B6=1

    if ((stu & 0x0400) != 0x0400)
    {
        return 0;
    }

    HDLC_RX_LEN = 0;
    over_err =	fram_err = 0;

    while(1)
    {

        if ((stu & 0x0001) == 0x0001)
        {
            sig_in = 0;
        }
        else
        {
            {
                sig_in = 1;
            }
        }

        if ((stu & 0x0010) == 0x0010)
        {
            fram_err++;   //
        }

        if ((stu & 0x0020) == 0x0020)
        {
            over_err++;   //
        }


        if ((stu & 0x0040) == 0x0040)	 //收到新的数据 ，B6=1
        {
            HDLC_RX_BUF[HDLC_RX_LEN++] = CMX865A_READ_E5();

            if (HDLC_RX_LEN > 295)
            {
                break;   //限制数据，最多300字节，超长跳出
            }
        }

        stu = CMX865A_READ_E6() ;

        if ((stu & 0x0400) != 0x0400)
        {
            break;
        }


    }




//	UART1_SendData(0xaa);
//	for (i=0;i<len;i++)   {	UART1_SendData(HDLC_RX_BUF[i]);  }// 调试
//  for (i=0;i<30;i++)   {	UART1_SendData(0X00);  }// 调试

//	UART1_SendString("========\r\n"); 	   //带编号格式化调试
//	for (i=0;i<40;i++)
//	{
//	UART1_SendString("("); UART1_DEBUG2(i*5*8);  		    UART1_SendString("): ");
//
//	for (n=0;n<5;n++) 	{	DISP_HDLC(HDLC_RX_BUF[i*5+n]);  	}
//	UART1_SendString(" \r\n");
//	}
//	UART1_SendString("========\r\n");


    UART2_SendString("TATAL:  ");
    UART2_DEBUG(HDLC_RX_LEN);
    UART2_SendString("over:  ");
    UART2_DEBUG(over_err);
    UART2_SendString("err:  ");
    UART2_DEBUG(fram_err);


    RX_OK_COUNT++;	//解码成功计数
    UART2_SendString("RX COUNT:  ");
    UART2_DEBUG(RX_OK_COUNT);

    for (i = 0; i < HDLC_RX_LEN; i++)
    {
        DISP_HDLC(HDLC_RX_BUF[i]);
    }


//	return 0;

    TOTAL_IDX = HDLC_RX_LEN * 8; //总的索引长度
    HDLC_RX_IDX = 0;	  		 //起始检查7E索引位置	  //	UART1_SendString("TATAL:  ");	UART1_DEBUG(TOTAL_IDX);

    for (i = 0; i < 10; i++) 	 //最多连续解码10次
    {
        stu = HDLC_DECODE();
        UART2_SendString("err:  ");
        UART2_SendData(stu + 0x30);
        UART2_SendString("\r\n");

        if (stu == 5)
        {
            return 1;	   //解码成功
        }
    }


    return 0;
}




uchar CMX865A_HDLC_RX_2()			// 中断解码方式
{
    uint i;
    uchar stu;

    stu = 0;

    if (CMX_RX_BUSY == 1)
    {
//		 	LED_STU=0;

        RX_OK_COUNT++;	//解码成功计数
//   	UART2_SendString("TATAL:  ");	UART2_DEBUG(HDLC_RX_LEN);
//   	UART2_SendString("RX COUNT:  ");	UART2_DEBUG(RX_OK_COUNT);
//	for (i=0;i<HDLC_RX_LEN;i++) 	{	DISP_HDLC(HDLC_RX_BUF[i]);  	}


        TOTAL_IDX = HDLC_RX_LEN * 8; //总的索引长度
        HDLC_RX_IDX = 0;	 //起始检查7E索引位置

        for (i = 0; i < 10; i++) 	 //最多连续解码10次
        {
            stu = HDLC_DECODE();

//	UART2_SendString("err:  ");	UART2_SendData(stu+0x30);	 	UART2_SendString("\r\n");
            if (stu == 5)
            {
                break;	   //解码成功
            }
        }

        HDLC_RX_LEN = 0;	//接收长度清0
        CMX_RX_BUSY = 0;	//重新接收
    }

//	UART1_SendData(0xaa);
//	for (i=0;i<len;i++)   {	UART1_SendData(HDLC_RX_BUF[i]);  }// 调试
//  for (i=0;i<30;i++)   {	UART1_SendData(0X00);  }// 调试

//	UART1_SendString("========\r\n"); 	   //带编号格式化调试
//	for (i=0;i<40;i++)
//	{
//	UART1_SendString("("); UART1_DEBUG2(i*5*8);  		    UART1_SendString("): ");
//
//	for (n=0;n<5;n++) 	{	DISP_HDLC(HDLC_RX_BUF[i*5+n]);  	}
//	UART1_SendString(" \r\n");
//	}
//	UART1_SendString("========\r\n");

    return stu;
}

//读接收数据,状态寄存器如下变化，B5=0 B6 =0	3C 00   接收完成 3C 40	  接收溢出3C 60

void CMX_RX_INT()	//定时中断 ,5ms中断一次
{
    uint DCD;

//	return;

    if (PTT == 1)
    {
        return;   //如果在发射状态,则不接收数据
    }

    if (CMX_RX_BUSY == 1)
    {
        return;   //等待数据处理,则不接收数据
    }

    DCD = CMX865A_READ_E6(); //E6  //新的数据 ，B10=1 B6=1

    if ((DCD & 0x0400) != 0x0400)	 //B10=0信号消失	 数据长度<20,则作废,重新接收,否则，数据接收完成
    {
        if (HDLC_RX_LEN < 20)
        {
            HDLC_RX_LEN = 0;
        }
        else
        {
            CMX_RX_BUSY = 1;
        }

        return ;
    }

    //B10=1
    if ((DCD & 0x0040) != 0x0040)
    {
        return ;	 	   //B6=0	 没有可接收的数据
    }

    //B6=1
    HDLC_RX_BUF[HDLC_RX_LEN++] = CMX865A_READ_E5();

    if (HDLC_RX_LEN > 295)
    {
        CMX_RX_BUSY = 1;   //限制数据，最多300字节，超长跳出
    }
}

void CMX_RX_Initial()	//定时中断
{
    CMX_RX_BUSY = 0;
    HDLC_RX_LEN = 0;	//接收长度清0


    RX_OK_COUNT = 0;	//解码成功计数
}
