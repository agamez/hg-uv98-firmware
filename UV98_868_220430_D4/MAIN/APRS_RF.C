#include "STC8A8K64D4.H"
#include "STC_EEPROM.H"

#include "UART1.H"
#include "UART2.H"
#include "DELAY.H"

#include "CRC16.H"
#include "APRS_RF.H"
#include "io.H"
#include "KISS_DECODE.H"

#include "CMX865A_SPI.H"
#include "PUBLIC_BUF.H"
#include "tostring.H"


/********* APRS编解码 ***********/
//sbit FX604_M1=P4^5;
//sbit FX604_M0=P2^7;
//sbit PTT 	 =P2^6;
//sbit FX604_DET=P2^4;
//sbit FX604_RX=P2^3;
//sbit FX604_TX=P2^2;

bit aprs_flag;	 //检查是否是flag=7E标志
bit old_input;	 //管脚反转状态
bit input_rev;	 //电平是否反转标记


void Delay_417us();					 //编解码定时
void DELAY_833US();	  //使用 做定时器, 延时833us

unsigned char APRS_STATUS;			//APRS解码状态，=03正常，其他指示为意外断开、常音、杂音、数据同步异常等错误
unsigned char one_count;	 		//编解码，连续1的计数
unsigned char HDLC_DATA;			//APRS解码得到1个字节

uchar  DATA_7E;	//0=前后标志，1=数据7E

unsigned char RF_DECODE();			 //RF APRS解码

uchar check_crc_a();		 //校验数据最后2位

/*****************************************/
void DELAY_TX_833US();	  //使用TIME1做定时器, 延时833us


sbit FX604_RX  = P1 ^ 6;



void DELAY_833US()	  //使用TIME1做定时器, 延时833us
{
    CCON = 0;					//清除CF、CR、CCF0、CCF1
    CH = (65536 - 760 * 2) / 256;		//PCA基准定时器设初值。
    CL = (65536 - 760 * 2) % 256;		//;65536-22.1184M振荡MHZ/12*833US	 =65536-1535
    CR = 1;						//启动PCA定时器。
}

void Delay_417us()	  		//使用TIME1做定时器,延时417us
{
    CCON = 0;					//清除CF、CR、CCF0、CCF1
    CH = (65536 - 384 * 2) / 256;		//PCA基准定时器设初值。
    CL = (65536 - 384 * 2) % 256;		//;65536-11.0592M振荡MHZ/12*50000US	 =65536-46080
    CR = 1;						//启动PCA定时器。

    while (!CF);
}





uchar check_crc_a()		 //校验数据最后2位
{
    GetCrc16_LEN(KISS_DATA, (KISS_LEN - 2));	 //计算数据的校验值

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



//检索首个FLAG=7E
void APRS_check_flag()
{
    unsigned char i;
    uint stu;

    APRS_STATUS = 0x00;
    HDLC_DATA = 0x00;

    if ((CMX865A_READ_E6() & 0x0001) == 0x0001)
    {
        FX604_RX = 0;
    }
    else
    {
        FX604_RX = 1;
    }

    old_input = FX604_RX;	 //记录输入电平

    one_count = 0;	  //连续1的计数
    i = 0;

    while (1)
    {
        Delay_417us();

        HDLC_DATA  >>= 1;			//右移1位,默认写0

        if  (one_count == 6)		  	// ;当前=0，前面6个1，发现到首个FLAG=7E
        {
            i++;		 	//连续检测到3个FLAG=7E

            if (i == 3)
            {
                APRS_STATUS = 0x03;
                one_count = 0;
                return;	 //检索成功，返回
            }    	//连续1的计数
        }

        one_count = 0;	    	//连续1的计数
        input_rev = 0;		 	//电平翻转标记

        while (input_rev == 0)
        {
            DELAY_833US();		//延时833us

            while (!CF)
            {
                stu = CMX865A_READ_E6();

                if ((stu & 0x0400) != 0x0400)
                {
                    APRS_STATUS = 0x04;    //异常错误跳出	    //E6  //新的数据 ，B10=1 B6=1
                    return;
                }

                if ((stu & 0x0001) == 0x0001)
                {
                    FX604_RX = 0;
                }
                else
                {
                    FX604_RX = 1;
                }

                if (FX604_RX != old_input)
                {
                    old_input = FX604_RX;	 //记录输入电平
                    input_rev = 1;	 //发生电平翻转
                    break;
                }
            }

            if (input_rev == 0)	  	//没翻转，写1
            {
                HDLC_DATA  >>= 1;	//先右移1位
                HDLC_DATA |= 0x80; 	  //写1

                one_count++;		  //连续1的计数+1

                if (one_count == 7)
                {
                    APRS_STATUS = 0x05;
                    return;
                }

                //;发生错误，连续7个"1",可能是长音或非APRS数据	 	//; 重新检测
            }
        }
    }
}




void APRS_RECDATA()
{
    unsigned char bit_count;
    uint stu;

    DATA_7E = 0;	 //数据7E标志清0

    bit_count = 0; //接收8位

    while (bit_count < 8)
    {
        DELAY_833US();	    //延时833us

        while (!CF)
        {
            stu = CMX865A_READ_E6();

            if ((stu & 0x0400) != 0x0400)
            {
                APRS_STATUS = 0x04;    //异常错误跳出	    //E6  //新的数据 ，B10=1 B6=1
                return;
            }

            if ((stu & 0x0001) == 0x0001)
            {
                FX604_RX = 0;
            }
            else
            {
                FX604_RX = 1;
            }

            if (FX604_RX != old_input)
            {
                old_input = FX604_RX;	 //记录输入电平
                input_rev = 1;	 //发生电平翻转
                break;
            }
            else
            {
                input_rev = 0;   //电平没翻转标记
            }
        }

        if (input_rev == 0)	  	//没翻转，写1
        {
            HDLC_DATA  >>= 1;	//先右移1位
            HDLC_DATA |= 0x80; 	//写1

            one_count++;		  //连续1的计数+1

            if (one_count == 7)
            {
                APRS_STATUS = 0x05;
                return;
            }

            //;发生错误，连续7个"1",可能是长音或非APRS数据
            //; 重新检测
            bit_count++;
        }
        else		   //发生电平翻转
        {
            Delay_417us();

            if (one_count == 6)
            {
                DATA_7E = 1;	   //是7E标志
            }

            if (one_count == 5)	//前面连续5个“1”，则忽略此位
            {

            }
            else
            {
                HDLC_DATA  >>= 1;	//右移1位,默认写0
                bit_count++;
            }

            one_count = 0;	  //连续1的计数


        }
    }
}


unsigned char RF_DECODE()	   //RF解码KISS数据
{
    KISS_LEN = 0;				//接收首个KISS数据

    APRS_check_flag();		 //寻找数据包前部首个FLAG=7E

    if (APRS_STATUS != 0x03)
    {
        return 1;    //如果断线、杂音、长音等异常错误跳出
    }

    if (HDLC_DATA != 0x7e)
    {
        return 2;   //如果断线、杂音、长音等异常错误跳出
    }

    //等待数据包前部的7E接收完毕

    while (1)		//等待数据包头部的7E全部接收完毕
    {
        APRS_RECDATA();

        if (HDLC_DATA != 0x7e)
        {
            break;
        }

        if (APRS_STATUS != 0x03)
        {
            return 3;     //如果断线、杂音、长音等异常错误跳出
        }
    }


    KISS_LEN = 0;				//接收首个KISS数据
    KISS_DATA[KISS_LEN] = HDLC_DATA;
    KISS_LEN++;



    while (1)		//数据包中部
    {
        APRS_RECDATA();

        if (APRS_STATUS != 0x03)
        {
            return 4;      //如果断线、杂音、长音等异常错误跳出
        }

        KISS_DATA[KISS_LEN] = HDLC_DATA;
        KISS_LEN++;	   //存入数据

        if (KISS_LEN > 120)
        {
            return 5;   //接收长度超长，如同时2个电台在发射，数据部分重叠，异常错误跳出
        }

        if (DATA_7E == 1)
        {
            break ;   //结尾标志7E
        }
    }

    KISS_LEN = KISS_LEN - 1;		 //接收的数据总长度，去掉最后接收的1个7E字节，包含2字节或1字节校验

//	KISS_LEN=KISS_LEN-2;		 //接收的数据总长度，去掉最后接收的2个7E字节，包含2字节或1字节校验
    if 	   (check_crc_a() == 1)
    {
        return 6;   //校验数据最后2位
    }

//	if 	   (check_crc_b()==1)  {return 1;}	   //校验数据最后1位+7E
    return 7;				   //校验数据错误
}
