#include "STC8A8K64D4.H"
#include "STC_EEPROM.H"

#include "CMX865A_SPI.H"

#include "UART1.H"
#include "UART4.H"


#include "DELAY.H"
#include "io.H"
#include "CRC16.H"
#include "DELAY.H"
#include "CMX865A_DECODE.H"
#include  <INTRINS.H> //Keil library 



/************* 定义W5100 CBUS的引脚端口 **************/
//sbit CMX865A_SCS	=P2^1;	//	CBUS	 定义CBUS的片选信号端口
//sbit CMX865A_SCLK	=P2^0;	//	CBUS	 定义CBUS的时钟信号端口
////sbit CMX865A_RST	=P0^3;	//	CBUS
//sbit CMX865A_MOSI	=P4^2;	//	CBUS	 定义CBUS的MOSI端口
//sbit CMX865A_MISO	=P4^1;	//	CBUS	 定义CBUS的MISO端口


//uchar CMX86X_TX_VOL;	 //1 0-7
//uchar CMX86X_RX_VOL;	 //1 0-7


//uchar CMX86X_TX_VOL;	 //1 0-7
//uchar CMX86X_RX_VOL;	 //1 0-7


#define CMX86X_TX_VOL  	EEPROM_Buffer[0X1D3]	 //1 0-7
#define CMX86X_RX_VOL  	EEPROM_Buffer[0X1D4]	 //1 0-7


//#define TX_MODE 0x90   //0X00-0XF0
//#define RX_MODE 0X90   //0X00-0XF0

#define TX_MODE 0x30   //0X00-0XF0
#define RX_MODE 0X30   //0X00-0XF0	
//

//标准	速率	音调1、0	应用
//BELL 202	1200	1200\2200	UV 	   0x30
//V.23  	1200	1300\2100	UV 	   0x50
//V.21 H	300	1650\1850	HF		 0x90
//V.21 L	300	980\1180	HF		 0x80

//BELL 103 H	300	2225\2025	HF	 0x70
//BELL 103 L	300	1270\1070	HF	 0x60


//b15 b14  b13  b12
//1  1  1  1  V.22 bis 2400 bps QAM  High band (Answering modem)
//1  1  1  0  “  Low band (Calling modem)
//1  1  0  1  V.22/Bell 212A 1200 bps DPSK  High band (Answering modem)
//1  1  0  0  “  Low band (Calling modem)
//1  0  1  1  V.22 600 bps DPSK  High band (Answering modem)
//1  0  1  0  “  Low band (Calling modem)
//1  0  0  1  V.21 300 bps FSK  High band (Answering modem)
//1  0  0  0  “  Low band (Calling modem)
//0  1  1  1  Bell 103 300 bps FSK  High band (Answering modem)
//0  1  1  0  “  Low band (Calling modem)
//0  1  0  1  V.23 FSK  1200 bps
//0  1  0  0  “  75 bps
//0  0  1  1  Bell 202 FSK  1200 bps
//0  0  1  0  “  150 bps
//0  0  0  1  DTMF / Tones
//0  0  0  0  Transmitter disabled 1  0  0  0  “  Low band (Calling modem)





//
//void CMX865A_833US()	  //使用TIME1做定时器, 延时833us
//{
//   	CCON = 0;					//清除CF、CR、CCF0、CCF1
//	CH = (65536-760*2)/256;		//PCA基准定时器设初值。
//	CL = (65536-760*2)%256;		//;65536-22.1184M振荡MHZ/12*833US	 =65536-1535
//	CR = 1;						//启动PCA定时器。
//		while (!CF);
//}


void CMX865A_Delay5us()		//@@22.1184MHz
{
//	unsigned char i;

    _nop_();
    _nop_();
    _nop_();
    _nop_();

//	i = 5;
//	while (--i);
}
//********************************************************************/
//选择同步数据模式，则在TX数据缓冲器中的8个数据位串行传输；
//数据B0首先被发送，因此是反的,原始数据需要高位低位交换后在发送
//同步数据模式，数据B0首先被发送
void CBUS_WRITE_E3 ( unsigned char dat )	//发送数据
{
    unsigned char i;

    for ( i = 0; i < 8; i++)
    {
        CMX865A_SCLK = 0; //	_nop_();	  //先发低位

        if ( (dat & 0x01) == 0X01 )
        {
            CMX865A_MOSI = 1;
        }
        else
        {
            CMX865A_MOSI = 0;
        }

        dat >>= 1;

//			if ( (dat & 0x80)==0X80 )   {CMX865A_MOSI=1; }  else    { CMX865A_MOSI=0; }
//			dat <<= 1;

        CMX865A_SCLK = 1; //	_nop_();
    }

//	    CMX865A_Delay5us(); 	//停止位延时
}
//同步数据模式，数据B0首先被送出
uchar CBUS_READ_E5 ()	 //读数据
{
    unsigned char i, j;

    j = 0;

    for( i = 0; i < 8; i++ )
    {
        CMX865A_SCLK = 0; //	_nop_();
//            j <<= 1;				//先接收高位
//            if( CMX865A_MISO==1 )  { j |= 0x01;}

        j >>= 1;				//先接收低位

        if( CMX865A_MISO == 1 )
        {
            j |= 0x80;
        }

        //_nop_();  // _nop_();
        CMX865A_SCLK = 1; //	_nop_();
    }

    return j;
}
//********************************************************************/


/********************************************************************
通过CBUS总线输出一个字节
********************************************************************/
void CBUS_WRITE_REG ( unsigned char dat )
{
    unsigned char i;

    for ( i = 0; i < 8; i++)
    {
        CMX865A_SCLK = 0; // _nop_();	  //先发高位

        if ( (dat & 0x80) == 0X80 )
        {
            CMX865A_MOSI = 1;
        }
        else
        {
            CMX865A_MOSI = 0;
        }

        dat <<= 1;
        CMX865A_SCLK = 1; //	_nop_();
    }
}

uchar CBUS_READ_REG ()
{
    unsigned char i, j;

    j = 0;

    for( i = 0; i < 8; i++ )
    {
        CMX865A_SCLK = 0; //	_nop_();  	     //先读高位

        j <<= 1;

        if( CMX865A_MISO == 1 )
        {
            j |= 0x01;
        }

        CMX865A_SCLK = 1; //	_nop_();

    }

    return j;
}

void CMX865A_RESET()
{
    CMX865A_SCS = 0;
    CMX865A_Delay5us();
    CBUS_WRITE_REG(0X01);
    CMX865A_SCS = 1;
    Delay_time_1ms(20);
}



void CMX865A_WRITE_TWO(uchar addr, uchar dat1, uchar dat2 )
{
    CMX865A_SCS = 0; //	 CMX865A_Delay5us();
    CBUS_WRITE_REG ( addr );
    CBUS_WRITE_REG ( dat1 );
    CBUS_WRITE_REG ( dat2 );
    CMX865A_SCS = 1;
}

uint CMX865A_READ_TWO(uchar addr )
{
    uint dat_msb, dat_lsb;
    CMX865A_SCS = 0; // CMX865A_Delay5us();
    CBUS_WRITE_REG ( addr );
    dat_msb = CBUS_READ_REG ();
    dat_lsb = CBUS_READ_REG ();
    CMX865A_SCS = 1;
    return dat_msb * 256 + dat_lsb;
}




uint CMX865A_READ_E6()  //E6
{
    uint stu;
    stu = CMX865A_READ_TWO(0XE6) ;		   //新的数据 ，B10=1 B6=1
//	if ((stu&0x0440)==0x0440)	 	{   return 1;	 	}
    return stu;
}


uchar CMX865A_READ_E5()
{
    uchar dat;

    CMX865A_SCS = 0; //CMX865A_Delay5us();
    CBUS_WRITE_REG ( 0XE5 );
    dat = CBUS_READ_E5 ();
    CMX865A_SCS = 1;

    return dat;
}


////读接收数据,状态寄存器如下变化，B5=0 B6 =0	3C 00   接收完成 3C 40	  接收溢出3C 60
//uchar CMX865A_RX_DATA()
//{	uchar dat;	   uint stu;
//
//
//   	stu= CMX865A_READ_TWO(0XE6)  ;
//	while((stu&0x0040)!=0x0040)	   //收到新的数据 ，B6=1
//	{
//	stu= CMX865A_READ_TWO(0XE6)  ;
//	}
//
//	CMX865A_SCS=0; // CMX865A_Delay5us();
//	CBUS_WRITE_REG ( 0XE5 );
//	dat=CBUS_READ_E5 ();
//	CMX865A_SCS=1;
//
//	return dat;
//}

//********************************************************************/
//写入待发数据,状态寄存器如下变化，B12=0 B11 =0	24 60   准备发送 34 60	  发送溢出3C 60
void CMX865A_TX_DATA( uchar dat )
{
    uint stu;

    CMX865A_SCS = 0; //CMX865A_Delay5us();
    CBUS_WRITE_REG ( 0xE3 );
    CBUS_WRITE_E3 ( dat );
    CMX865A_SCS = 1;

    stu = CMX865A_READ_TWO(0XE6)  ;

    while((stu & 0x1000) != 0x1000)	 // //等待发送完毕，B12=1
    {
        stu = CMX865A_READ_TWO(0XE6)  ;
    }
}

//********************************************************************/
//void  CMX865A_TX_TONE(uchar tone)  //tone=1 1200  0=2200hz
//{
//	if (tone==1){CMX865A_WRITE_TWO(0xE1,0x30|(7<<1),0X1B);}else{CMX865A_WRITE_TWO(0xE1,0x30|(7<<1),0X1A);}
//}

void CMX865A_TX_OFF()
{
    CMX865A_WRITE_TWO(0xE1, 0x00 | (CMX86X_TX_VOL << 1), 0X1C);	 //关闭发送器
}

//void CMX865A_TONE_TEST()
//{	PTT=1;	 LED_RED=0;  Delay_time_1ms(10);
//
//	CMX865A_WRITE_TWO(0xE1,TX_MODE|(CMX86X_TX_VOL<<1),0X1B);	 //连续发 1	1200HZ
//	Delay_time_25ms(40*10);
////	CMX865A_WRITE_TWO(0xE1,TX_MODE|(CMX86X_TX_VOL<<1),0X1A);	 //连续发 0	2200hz
////	Delay_time_25ms(40*5);
////	CMX865A_WRITE_TWO(0xE1,TX_MODE|(CMX86X_TX_VOL<<1),0X1C);
//	CMX865A_TX_OFF();					  //关闭发送器
//
//
//	LED_RED=1;		PTT=0;
//}



void TONE1200()
{
    PTT = 1;
    LED_STU = 0;
    Delay_time_1ms(10);
    UART4_SendString("$E01\r\n");

    CMX865A_WRITE_TWO(0xE1, 0x30 | (CMX86X_TX_VOL << 1), 0X1B);	 //连续发 1	1200HZ

}
void TONE2200()
{
    PTT = 1;
    LED_STU = 0;
    Delay_time_1ms(10);
    UART4_SendString("$E01\r\n");

    CMX865A_WRITE_TWO(0xE1, 0x30 | (CMX86X_TX_VOL << 1), 0X1A);	 //连续发 0	2200HZ

}

void TONE_OFF()
{
    PTT = 1;
    LED_STU = 0;
    Delay_time_1ms(10);
    CMX865A_WRITE_TWO(0xE1, 0x00 | (CMX86X_TX_VOL << 1), 0X1C);	 //关闭发送器
    PTT = 0;
    LED_STU = 1;
    UART4_SendString("$E00\r\n");
}


void CMX865A_TX_ON()
{
//V2.3 FSK 0x50
//bell 202 FSK  0x30
//tx level (0-7) 7=0DB最大

//TX设置bell 202 FSK模式，tx level=0DB, Tx Synchronous mode, Data bytes from Tx Data Buffer

//标准	速率	音调1、0	应用
//BELL 202	1200	1200\2200	UV 	   0x30
//V.23  	1200	1300\2100	UV 	   0x50
//V.21 H	300	1650\1850	HF		 0x90
//V.21 L	300	980\1180	HF		 0x80

//BELL 103 H	300	2225\2025	HF	 0x70
//BELL 103 L	300	1270\1070	HF	 0x60

//b15 b14  b13  b12
//1  0  0  1  V.21 300 bps FSK  High band (Answering modem)
//1  0  0  0  “  Low band (Calling modem)
//0  1  1  1  Bell 103 300 bps FSK  High band (Answering modem)
//0  1  1  0  “  Low band (Calling modem)
//0  1  0  1  V.23 FSK  1200 bps
//0  0  1  1  Bell 202 FSK  1200 bps
//0  0  0  0  Transmitter disabled 1  0  0  0  “  Low band (Calling modem)

//	CMX865A_WRITE_TWO(0xE1,TX_MODE|(7<<1),0X1C);	 //发送实际数据


    CMX865A_WRITE_TWO(0xE1, TX_MODE | (CMX86X_TX_VOL << 1), 0X1C);	 //发送实际数据

//  CMX865A_WRITE_TWO(0xE1,TX_MODE|(7<<1),0X1B);	 //连续发 1	1200HZ
//  CMX865A_WRITE_TWO(0xE1,TX_MODE|(7<<1),0X1A);	 //连续发 0	2200hz
//  CMX865A_WRITE_TWO(0xE1,TX_MODE|(1<<1),0X18);	 //连续发交替1 0






//TX设置V.23 FSK模式，tx level=0DB, Tx Synchronous mode, Data bytes from Tx Data Buffer
//	CMX865A_WRITE_TWO(0xE1,TX_MODE|(7<<1),0X1C);	 //发送实际数据
//  CMX865A_WRITE_TWO(0xE1,TX_MODE|(7<<1),0X1B);	 //连续发 1	1200HZ
//  CMX865A_WRITE_TWO(0xE1,TX_MODE|(7<<1),0X1A);	 //连续发 0	2200hz
//  CMX865A_WRITE_TWO(0xE1,TX_MODE|(7<<1),0X18);	 //连续发交替1 0

//	CMX865A_WRITE_TWO(0xE1,0x00|(7<<1),0X1C);	 //关闭发送器
}

void CMX865A_PWUP()	   //掉电模式
{
    CMX_RX_BUSY = 1;
    Delay_time_1ms(10);   //防止通知在解码读取状态
    CMX865A_WRITE_TWO(0xE0, 0x00, 0X00); //掉电模式
}


//void read_cmx_set()
//{
//		   			UART1_SendData(CMX86X_TX_VOL+0X30);
//
//			UART1_SendData(CMX86X_RX_VOL+0X30);
//
//}


void CMX_RX_ON()
{

    CMX_RX_BUSY = 1;
    Delay_time_1ms(10);   //防止通知在解码读取状态
    CMX865A_WRITE_TWO(0xE2, RX_MODE | (CMX86X_RX_VOL << 1), 0X3E);	//-10.5dB
    CMX_RX_BUSY = 0;

}

//void CMX_RX_OFF()
//{
//	CMX865A_WRITE_TWO(0xE2,0x00|(CMX86X_RX_VOL<<1),0X3E);	//-10.5dB
//}

//初始化CMX865A相应寄存器
void CMX865A_Init()
{
//	CMX86X_TX_VOL=CMX86X_TX_VOL;	 //1
//	CMX86X_RX_VOL=CMX86X_RX_VOL;	 //1

// CMX865A_SCS	=CMX865A_SCLK=CMX865A_MOSI=CMX865A_MISO=1;	//	CBUS	 定义CBUS的片选信号端口

    CMX865A_RESET();  //接通电源后，先复位
    Delay_time_25ms(1);	//必须延时20ms,等待时钟和VBIAS启动
    CMX865A_WRITE_TWO(0xE0, 0x11, 0X80); //b8=b7=1   接通电源并复位
    Delay_time_25ms(1);	//必须延时20ms,等待时钟和VBIAS启动

//TXAN=OUT TXA=ON，关闭闭环测试，1200MODEM,开电源 ,启用IRQ中断输出
//B7:设置此位为1重置CMX865的内部电路，清除所有比特的传输和
//状态寄存器的接收模式寄存器、编程寄存器和B13-0

//   CMX865A_WRITE_TWO(0xE0,0x01,0X00);  //正常状态
//   CMX865A_WRITE_TWO(0xE0,0x09,0X00);  //开启循环测试

    CMX865A_WRITE_TWO(0xE0, 0x11, 0X40); //正常状态	11.0592M 开启滤波器


//   CMX865A_WRITE_TWO(0xE0,0x15,0X40);  //正常状态	11.0592M 关闭滤波器

//   CMX865A_WRITE_TWO(0xE0,0x09,0X40);  //开启循环测试
//********************************************************************/
//   CMX865A_TX_ON();
    CMX865A_TX_OFF();
//********************************************************************/
//RX设置bell 202 FSK模式，Rx level=0DB, Rx Synchronous mode, 8 data bits
//	CMX865A_WRITE_TWO(0xE2,0x30|(7<<1),0X3E);	//0dB

    CMX_RX_ON();
//	CMX865A_WRITE_TWO(0xE2,0x30|(CMX86X_RX_VOL<<1),0X3E);	//-10.5dB



//RX设置V.23 FSK模式，Rx level=0DB, Rx Synchronous mode, 8 data bits
//	CMX865A_WRITE_TWO(0xE2,0x50|(7<<1),0X3E);
//	CMX865A_WRITE_TWO(0xE2,0x30|(7<<1),0X0E);	 //关闭接收器
//********************************************************************/
}
