
#include <intrins.h>

#include "STC8A8K64D4.H"
#include "DELAY.H"
#include "AT24C512.h"
#include "IO.H"
#include "UART2.H"
#include "tostring.H"


sbit AT24_WP = P4 ^ 2;
sbit AT24_SCL = P2 ^ 5;
sbit AT24_SDA = P2 ^ 4;
 

unsigned char  AT24C512_RW_BUF[256];

 

/**************************************
延时5US*11.0592=55
**************************************/
void AT24_Delay()
{
    uchar i;

    for(i = 0; i < 10; i++)
    {
        _nop_();
        _nop_();
        _nop_();
    }
}

/**************************************
起始信号
**************************************/

void AT24C512_Start()
{
    AT24_SDA = 1;                    //拉高数据线
    AT24_Delay();                 //延时
    AT24_SCL = 1;                    //拉高时钟线
    AT24_Delay();                 //延时
    AT24_SDA = 0;                    //产生下降沿
    AT24_Delay();                 //延时
    AT24_SCL = 0;                    //拉低时钟线
    AT24_Delay();                 //延时
}
/**************************************
停止信号
**************************************/

void AT24C512_Stop()
{
    AT24_SDA = 0;                    //拉低数据线
    AT24_Delay();                 //延时
    AT24_SCL = 1;                    //拉高时钟线
    AT24_Delay();                 //延时
    AT24_SDA = 1;                    //产生上升沿
    AT24_Delay();                 //延时
}

/**************************************
发送应答信号
入口参数:ack (0:ACK 1:NAK)
**************************************/
void AT24CXX_ACK(bit ACK)
{
    AT24_SDA = ACK;                  //写应答信号
    AT24_SCL = 1;                    //拉高时钟线
    AT24_Delay();                 //延时
    AT24_SCL = 0;                    //拉低时钟线
    AT24_Delay();                 //延时
}


/**************************************
接收应答信号
**************************************/
bit AT24_RecvACK()
{
    AT24_SCL = 1;                    //拉高时钟线

    AT24_SDA = 1;
    AT24_Delay();                 //延时
    CY = AT24_SDA;                   //读应答信号

//	if (AT24_SDA==1)	{AT24_LINK=0;}  //没有应答，则AT24没安装
//	else{AT24_LINK=1;}

    AT24_SCL = 0;                    //拉低时钟线
    AT24_Delay();                 //延时

    return CY;
}

/**************************************
向IIC总线发送一个字节数据
**************************************/
uchar AT24CXX_write_byte(unsigned char value)
{
    unsigned char i;

    for (i = 0; i < 8; i++)     //8位计数器
    {
        value <<= 1;            //移出数据的最高位
        AT24_SDA = CY;        //送数据口
        AT24_Delay();            //延时
        AT24_SCL = 1;          //拉高时钟线
        AT24_Delay();            //延时
        AT24_SCL = 0;          //拉低时钟线
        AT24_Delay();             //延时
    }

    return AT24_RecvACK();		 // 0 =有应答  1=没有应答
}


/**************************************
从IIC总线接收一个字节数据
**************************************/
uchar AT24CXX_read_byte()
{
    uchar i;
    uchar dat = 0;

    AT24_SDA = 1;            //使能内部上拉,准备读取数据,

    for (i = 0; i < 8; i++)    //8位计数器
    {
        dat <<= 1;
        AT24_SCL = 1;         //拉高时钟线
        AT24_Delay();           //延时
        dat |= AT24_SDA;     //读数据
        AT24_SCL = 0;         //拉低时钟线
        AT24_Delay();            //延时
    }

    return dat;
}


void AT24CXX_WRITE(unsigned int addr, unsigned char wdata)
{
    AT24_WP = 0;
    AT24C512_Start();
    AT24CXX_write_byte(0xa0);
    AT24CXX_write_byte(addr / 256);
    AT24CXX_write_byte(addr % 256);
    AT24CXX_write_byte(wdata);
    AT24C512_Stop();
    AT24_WP = 1;
    Delay_time_1ms(5);	//写数据要加延时，不然死机，写不用加延时
}



unsigned char AT24CXX_READ(unsigned int addr)
{
    unsigned char i;
    AT24C512_Start();
    AT24CXX_write_byte(0xa0);
    AT24CXX_write_byte(addr / 256);
    AT24CXX_write_byte(addr % 256);

    AT24C512_Start();
    AT24CXX_write_byte(0xa1);
    i = AT24CXX_read_byte();
    AT24C512_Stop();

    return i;
}
//128字节的页写
unsigned char AT24CXX_WRITE_N(unsigned int addr, unsigned char  *buf, uint len)
{
    uint i;

    AT24_WP = 0;
    AT24C512_Start();
    AT24CXX_write_byte(0xa0);
    AT24CXX_write_byte(addr / 256);
    AT24CXX_write_byte(addr % 256);

    for(i = 0; i < len; i++)
    {
        AT24CXX_write_byte(*buf);
        buf++;
    }

    AT24C512_Stop();
    AT24_WP = 1;

    Delay_time_1ms(5);	//	//写数据必须要加延时

    return i;
}

//128字节的页读
void  AT24CXX_READ_N(unsigned int addr, unsigned char  *buf, uint len)
{
    uint i;
    AT24C512_Start();
    AT24CXX_write_byte(0xa0);
    AT24CXX_write_byte(addr / 256);
    AT24CXX_write_byte(addr % 256);

    AT24C512_Start();
    AT24CXX_write_byte(0xa1);

    for(i = 0; i < len; i++)
    {
        *buf = AT24CXX_read_byte();
        buf++;
        AT24CXX_ACK(i == (len - 1));
    }

    AT24C512_Stop();

}




//
//void AT2C512_TEST()
//{
//	uint i;		   uchar temp[20];
//
////	AT24CXX_WRITE(0x0000,0x0E);	 	//指定地址写入一个字节
////	UART1_SendData(AT24CXX_READ(0x0000));	 //指定地址读出一个字节
////	for(i=0;i<256;i++)	 {UART1_SendData(AT24CXX_READ(0x0000+i));  }
////	AT24CXX_WRITE_N(0x0000,AT24C512_W_BUF,128);		 //指定地址写入128个字节
////	AT24CXX_READ_N(0x0000,AT24C512_R_BUF,128);		//指定地址读出128个字节
////	for(i=0;i<128;i++)	 {  UART1_SendData(AT24C512_R_BUF[i]); }
//
//	AT24CXX_READ_N(0,AT24C512_RW_BUF,20);
//	DEBUG_KISS(AT24C512_RW_BUF,20);
//
//	for (i = 0; i<9; i++) { temp[i]=AT24CXX_READ(0+16+i); temp[i+1]=0; }  //呼号
//	DEBUG_KISS(temp,9);
//
//
//}
//

void AT2C512_CLEAN()   //清除全部记录0-511,清除索引
{
    uint i, n;
    UART2_SendString("Wait...\r\n");

    for(i = 0; i < 128; i++)
    {
        AT24C512_RW_BUF[i] = 0xff;
    }

    for(n = 0; n < 512; n++)
    {
        AT24CXX_WRITE_N(n * 128, AT24C512_RW_BUF, 128);		   //指定地址写入128个字节
    }

    UART2_SendString("DATA Clean OK\r\n");
}

//  0x0000-0x6400  （0-25600）,每条信标占用256字节，一共100条记录，索引0-99存储在 0x8000
//  0x6500-0x7180    ,列表存储区，每16字节一套索引，一共100条索引
//  BH4TDV-XX 99 信标存储地址索引 0-99
//	0xFE00  最后512 存储设置数据
