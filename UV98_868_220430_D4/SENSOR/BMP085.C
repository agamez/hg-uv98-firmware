
/************* BMP085传感器 **************/

#include "STC15Fxxxx.H"
#include  <INTRINS.H> //Keil library 
#define   uchar unsigned char	   //0-255单字节	用uchar 代替char
#define   uint unsigned int		   //0-65536双字节	用uint 代替int

#include "tostring.H"

#include "BMP085.H"
#include "bmp280.h"


#define	  BMP085_SlaveAddress   0xee	  //定义器件在IIC总线中的从地址                               
#define   OSS 0	// Oversampling Setting (note: code is not set up to use other OSS values)


sbit	SCL=P4^2;      //IIC时钟引脚定义
sbit	SDA=P4^3;      //IIC数据引脚定义
 	
//unsigned char  bmp085_ge,bmp085_shi,bmp085_bai,bmp085_qian,bmp085_wan,bmp085_shiwan;           //显示变量
//unsigned char  bmp085_tp_ge,bmp085_tp_shi,bmp085_tp_bai;

//unsigned char QY_SHIWAN,QY_WAN,QY_QIAN,QY_BAI,QY_SHI;

uchar QY[10];       
 
unsigned int dis_data;                              //变量

short ac1,ac2,ac3;
unsigned short ac4,ac5,ac6;
short b1,b2; 
short mb,mc,md;

void delay(unsigned int k);
void conversion(long temp_data);

void  Single_Write(uchar SlaveAddress,uchar REG_Address,uchar REG_data);   //单个写入数据
uchar Single_Read(uchar REG_Address);                                      //单个读取内部寄存器数据
void  Multiple_Read(uchar,uchar);                                          //连续的读取内部寄存器数据
//------------------------------------
void Delay5us();
void Delay5ms();
void BMP085_Start();
void BMP085_Stop();
void BMP085_SendACK(bit ack);
bit  BMP085_RecvACK();
void BMP085_SendByte(uchar ndat);
uchar BMP085_RecvByte();
void BMP085_ReadPage();
void BMP085_WritePage();



uchar bmp085Convert();
void Init_BMP085();
//-----------------------------------

bit BMP085_LINK;	//0=没安装	1=安装


//*********************************************************
//void conversion(long temp_data)  
//{  
//    
//    bmp085_shiwan=temp_data/100000+0x30 ;
//    temp_data=temp_data%100000;   //取余运算 
//
//    bmp085_wan=temp_data/10000+0x30 ;
//    temp_data=temp_data%10000;   //取余运算
//
//	bmp085_qian=temp_data/1000+0x30 ;
//    temp_data=temp_data%1000;    //取余运算
//
//    bmp085_bai=temp_data/100+0x30   ;
//    temp_data=temp_data%100;     //取余运算
//
//    bmp085_shi=temp_data/10+0x30    ;
//    temp_data=temp_data%10;      //取余运算
//
//    bmp085_ge=temp_data+0x30; 	
//}
//
///*******************************/
		
 
/**************************************
延时5微秒(STC90C52RC@12M)
不同的工作环境,需要调整此函数，注意时钟过快时需要修改
当改用1T的MCU时,请调整此延时函数

1TCPU x=5US*11.0592=55
1TCPU x=5US*22.1184=110
1TCPU x=5US*33.1776=165
**************************************/
//void Delay5us()
//{
//	uchar i ,n;
//	
////	for(i=0;i<55;i++)	{n++;}		//11.0592M
//	
//	for(i=0;i<110;i++)	{n++;}		//22.1184M
//	
////	for(i=0;i<165;i++)	{n++;}		//33.1776M
//}
void Delay5us()		//@22.1184MHz
{
	unsigned char i;

	_nop_();
	i = 25;
	while (--i);
}

/**************************************
延时5毫秒(STC90C52RC@12M)
不同的工作环境,需要调整此函数
当改用1T的MCU时,请调整此延时函数
**************************************/
//;延时5ms		  ;65536-11.0592M振荡MHZ/12*5000US	 =65536-4608
void Delay5ms()
{	/*	*/
//   	TR1=0;
//	TF1=0;
//	TH1=(65536-4608)/256;
//	TL1=(65536-4608)%256;	
//    TR1=1;
//	while (!TF1);
	

//		CCON = 0;					//清除CF、CR、CCF0、CCF1
//	CH = (65536-4608)/256;		//PCA基准定时器设初值。
//	CL = (65536-4608)%256;		//;65536-11.0592M振荡MHZ/12*5000US	 =65536-4608
//	CR = 1;						//启动PCA定时器。
//	while (!CF);


	CCON = 0;					//清除CF、CR、CCF0、CCF1
	CH = (65536-9216)/256;		//PCA基准定时器设初值。
	CL = (65536-9216)%256;		//;65536-22.1184M振荡MHZ/12*5000US	 =65536-9216
	CR = 1;						//启动PCA定时器。
	while (!CF);

//   	CCON = 0;					//清除CF、CR、CCF0、CCF1
//	CH = (65536-13824)/256;		//PCA基准定时器设初值。
//	CL = (65536-13824) %256;	//65536-33.1776M振荡MHZ/12*5000US	 =65536-13824
//	CR = 1;						//启动PCA定时器。
//	while (!CF);


}



/**************************************
起始信号
**************************************/
void BMP085_Start()
{
    SDA = 1;                    //拉高数据线
    SCL = 1;                    //拉高时钟线
    Delay5us();                 //延时
    SDA = 0;                    //产生下降沿
    Delay5us();                 //延时
    SCL = 0;                    //拉低时钟线
}

/**************************************
停止信号
**************************************/
void BMP085_Stop()
{
    SDA = 0;                    //拉低数据线
    SCL = 1;                    //拉高时钟线
    Delay5us();                 //延时
    SDA = 1;                    //产生上升沿
    Delay5us();                 //延时
   
}

/**************************************
发送应答信号
入口参数:ack (0:ACK 1:NAK)
**************************************/
void BMP085_SendACK(bit ack)
{
    SDA = ack;                  //写应答信号
    SCL = 1;                    //拉高时钟线
    Delay5us();                 //延时
    SCL = 0;                    //拉低时钟线
    Delay5us();                 //延时
}

/**************************************
接收应答信号
**************************************/
bit BMP085_RecvACK()
{
    SCL = 1;                    //拉高时钟线
    Delay5us();                 //延时
    CY = SDA;                   //读应答信号
		
	if (SDA==1)	{BMP085_LINK=0;}  //没有应答，则BMP085没安装
	else{BMP085_LINK=1;}

    SCL = 0;                    //拉低时钟线
    Delay5us();                 //延时

    return CY;
}

/**************************************
向IIC总线发送一个字节数据
**************************************/
void BMP085_SendByte(uchar dat)
{
    uchar i;

    for (i=0; i<8; i++)         //8位计数器
    {
        dat <<= 1;              //移出数据的最高位
        SDA = CY;               //送数据口
        SCL = 1;                //拉高时钟线
        Delay5us();             //延时
        SCL = 0;                //拉低时钟线
        Delay5us();             //延时
    }
    BMP085_RecvACK();
}

/**************************************
从IIC总线接收一个字节数据
**************************************/
uchar BMP085_RecvByte()
{
    uchar i;
    uchar dat = 0;

    SDA = 1;                    //使能内部上拉,准备读取数据,
    for (i=0; i<8; i++)         //8位计数器
    {
        dat <<= 1;
        SCL = 1;                //拉高时钟线
        Delay5us();             //延时
        dat |= SDA;             //读数据               
        SCL = 0;                //拉低时钟线
        Delay5us();             //延时
    }
    return dat;
}
/*
//单字节写入BMP085内部数据*******************************

void Single_Write(uchar SlaveAddress,uchar REG_Address,uchar REG_data)
{
    BMP085_Start();                  //起始信号
    BMP085_SendByte(SlaveAddress);   //发送设备地址+写信号
    BMP085_SendByte(REG_Address);    //内部寄存器地址
    BMP085_SendByte(REG_data);       //内部寄存器数据
    BMP085_Stop();                   //发送停止信号
}
*/
/*
//单字节读取BMP085内部数据********************************
uchar Single_Read(uchar REG_Address)
{  uchar REG_data;
    BMP085_Start();                          //起始信号
    BMP085_SendByte(BMP085_SlaveAddress);           //发送设备地址+写信号
    BMP085_SendByte(REG_Address);            //发送存储单元地址	
    BMP085_Start();                          //起始信号
    BMP085_SendByte(BMP085_SlaveAddress+1);         //发送设备地址+读信号
    REG_data=BMP085_RecvByte();              //读出寄存器数据
	BMP085_SendACK(1);   
	BMP085_Stop();                           //停止信号
    return REG_data; 
}
*/
//*********************************************************
//读出BMP085内部数据,连续两个
//*********************************************************
short Multiple_read(uchar ST_Address)
{   
	uchar msb, lsb;
	short _data;
    BMP085_Start();                          //起始信号
    BMP085_SendByte(BMP085_SlaveAddress);    //发送设备地址+写信号
    BMP085_SendByte(ST_Address);             //发送存储单元地址
    BMP085_Start();                          //起始信号
    BMP085_SendByte(BMP085_SlaveAddress+1);         //发送设备地址+读信号

    msb = BMP085_RecvByte();                 //BUF[0]存储
    BMP085_SendACK(0);                       //回应ACK
    lsb = BMP085_RecvByte();     
	BMP085_SendACK(1);                       //最后一个数据需要回NOACK

    BMP085_Stop();                           //停止信号
    Delay5ms();
    _data = msb << 8;
	_data |= lsb;	
	return _data;
}
//********************************************************************
long bmp085ReadTemp(void)
{

    BMP085_Start();                  //起始信号
    BMP085_SendByte(BMP085_SlaveAddress);   //发送设备地址+写信号
    BMP085_SendByte(0xF4);	          // write register address
    BMP085_SendByte(0x2E);       	// write register data for temp
    BMP085_Stop();                   //发送停止信号
	//delay(10);	// max time is 4.5ms
	Delay5ms();
	return (long) Multiple_read(0xF6);
}
//*************************************************************
long bmp085ReadPressure(void)
{
	long pressure = 0;

    BMP085_Start();                   //起始信号
    BMP085_SendByte(BMP085_SlaveAddress);   //发送设备地址+写信号
    BMP085_SendByte(0xF4);	          // write register address
    BMP085_SendByte(0x34);       	  // write register data for pressure
    BMP085_Stop();                    //发送停止信号
	//delay(10);    	                  // max time is 4.5ms
	Delay5ms();
	pressure = Multiple_read(0xF6);
	pressure &= 0x0000FFFF;
	
	return pressure;	
	//return (long) bmp085ReadShort(0xF6);
}

//**************************************************************

//初始化BMP085，根据需要请参考pdf进行修改**************
void Init_BMP085()
{
	ac1 = Multiple_read(0xAA);
	ac2 = Multiple_read(0xAC);
	ac3 = Multiple_read(0xAE);
	ac4 = Multiple_read(0xB0);
	ac5 = Multiple_read(0xB2);
	ac6 = Multiple_read(0xB4);
	b1 =  Multiple_read(0xB6);
	b2 =  Multiple_read(0xB8);
	mb =  Multiple_read(0xBA);
	mc =  Multiple_read(0xBC);
	md =  Multiple_read(0xBE);
}
//***********************************************************************
uchar bmp085Convert()
{

	uchar k;

	long ut;
	long up;
	long x1, x2, b5, b6, x3, b3, p;
	unsigned long b4, b7;
	long  temperature;
	long  pressure;




 	Init_BMP085();

	
	
	ut = bmp085ReadTemp();	   // 读取温度
//	ut = bmp085ReadTemp();	   // 读取温度
 	
	up = bmp085ReadPressure();  // 读取压强
//	up = bmp085ReadPressure();  // 读取压强


	if( BMP085_LINK==0)	  //没有检测到BMP180
	{
	
	
	
	 return 0;
	}	 

	x1 = ((long)ut - ac6) * ac5 >> 15;
	x2 = ((long) mc << 11) / (x1 + md);
	b5 = x1 + x2;
	temperature = (b5 + 8) >> 4;
	
	//*************
	
//	conversion(temperature);
	
	
	
//	bmp085_tp_bai=bmp085_bai;								    
//	bmp085_tp_shi=bmp085_shi;
//	bmp085_tp_ge=bmp085_ge;
	
	
//	HUASHI_BMP085=((bmp085_tp_bai-0x30)*10+(bmp085_tp_shi-0x30))*9/5+32;   	//转成华氏温度	
//	HUASHI_BMP085_BAI=HUASHI_BMP085/100%10+0x30;   		  
//	HUASHI_BMP085_SHI=HUASHI_BMP085/10%10+0x30; 
//	HUASHI_BMP085_GE=HUASHI_BMP085%10+0x30; 

/*	*/ 

//	
//	SendData('T');		  //温度显示
//	SendData(':');
//	
//	SendData(bmp085_tp_bai);
//	SendData(bmp085_tp_shi);
//	SendData('.');
//	SendData(bmp085_tp_ge);
//	
//	SendData('C');		//温度单位
//	
//	SendData(0x0d);
//	SendData(0x0a);

 

	 
     //*************
	
	b6 = b5 - 4000;
	x1 = (b2 * (b6 * b6 >> 12)) >> 11;
	x2 = ac2 * b6 >> 11;
	x3 = x1 + x2;
	b3 = (((long)ac1 * 4 + x3) + 2)/4;
	x1 = ac3 * b6 >> 13;
	x2 = (b1 * (b6 * b6 >> 12)) >> 16;
	x3 = ((x1 + x2) + 2) >> 2;
	b4 = (ac4 * (unsigned long) (x3 + 32768)) >> 15;
	b7 = ((unsigned long) up - b3) * (50000 >> OSS);
	if( b7 < 0x80000000)
	     p = (b7 * 2) / b4 ;
           else  
		    p = (b7 / b4) * 2;
	x1 = (p >> 8) * (p >> 8);
	x1 = (x1 * 3038) >> 16;
	x2 = (-7357 * p) >> 16;
	pressure = p + ((x1 + x2 + 3791) >> 4);
	
   BMPXX_QY=(uint) (pressure/10);

  tostring(BMPXX_QY);
//	conversion(pressure);

//	QY_SHIWAN=bmp085_shiwan;
//	QY_WAN= bmp085_wan;
//	QY_QIAN=bmp085_qian;
//	QY_BAI=bmp085_bai;
//	QY_SHI=bmp085_shi;
	
	k=0;
	QY[k]=wan;	k++;
	QY[k]=qian;	k++;
	QY[k]=bai;	k++;
	QY[k]=shi;	k++;
	QY[k]='.';	k++;
	QY[k]=ge;	k++;
//	QY[k]='p';	k++;
//	QY[k]='a';	k++;
	QY[k]=0x00;


//	
//	SendData('P');		  //显示压强
//	SendData(':');
//	
//	SendData(bmp085_shiwan);
//	SendData(bmp085_wan);
//	SendData(bmp085_qian);
//	SendData(bmp085_bai);
//	SendData(bmp085_shi);
//	SendData(bmp085_ge);
//	SendData(0x0d);
//	SendData(0x0a);


	 
//标准气象信标格式，全数据
//217/000g003t058r010P009p040h62b10236	  36字节长
//      0-2 3 4-6  	7-10  	11-14  15-18	  19-22		  23-26     27-29  		30-35
//		217	/ 000	g003 	t058   r010       P009        p040      h62  		b10236
//项目：风向，风速，阵风，	温度，雨/上小时，雨/上24小时,雨/午夜起	,湿度		,气压
//长度：3  ,1 ,  3  , 4   , 	4   , 4        ,  4         , 4       	, 3  		,  6
//单位：度  ，英制,5分钟，	华氏，小时     ，24小时     ，零点起	，百分比	，帕*10


//unsigned char xdata weather_is_beacen[]={"217/000g003t058r010P009p040h62b10236 WX_beacen \r\n"};
	return 1;
}


