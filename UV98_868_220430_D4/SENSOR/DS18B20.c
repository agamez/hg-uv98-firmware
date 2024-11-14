
#include "STC15Fxxxx.H"
#include "SENSOR\DS18B20.h"
#include "tostring.H"
#include "MAIN\UART1.H"



#include  <INTRINS.H> //Keil library 

#define   uchar unsigned char	   //0-255单字节	用uchar 代替char
#define   uint unsigned int		   //0-65536双字节	用uint 代替int
#include "HEX_TO_ASC.H"

long DS18B20_WENDU;
uchar  DS18B20_TEMP[10];

sbit DQ = P2^1;                     //DS18B20的数据口位P3.3
uint TPH;                           //存放温度值的高字节
uint TPL;                           //存放温度值的低字节

void DelayXus(uchar n);
uchar DS18B20_Reset();
void DS18B20_WriteByte(uchar dat);
uchar DS18B20_ReadByte();


/*************************************************** 
	DS18B20读温度程序
****************************************************/ 
uchar DS18B20_READTEMP() 
{ 
    uchar i,n;
	uint temp;
	bit   ZERO; //0=零上，1=零下

	if (DS18B20_Reset()==0)	 {return 0;	 }
	
	//DS18B20_Reset();                //设备复位
    DS18B20_WriteByte(0xCC);        //跳过ROM命令
    DS18B20_WriteByte(0x44);        //开始转换命令
    while (!DQ);                    //等待转换完成

	if (DS18B20_Reset()==0)	{return 0; 	 }
	
    //DS18B20_Reset();                //设备复位
    DS18B20_WriteByte(0xCC);        //跳过ROM命令
    DS18B20_WriteByte(0xBE);        //读暂存存储器命令
    TPL = DS18B20_ReadByte();       //读温度低字节
    TPH = DS18B20_ReadByte();       //读温度高字节

// UART1_SendData(0xaa);	UART1_SendData(TPH);		UART1_SendData(TPL);

	for(i=0;i<6;i++)   {DS18B20_TEMP[i]=0x00;}

	if ((TPH&0x80)==0x80)	  //摄氏度<0	
	{
		ZERO=1;
		DS18B20_WENDU=-(~((TPH*256)+TPL)+1)*0.0625*10; 	 //保留1位小数
		//-0.1 ~ -54.5
		DS18B20_TEMP[0]='-';
	  	}
	else
	{					  //摄氏度>=0 
		ZERO=0;
		DS18B20_WENDU=((TPH*256)+TPL)*0.0625*10; 	   //保留1位小数
		//0~124.5
		DS18B20_TEMP[0]=' ';
	}
 	if (ZERO==1) { temp=(uint)-DS18B20_WENDU; }else{temp=(uint)DS18B20_WENDU;}	 
	



	tostring(temp);

	DS18B20_TEMP[1]=wan;
	DS18B20_TEMP[2]=qian;
	DS18B20_TEMP[3]=bai;
	DS18B20_TEMP[4]=shi;
	DS18B20_TEMP[5]='.';
	DS18B20_TEMP[6]=ge;
	DS18B20_TEMP[7]=0x00;

	n=0;
	while(DS18B20_TEMP[1]=='0')	 //首位0消隐 ,最少保留一位
	{
	for(i=1;i<8;i++) 	{ DS18B20_TEMP[i]=DS18B20_TEMP[i+1]; }
	n++; if (n>2){break;}
	}

UART1_SendString("DS18B20: ");	UART1_SendString(DS18B20_TEMP);	UART1_SendString("\r\n");

	return 1;
}

/**************************************
延时X微秒(STC12C5A60S2@12M)
不同的工作环境,需要调整此函数
此延时函数是使用1T的指令周期进行计算,与传统的12T的MCU不同
**************************************/
void DelayXus(uchar n)		  //22.1184M
{	 	
	while (n--)
	{
	_nop_();_nop_();_nop_();_nop_();    _nop_();_nop_();_nop_();_nop_();  	_nop_();_nop_();
	_nop_();_nop_();_nop_();_nop_();    _nop_();_nop_();_nop_();_nop_();  	_nop_();_nop_();
	_nop_();       _nop_();  

//
//  _nop_();_nop_();_nop_();_nop_();    _nop_();_nop_();_nop_();_nop_();  	_nop_();_nop_();
//	_nop_();_nop_();_nop_();_nop_();    _nop_();_nop_();_nop_();_nop_();  	_nop_();_nop_();
//	_nop_();       _nop_();  
//


	}
}






/**************************************
复位DS18B20,并检测设备是否存在

**************************************/
uchar DS18B20_Reset()
{
	CY = 1;
    DQ = 0;                     //送出低电平复位信号
    DelayXus(240);              //延时至少480us
    DelayXus(240);              //延时至少480us

    DQ = 1;                     //释放数据线
    DelayXus(60);               //等待60us
    CY = DQ;                    //检测存在脉冲
    DelayXus(240);              //等待设备释放数据线
	DelayXus(180);
	if (CY==0) 		{return 1;}	   //检测存在脉冲
	return 0;
}

/**************************************
从DS18B20读1字节数据
**************************************/
uchar DS18B20_ReadByte()
{
    uchar i;
    uchar dat = 0;

    for (i=0; i<8; i++)             //8位计数器
    {	
		dat >>= 1;
        DQ = 0;                     //开始时间片
        DelayXus(1);                //延时等待10us
        DQ = 1;                     //准备接收
        DelayXus(1);                //接收延时10us
        if (DQ) dat |= 0x80;        //读取数据
        DelayXus(60);               //等待60us时间片结束
    }

    return dat;
}

/**************************************
向DS18B20写1字节数据
**************************************/
void DS18B20_WriteByte(uchar dat)
{
    char i;

    for (i=0; i<8; i++)             //8位计数器
    {
        DQ = 0;                     //开始时间片
        DelayXus(1);                //延时等待10us
        dat >>= 1;                  //送出数据
        DQ = CY;
        DelayXus(60);               //等待60us时间片结束
        DQ = 1;                     //恢复数据线
        DelayXus(1);                //恢复延时10us
    }
}
