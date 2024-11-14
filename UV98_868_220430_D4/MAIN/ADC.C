#include "STC8A8K64D4.H"
#include "ADC.H"
#include "io.H"
#include "DELAY.H"
//#include "UART1.H"
#include "tostring.H"



#include  <INTRINS.H> //Keil library 


/********* AD ***********/
#define ADC_ON		0x80
#define ADC_START	0x40	//自动清0
#define ADC_FLAG	0x20	//软件清0

unsigned char DY[8];
void LED_SHOW(uchar n);


void adc_Initial()	   //ADC初始化
{
    P1M1 |= 0x80;		//P 1.7  高阻用于ADC

    ADCCFG = 0x0f; //设置ADC时钟为系统时钟/2/16/16
    ADC_CONTR = ADC_ON | 7;	 //使能ADC模块 ,注意ADC	初始化影响P1.0 RXD2的问题
}




/********************* 做一次ADC转换 *******************/
uint	adc10_start(uchar channel)	//channel = 0~7
{
    ADC_RES = 0;
    ADC_RESL = 0;
    ADC_CONTR = (ADC_CONTR & 0x80) | ADC_START | channel;   //启动AD转换

    _nop_();
    _nop_();
    _nop_();
    _nop_();

    while(!(ADC_CONTR & ADC_FLAG));  //查询ADC完成标志

    ADC_CONTR &= ~ADC_FLAG;	 //清除转换完成标志
    return	(ADC_RES * 16) + (ADC_RESL >> 4);
}

void READ_ADC()			 //读P1.7	  	//读取电压值
{
    float temp, ad;
    uint vdd;
    uchar i;
    ad =	adc10_start(7);		 //读P1.7
//	vdd= ad*3.30*100*11/1024;
    temp = ad * 3.3 * 2.96 / 40.96;

//	temp= ad*3.3*100*/40.96;

    vdd = (uint)temp;
    tostring(vdd);

    if (vdd < 1000) //插入电压
    {
        i = 0;
        DY[i++] = bai;
        DY[i++] = '.';
        DY[i++] = shi;
    }
    else
    {
        i = 0;
        DY[i++] = qian;
        DY[i++] = bai;
        DY[i++] = '.';
        DY[i++] = shi;
    }

    DY[i++] = 0X00; //结束符
}

