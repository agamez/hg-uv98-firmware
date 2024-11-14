/*------------------------------------------------------*/
/* ---  		       	     -------------------------------*/
/* --- MCU = STC8A8K64D4	22.1184M				 -------------*/
/* --- WeChat: (86)13013684000  ------------------------*/
/* ---   				----------------------------------------*/
/* --- Tel: 13013684000  15052205720--------------------*/
/* --- Web: BH4TDV.TAOBAO.com --------------------------*/
/*------------------------------------------------------*/

//未指定长度的文本型字符串，末尾编译器自动补0x00
/////////////////////////////////////////////////
//注意: n系列的芯片,上电后所有与PWM相关的IO口均为
//      高阻态,需将这些口设置为准双向口或强推挽模式方可正常使用
/////////////////////////////////////////////////

#include  <INTRINS.H> //Keil library 
#include "STC8A8K64D4.H"
#include "STC_EEPROM.H"
#include "PUBLIC_BUF.H"

#include "UART1.H"
#include "UART2.H"
#include "UART3.H"
#include "UART4.H"

#include "DELAY.H"
#include "ADC.H"

#include "SENSOR\DS18B20.h"
#include "SENSOR\bmp280.h"
#include "IO.H"
#include "GPS2.H"
#include "DISP_GPS.H"
#include "CMX865A_DECODE.H"



#include "KISS_DECODE.H"


#include "KISS2ASC.H"

#include "BEACON.H"

#include "CMX865A_SPI.H"

#include "AT24C512.h"


#include "BT.H"

/************* 定时信标相关 **************/
void Timer0_init();			   					//定时器0初始化
unsigned int  time_a;	//50MS时间单位计数
/************* 通用 **************/
void  Initial_51G2();



uchar BT_LINK;
uint BT_LINK_TIME;


sbit TXD_1	= P3 ^ 1;	 //
sbit RXD_1  = P3 ^ 0;	 //


sbit RXD3	= P0 ^ 0;	 //
sbit TXD3	= P0 ^ 1;


//#define SLEEP      PCON= 0x02;		  //掉电模式，电流0.1uA，中断触发后，由0000H开始
//#define SHUT       PCON= 0x01;		  //挂起模式，电流2MA，中断触发后，继续下一条指令

//CMX865A_TX_TONE(0x01); CMX865A_TX_TONE(0);	 	BEACON_GPS_TXD();
//
//void INT0_int (void) interrupt 0	   //中断里CPU无法掉电休眠
//{

//}
//
////
////IE.0 0 外部中断0
////IE.1 1 定时器0 溢出
////IE.2 2 外部中断1
////IE.3 3 定时器1 溢出
////IE.4 4 串口中断
////IE.5 5 定时器2 溢出
//

void INT0_int (void) interrupt 0	   //中断里CPU无法掉电休眠
{
    BT_LINK_TIME = 0;
}

void time0_init()
{

    TIME_1S = 0;

    time_a = 0;				//40*25mS=1S
    /************* 初始化定时器 **************/
    TR0 = 0;					//50ms中断1次
    TF0 = 0;

    TH0 = (65536 - 9216) / 256;	//定时5MS=5000US		  ;65536-22.1184M振荡MHZ/12*5000US	 =65536-9216
    TL0 = (65536 - 9216) % 256;


    TR0 = 1;				//启动T0


    ET0 = 1;			//允许T0中断

}

void  Initial_uv98()
{
    uchar i;
    PTT = 0;

//注意: 芯片上电后所有与PWM相关的IO口均为高阻态
//需将这些口设置为准双向口或强推挽模式方可正常使用
//相关IO:
//P0.6/P0.7
//P1.6/P1.7/
//P2.1/P2.2/P2.3/P2.7
//P3.7
//P4.2/P4.4/P4.5
//另外还有P1.0/P1.4两个口在上电时为强推挽输出,CPU批次BUG
//程序初始化时也需将这两个口设置为弱上拉准双向口模式

//P3.0 P3.1 上电=VCC/2
    P0M1 = 0x00;
    P0M0 = 0X00;	 //P0	全部设置为弱上拉准双向口模式
    P1M1 = 0x00;
    P1M0 = 0X00;	 //P1	全部设置为弱上拉准双向口模式
    P2M1 = 0x00;
    P2M0 = 0X00;	 //P2	全部设置为弱上拉准双向口模式
    P3M1 = 0x00;
    P3M0 = 0X00;	 //P3	全部设置为弱上拉准双向口模式
    P4M1 = 0x00;
    P4M0 = 0X00;	 //P4	全部设置为弱上拉准双向口模式
    P5M1 = 0x00;
    P5M0 = 0X00;	 //P5	全部设置为弱上拉准双向口模式

//设置中断优先级
//CPU复位后，各中断均为最低优先级0
//	IPH|=0X02; 	IP|=0X02;	//定时器0设为最高3
//	IPH|=0X10; 	//串口0设为优先级2

    P2M0 = 0X84;	 //P2.3 PTT推挽输出  P2.7 GPS推挽输出
    P3M0 = 0X42;	 //P3.1 3.6推挽输出txd

 

    for(i = 0; i < 2; i++)
    {
        LED_STU = 0;
//		LED_GPRS=LED_GPS=LED_TX=LED_RX=LED_TX=0;	 //点亮全部指示灯，检查LED，全亮=白色
        Delay_time_25ms(1);
        LED_STU = 1;
//		LED_GPRS=LED_GPS=LED_TX=LED_RX=LED_TX=1;	 //关LED
//		Delay_time_25ms(2);
    }

    Initial_check_erom();  //检查MAC地址	是否已存储，没有则TNC没初始化

    POWER_READ_SETUP();  	//开机读取全部参数设置到缓冲
    adc_Initial();		   //初始化ADC

    UART1_Initial();	//初始化串口1
    UART2_Initial();	//初始化串口2
    UART3_Initial();	//初始化串口3
    UART4_Initial();	//初始化串口4

    /************* 其他设置 **************/
//---------------------------------
//	DS18B20_READTEMP() ;   //开机读取几次温度，避免显示85度
//	DS18B20_READTEMP() ;

    //---------------------------------
    GPS_INIT();

    CMX865A_Init();
    CMX_RX_Initial();

    //---------------------------------
    time0_init();


    EA  = 1;			//允许全局中断
}



//========================================================================
// 备注:
//========================================================================
void timer0 (void) interrupt 1
{
    CMX_RX_INT();		//5ms中断一次,小于6.5ms都正常

    UART1_RX_TIME++;	//UARTx  接收数据计时
    UART2_RX_TIME++;	//UARTx  接收数据计时
    UART4_RX_TIME++;	//UARTx  接收数据计时


    TIME_1S++;

    time_a++;

    if (time_a > (200 - 1))	 //1秒
    {
        time_a = 0;

        BT_LINK_TIME++;

        UART3_OUTTIME++;   //接收数据计数，用于判断GPS是否断开
        GPS_WAIT_TIME++;   //智能信标时间基准
        GPS_BEACON_TIME++; //GPS定时信标计时
    }
}

void main()
{
    Initial_uv98();	//初始化TNC

    if (EEPROM_Buffer[0X3A] == 0)
    {
        BL_PWR = 0;   //初始化蓝牙
    }
    else
    {
        BL_PWR = 1;
    }

    if (EEPROM_Buffer[0X3A] == 1)
    {
        SETUP_BL_NAME();   //初始化蓝牙名称
    }

    READ_BMP280();
    UART4_TX_EEROM() ; 	//Delay_time_25ms(10);

    while (1)
    {
        UART1_FUN();    //处理串口1接收到的数据，设置，9600
        UART2_FUN();	//处理串口2的蓝牙数据，9600
        UART3_FUN();	//处理串口3的内置GPS数据，9600
        UART4_FUN(); 	//处理串口4的GPRS G3524数据,115200
        APRS_KISS_DECODE(); //RF解码KISS数据

        BECON_MODE();	//定时信标

        if (TIME_1S > 200 - 1)
        {
            TIME_1S = 0;
            DISP_GPS_STU();
        }


    }
}

//$PCAS11,0*1D	   //便携模式
//$PCAS11,1*1C	   //静态模式
//$PCAS11,2*1F		//步行模式
//$PCAS11,3*1E		//车载模式
//$PCAS11,4*19		//航海模式
//$PCAS11,5*18		//航空模式<1g
//$PCAS11,6*1B		//航空模式<2g
//$PCAS11,7*1A		//航空模式<4g
//$PCAS10,0*1C		//热启动





