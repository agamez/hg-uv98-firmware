#include "STC8A8K64D4.H"
#include "UART4.H"
#include "UART2.H"

#include "UARTx.H"
//#include "UART2.H"
#include "PUBLIC_BUF.H"

#include  <string.H> //Keil library 

#define FOSC    22118400L		   //时钟22.1184M
//#define FOSC    11059200L		   //时钟22.1184M
#define BAUD    115200			   //GPRS通讯速率115200

#define S4RI  0x01              //S4CON.0
#define S4TI  0x02              //S4CON.1
#define S4RB8 0x04              //S4CON.2
#define S4TB8 0x08              //S4CON.3
#define S4_S0 0x04              //P_SW2.2


//bit UART4_RX_BUSY;	   //串口4接收数据等待处理
bit UART4_TX_BUSY;	   //串口4发送空闲


unsigned char  UART4_BUF_DATA[600] ;		  //串口缓冲尺寸
unsigned int  UART4_BUF_LENTH;

unsigned char UART4_RX_TIME;	//UARTx  接收数据计时

/*----------------------------
发送串口数据
----------------------------*/
void UART4_SendData(uchar dat)
{
    while (UART4_TX_BUSY);      //等待前面的数据发送完成

    UART4_TX_BUSY = 1;
    S4BUF = dat;                //写数据到UART4数据寄存器
}

/*----------------------------
发送字符串
----------------------------*/
void UART4_SendString(char *s)
{
    while (*s)                  //检测字符串结束标志
    {
        UART4_SendData(*s++);      //发送当前字符
    }

    TIME_1S = 0; //计时清0，防止GPS状态连续输出


}



/*----------------------------
UART4 中断服务程序
-----------------------------*/
void UART4() interrupt 18 using 1
{
    if (S4CON & S4TI)
    {
        S4CON &= ~S4TI;    	   //清忙标志    //清除S4TI位
        UART4_TX_BUSY = 0;
        return;
    }

    if (S4CON & S4RI)
    {
        S4CON &= ~S4RI;         //清除S4RI位

//        if (UART4_RX_BUSY == 1)
//        {
//            return;   //如果接收的数据没处理，则忽略新的数据，数据长度重置0
//        }

        UART4_RX_TIME = 0;		//接收计时清空
        UART4_BUF_DATA[UART4_BUF_LENTH++] = S4BUF;
        UART4_BUF_DATA[UART4_BUF_LENTH] = 0x00;			 //接收1个字节数据	//	补结束符号

        if (UART4_BUF_LENTH > 550)
        {
            UART4_BUF_LENTH = 0;
            return;
        }

        // 数据大于128，接收数据长度，溢出错误处理			//数据长度重置=0
    }
}


void UART4_FUN() ////如果电台信道空闲，则处理串口接收到KISS数据
{
    uint i;

    if ((UART4_BUF_LENTH != 0) && (UART4_RX_TIME > 10))
    {
//        UART4_RX_BUSY = 1; //停止接收


//		for(i=0;i<UART4_BUF_LENTH;i++)   	 {   UART2_SendData(UART4_BUF_DATA[i]);    }  //调试


        for(i = 0; i < UART4_BUF_LENTH; i++)
        {
            UARTx_BUF[i] =  UART4_BUF_DATA[i] ;
            UARTx_BUF[i + 1] = 0;
        }

        UARTx_BUF_LENTH = UART4_BUF_LENTH;
        UART4_BUF_LENTH = 0;
        UART4_RX_TIME = 0;
//        UART4_RX_BUSY = 0; //允许重新接收
        UART_X_CMD(4);
    }
}


void UART4_Initial()
{
//	P_SW2 &= ~S4_S0;            //S4_S0=0 (P0.2/RxD4, P0.3/TxD4)
    //  P_SW2 |= S4_S0;             //S4_S0=1 (P5.2/RxD4_2, P5.3/TxD4_2)
    S4CON = 0x50;               //8位可变波特率

    T4L = (65536 - (FOSC / 4 / 9600)); //设置波特率重装值
    T4H = (65536 - (FOSC / 4 / 9600)) >> 8;
    T4T3M |= 0x20;              //定时器4为1T模式
    T4T3M |= 0x80;              //定时器4开始计时

    UART4_TX_BUSY = 0;
//    UART4_RX_BUSY = 0;
    UART4_BUF_LENTH = 0;	//数据长度重置=0	  //清除串口2缓冲数据

    IE2 |= 0x10;                 //使能串口4中断
//	EA = 1;
}

