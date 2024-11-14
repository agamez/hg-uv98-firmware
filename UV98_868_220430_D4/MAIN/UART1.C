#include "STC8A8K64D4.H"
#include "UART1.H"
#include "UARTx.H"
#include "UART2.H"

#include "tostring.H"


#define BAUD    9600			   //串口/蓝牙通讯速率9600



#define S1_S0 0x40              //P_SW1.6
#define S1_S1 0x80              //P_SW1.7


//#define ISP_ON      IAP_CONTR= 0x60;		  //ISP下载
//#define RESET       IAP_CONTR= 0x20;		  //系统复位

unsigned char UART1_BUF_DATA[600];	//接收数据缓存
unsigned int  UART1_BUF_LENTH;	//串口收发数据长度

bit UART1_TX_BUSY;		//串口3发送空闲

unsigned char UART1_RX_TIME;	//UARTx  接收数据计时

/*----------------------------
发送串口数据
----------------------------*/
void UART1_SendData(uchar dat)
{
    while (UART1_TX_BUSY);    //等待前面的数据发送完成

    UART1_TX_BUSY = 1;
    SBUF = dat;                //写数据到UART2数据寄存器
}

/*----------------------------
发送字符串
----------------------------*/
void UART1_SendString(char *s)
{
    while (*s)                  //检测字符串结束标志
    {
        UART1_SendData(*s++);         //发送当前字符
    }
}




void UART1_DEBUG(unsigned int n)	   //调试输出整数
{
    tostring(n);
 
    UART1_SendString(str_txt);
    UART1_SendString("\r\n");
}



//*****************************************
//UART1串行口中断
//*****************************************
void UART1() interrupt 4 using 1
{
    if(TI)
    {
        TI = 0;	   //注意TI中断引起的死机 	//串口发送完一个字节标志    //清忙标志
        UART1_TX_BUSY = 0;
        return;
    }

    if(RI)
    {
        RI = 0;



        UART1_RX_TIME = 0;		//接收计时清空
        UART1_BUF_DATA[UART1_BUF_LENTH++] = SBUF;
        UART1_BUF_DATA[UART1_BUF_LENTH] = 0x00;		//	补结束符号//接收1个字节数据

        if (UART1_BUF_LENTH > 590)
        {
            UART1_BUF_LENTH = 0;
            return;
        }

        // 数据大于128，接收数据长度，溢出错误处理			//数据长度重置=0
    }
}





void UART1_FUN()	//处理串口接收到的数据 ,解析数据
{
    uint i;

    if ((UART1_BUF_LENTH != 0) && (UART1_RX_TIME > 5))
    {

//	 for(i=0;i<UART1_BUF_LENTH;i++)   	 {   UART2_SendData(UART1_BUF_DATA[i]);    }  //调试
        for(i = 0; i < UART1_BUF_LENTH; i++)
        {
            UARTx_BUF[i] =  UART1_BUF_DATA[i] ;
            UARTx_BUF[i + 1] = 0;
        }

        UARTx_BUF_LENTH = UART1_BUF_LENTH;
        UART1_BUF_LENTH = 0;
        UART1_RX_TIME = 0;

        UART_X_CMD(1);
    }

}


//
//void UART1_select_p3031()
//{
//	ACC = P_SW1;
//    ACC &= ~(S1_S0 | S1_S1);    //S1_S0=0 S1_S1=0
//    P_SW1 = ACC;                //(P3.0/RxD, P3.1/TxD)
//}
//
//void UART1_select_p3637()
//{
//  ACC = P_SW1;
//  ACC &= ~(S1_S0 | S1_S1);    //S1_S0=1 S1_S1=0
//  ACC |= S1_S0;               //(P3.6/RxD_2, P3.7/TxD_2)
//  P_SW1 = ACC;
//}

void UART1_Initial()
{
//  ACC = P_SW1;
//  ACC &= ~(S1_S0 | S1_S1);    //S1_S0=0 S1_S1=1
//  ACC |= S1_S1;               //(P1.6/RxD_3, P1.7/TxD_3)
//  P_SW1 = ACC;

    SCON = 0x50;                //8位可变波特率
    T2L = (65536 - (FOSC / 4 / BAUD)); //设置波特率重装值
    T2H = (65536 - (FOSC / 4 / BAUD)) >> 8;
    AUXR = 0x14;                //T2为1T模式, 并启动定时器2
    AUXR |= 0x01;               //选择定时器2为串口1的波特率发生器

    UART1_TX_BUSY = 0;
    UART1_BUF_LENTH = 0;	//数据长度重置=0	  //清除串口2缓冲数据

    ES = 1;                     //使能串口1中断
}




