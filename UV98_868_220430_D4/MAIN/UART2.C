#include "STC8A8K64D4.H"
#include "PUBLIC_BUF.H"


#include "UART1.H"
#include "UARTx.H"
#include "UART2.H"

#include "tostring.H"


#define S2RI  0x01              //S2CON.0
#define S2TI  0x02              //S2CON.1
//#define S2RB8 0x04              //S2CON.2
//#define S2TB8 0x08              //S2CON.3
//#define S2_S0 0x01              //P_SW2.0

unsigned char   UART2_BUF_DATA[600] ;		  //串口缓冲尺寸
unsigned int 	UART2_BUF_LENTH;	//串口收发数据长度

//bit UART2_RX_BUSY;	    //串口2接收数据等待处理
bit UART2_TX_BUSY;		//串口2发送空闲


unsigned char UART2_RX_TIME;	//UARTx  接收数据计时

/*----------------------------
发送串口数据
----------------------------*/
void UART2_SendData(uchar dat)
{
    while (UART2_TX_BUSY);               //等待前面的数据发送完成

    UART2_TX_BUSY = 1;
    S2BUF = dat; //组合登录数据;                //写数据到UART2数据寄存器
}

/*----------------------------
发送字符串
----------------------------*/
void UART2_SendString(char *s)
{
    while (*s)                  //检测字符串结束标志
    {
        UART2_SendData(*s++);         //发送当前字符
    }
}

/*----------------------------
UART2 中断服务程序
-----------------------------*/
void UART2() interrupt 8 using 1
{
    if (S2CON & S2TI)
    {
        S2CON &= ~S2TI;    //清除S2TI位  		          //清忙标志
        UART2_TX_BUSY = 0;
        return;
    }

    if (S2CON & S2RI)
    {
        S2CON &= ~S2RI;         //清除S2RI位

//        if (UART2_RX_BUSY == 1)
//        {
//            return;   //如果接收的数据没处理，则忽略新的数据，数据长度重置0
//        }

        UART2_RX_TIME = 0;		//接收计时清空
        UART2_BUF_DATA[UART2_BUF_LENTH++] = S2BUF;
        UART2_BUF_DATA[UART2_BUF_LENTH] = 0x00;	 //接收1个字节数据		//	补结束符号

        if (UART2_BUF_LENTH > 550)
        {
            UART2_BUF_LENTH = 0;
            return;
        }

        // 数据大于128，接收数据长度，溢出错误处理			//数据长度重置=0
    }
}






void UART2_DEBUG(unsigned int n)	   //调试输出整数
{
    tostring(n);

    UART2_SendString(str_txt);
    UART2_SendString("\r\n");

}



void UART2_FUN() ////如果电台信道空闲，则处理串口接收到KISS数据
{
    uint i;

    if ((UART2_BUF_LENTH != 0) && (UART2_RX_TIME > 10))
    {

        //for(i=0;i<UART2_BUF_LENTH;i++)   	 {   UART2_SendData(UART2_BUF_DATA[i]);    }  //调试
        for(i = 0; i < UART2_BUF_LENTH; i++)
        {
            UARTx_BUF[i] =  UART2_BUF_DATA[i] ;
            UARTx_BUF[i + 1] = 0;
        }

        UARTx_BUF_LENTH = UART2_BUF_LENTH;

        UART2_BUF_LENTH = 0;
        UART2_RX_TIME = 0;

        UART_X_CMD(2);
    }
}




//
//void UART2_select_p1011()	  //蓝牙
//{	P_SW2 &= ~S2_S0;     //S2_S0=0 (P1.0/RxD2, P1.1/TxD2)
//}
//
//void UART2_select_p4647()	  //LCD
//{	P_SW2 |= S2_S0;      //S2_S0=1 (P4.6/RxD2_2, P4.7/TxD2_2)
//}


void UART2_Initial()
{
//	P_SW2 &= ~S2_S0;            //S2_S0=0 (P1.0/RxD2, P1.1/TxD2)
//	//  P_SW2 |= S2_S0;             //S2_S0=1 (P4.6/RxD2_2, P4.7/TxD2_2)

//	UART2_select_p4647();

    S2CON = 0x50;               //8位可变波特率
//	波特率不用设置，和串口1，相同
//	T2L = (65536 - (FOSC/4/BAUD));   //设置波特率重装值
//	T2H = (65536 - (FOSC/4/BAUD))>>8;
//	AUXR = 0x14;                //T2为1T模式, 并启动定时器2

    UART2_TX_BUSY = 0;
    UART2_BUF_LENTH = 0;	//数据长度重置=0	  //清除串口2缓冲数据

    IE2 |= 0x01;                 //使能串口2中断
//	EA = 1;

}
