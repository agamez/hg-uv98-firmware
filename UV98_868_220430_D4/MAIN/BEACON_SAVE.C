#include "STC8A8K64D4.H"
#include "PUBLIC_BUF.H"



#include "UART2.H"

#include "DELAY.H"



#include "KISS_Analysis.H"
#include "BEACON_SAVE.H"
#include "GPS2.H"
//#include "BEACON.H"

#include "AT24C512.h"
#include "tostring.H"



#include  <string.H> //Keil library 




#include  <INTRINS.H> //Keil library 



#define nop() _nop_()



#define MAX_LEN 100

void WRITE_BECAON(uint idx)
{
    uint i, k, len;
    uint add;
    uchar temp[20];

    //如果没有初始化，则初始地址为0
//	if (AT24CXX_READ(0x8000)>99) {AT24CXX_WRITE(0x8000,0);  }
//	idx=AT24CXX_READ(0x8000);

    //写入记录
    for (i = 0; i < 256; i++)
    {
        AT24C512_RW_BUF[i] = 0x00;     //	全部清0
    }

    //写入前128个字节数据，包含日期、时间、距离、相对正北方向、相对车头方向

    if (GPS_LOCKED == 1)
    {
        AT24C512_RW_BUF[0x19] = 'A';

        for (i = 0; i < 8; i++)
        {
            AT24C512_RW_BUF[i] = GPS_TIME[i];   //写入时间,最长8字节
        }

        for (i = 8; i < 16; i++)
        {
            AT24C512_RW_BUF[i] = GPS_DATE[i - 6];   //写入日期,最长8字节
        }
    }
    else
    {
        AT24C512_RW_BUF[0x19] = 'V';

        for (i = 0; i < 16; i++)
        {
            AT24C512_RW_BUF[i] = '-';   //写入日期
        }

        AT24C512_RW_BUF[0x02] = ':';
        AT24C512_RW_BUF[0x05] = ':';
        AT24C512_RW_BUF[0x0a] = ':';
        AT24C512_RW_BUF[0x0d] = ':';
    }

    for (i = 0; i < 9; i++)
    {
        AT24C512_RW_BUF[0x10 + i] = UI_CALL[i];   //写入呼号,最长9字节
    }

    for (i = 0; i < 8; i++)
    {
        AT24C512_RW_BUF[0x20 + i] = UI_WD[i];   //写入纬度,最长8字节
    }

    AT24C512_RW_BUF[0x27] = UI_WD_DIR;

    for (i = 0; i < 6; i++)
    {
        AT24C512_RW_BUF[0x29 + i] = UI_JULI[i];   //写入距离,最长6字节
    }

    for (i = 0; i < 9; i++)
    {
        AT24C512_RW_BUF[0x30 + i] = UI_JD[i];   //写入经度,最长9字节
    }

    AT24C512_RW_BUF[0x38] = UI_JD_DIR;

    //==============================================
    for (i = 0; i < 6; i++)
    {
        AT24C512_RW_BUF[0x40 + i] = UI_ALT[i];   //写入海拔,最长6字节
    }

    for (i = 0; i < 6; i++)
    {
        AT24C512_RW_BUF[0x47 + i] = UI_SPEED[i];   //写入速度,最长6字节
    }

    //==============================================

    //--------------------------------------------------------
    for (i = 58; i < 61; i++)
    {
        AT24C512_RW_BUF[i] = UI_DIR[i - 58];   //写入航向,最长3字节
    }



    AT24C512_RW_BUF[0X3F] =	 UI_ICON  ; // 图标
    //--------------------------------------------------------
    k = 0;
    tostring(UI_Angle_N);	   //相对正北方位,最长3字节
    temp[k++] = bai;
    temp[k++] = shi;
    temp[k++] = ge;
    temp[k++] = 0x00;

    for (i = 0; i < 3; i++)
    {
        AT24C512_RW_BUF[0x1a + i] = temp[i];
    }

    //--------------------------------------------------------
    k = 0;
    tostring(UI_Angle_CAR);		 //相对车头方位,最长3字节
    temp[k++] = bai;
    temp[k++] = shi;
    temp[k++] = ge;
    temp[k++] = 0x00;

    for (i = 0; i < 3; i++)
    {
        AT24C512_RW_BUF[0x1d + i] = temp[i];
    }

    if (KISS_LEN > 127)
    {
        len = 127;
    }
    else
    {
        len = KISS_LEN;
    }

    for (i = 0; i < len; i++)
    {
        AT24C512_RW_BUF[i + 128] = (KISS_DATA[i]);   //写入原始KISS数据，必定长度小于128
    }

    //  0x0000-0x6400  （0-25600）,每条信标占用256字节，一共100条记录，索引0-99存储在 0x8000
    add = idx * 256;	 //	UART1_SendString("ADD:  ");	UART1_DEBUG(idx);
    AT24CXX_WRITE_N(add, AT24C512_RW_BUF, 128);	 	//写入前128字节存储器,分2次写，每次128字节
    AT24CXX_WRITE_N(add + 128, AT24C512_RW_BUF + 128, 128); //写入后128字节存储器
//	UART1_SendString("DATA WRITE LEN:  ");	DEBUG_KISS(AT24C512_RW_BUF,256);
//	for (i = 0; i<256; i++)   {  AT24C512_RW_BUF[i]=AT24CXX_READ(add+i);  }
//	UART1_SendString("DATA READ LEN:  ");	DEBUG_KISS(AT24C512_RW_BUF,256);
}

void IDX_UPDATA_A(uint idx)	   //索引全部向后移动,首位填写最新信标地址
{
    uint	i;
    AT24CXX_READ_N(0x6500, AT24C512_RW_BUF, 128);	//指定地址读出128个字节

    for (i = 120; i > 0; i--)
    {
        AT24C512_RW_BUF[i] = AT24C512_RW_BUF[i - 1];  //索引全部向后移动
    }

    AT24C512_RW_BUF[0] = idx;
    AT24CXX_WRITE_N(0x6500, AT24C512_RW_BUF, 128);	 //指定地址写入128个字节

//	UART1_SendString("DATA IDX A:  ");	 		  DEBUG_KISS(AT24C512_RW_BUF,100);		//调试
}

uchar IDX_UPDATA_B(uint idx)	   //索引全部向后移动,首位填写最新信标地址
{
    uint i;
    uchar temp;
    AT24CXX_READ_N(0x6500, AT24C512_RW_BUF, 128);	//指定地址读出128个字节

    if (AT24C512_RW_BUF[0] == idx)	 //如果首位对比一直，则不需要向后移动
    {
        AT24C512_RW_BUF[0] = idx;
        AT24CXX_WRITE_N(0x6500, AT24C512_RW_BUF, 128);	 //指定地址写入128个字节

//			UART1_SendString("DATA IDX B:  ");	  DEBUG_KISS(AT24C512_RW_BUF,100);		//调试
    }
    else
    {
//		AT24CXX_READ_N(0x6500,AT24C512_RW_BUF,128);	//指定地址读出128个字节

        for (i = 0; i < MAX_LEN; i++) 											 //------100
        {
            if (AT24C512_RW_BUF[i] == idx)
            {
                temp = i;
                break;
            }

//			if (i==MAX_LEN-1){	UART2_SendString("idx out:  \r\n");	 return 2;}	  //没有检索到索引编号，发生错误		  //------ 99
            if (i == MAX_LEN - 1)
            {
                return 2;   //没有检索到索引编号，发生错误		  //------ 99
            }

        }


        for (i = temp; i > 0; i--)
        {
            AT24C512_RW_BUF[i] = AT24C512_RW_BUF[i - 1];  //索引全部向后移动一行
        }

        AT24C512_RW_BUF[0] = idx;
        AT24CXX_WRITE_N(0x6500, AT24C512_RW_BUF, 128);	 //指定地址写入128个字节

//	 	UART1_SendString("DATA IDX C:  ");	  DEBUG_KISS(AT24C512_RW_BUF,100);		//调试
    }

    return 1;
}



uchar BEACON_SAVE()    //写入信标列表和存储信标
{
    uint i  ;
    uint idx, add;
    uchar temp[20];
    uchar *p;


    for (idx = 0; idx < MAX_LEN; idx++)
    {
        add = idx * 256;		 //		UART1_DEBUG(add);

        for (i = 0; i < 9; i++)
        {
            temp[i] = AT24CXX_READ(add + 16 + i);    //呼号
            temp[i + 1] = 0;
        }

//	UART1_SendString("CALL :  ");	 DEBUG_KISS(temp,9);
        if (temp[0] == 0xff)
        {
            IDX_UPDATA_A(idx);     //空行，则直接填入信标 ,索引填入第一行
            WRITE_BECAON(idx);
            return 0;
        }


        //	//读出呼号
        //	for (i = 0; i<9; i++) { temp[i]=AT24CXX_READ(add+16+i); temp[i+1]=0; } 	 //呼号

        p = strstr(UI_CALL, temp);	//UI_CALL 对比呼号

        if  (p != NULL)			 	//对比成功，替换信标段
        {
            if( IDX_UPDATA_B(idx) == 1)
            {
                WRITE_BECAON(idx);
            }

            return 1;
            //如果更新索引成功，则替换信标段
        }

        //对比不成功，继续对比

        if (idx == MAX_LEN - 1) 	 	 //全部对比完，没对比上，删除最后一条记录
        {
            idx = AT24CXX_READ(0x6500 + MAX_LEN - 1);
            IDX_UPDATA_A(idx);
            WRITE_BECAON(idx);
            return 2;
        }
    }

    return 3;
}




//	0xFE00  最后512 存储设置数据