#include "STC8A8K64D4.H"
#include "STC_EEPROM.H"

#include "KISS2GAOMIN.H"
#include "KISS2ASC.H"
//#include "UART1.H"
#include "UART2.H"
//#include "UART4.H"

#include "BEACON.H"
#include "BT.H"
//#include "MICE_DECODE.H"

#include "tostring.H"

#include "KISS_Analysis.H"
//#include "tostring.H"
#include "DELAY.H"

#include "PUBLIC_BUF.H"


#include <string.h>



//蓝牙输出0=航点、1=KISS、2=UI数据 3=GPS 4=GPS+UI	  5=OFF




void Disp_Analytic_data()		 //蓝牙/ISP口显示附加信息
{
//	uchar SN_RX_BUFFER[200];
    uint  k ;
    uchar temp[20];

//	return;

    if (EEPROM_Buffer[0x00A5] != 1)
    {
        return;
    }


    SN_RX_BUFFER[0] = 0;
    //==============================================
    strcat(SN_RX_BUFFER, "{alt: ");

    if (UI_ALT[0] == 0)
    {
        strcat(SN_RX_BUFFER, "------");
    }
    else
    {
        strcat(SN_RX_BUFFER, UI_ALT);
    }

    //写入海拔,最长6字节
    strcat(SN_RX_BUFFER, " || speed: ");

    if (UI_SPEED[0] == 0)
    {
        strcat(SN_RX_BUFFER, "---.-");
    }
    else
    {
        strcat(SN_RX_BUFFER, UI_SPEED);
    }

    //写入速度,最长6字节

    //==============================================
    strcat(SN_RX_BUFFER, " || course: ");

    if (UI_DIR[0] == 0)
    {
        strcat(SN_RX_BUFFER, "---");
    }
    else
    {
        strcat(SN_RX_BUFFER, UI_DIR);
    }

    //写入航向,最长3字节
    //--------------------------------------------------------
    strcat(SN_RX_BUFFER, " || Dir(north): ");
    k = 0;
    tostring(UI_Angle_N);	   //相对正北方位,最长3字节
    temp[k++] = bai;
    temp[k++] = shi;
    temp[k++] = ge;
    temp[k++] = 0x00;

    strcat(SN_RX_BUFFER, temp);
    //--------------------------------------------------------
    strcat(SN_RX_BUFFER, " || Dir(Relative): ");
    k = 0;
    tostring(UI_Angle_CAR);		 //相对车头方位,最长3字节
    temp[k++] = bai;
    temp[k++] = shi;
    temp[k++] = ge;
    temp[k++] = 0x00;

    strcat(SN_RX_BUFFER, temp);

    //--------------------------------------------------------
    strcat(SN_RX_BUFFER, " || distance: ");		  //距离





    switch (UI_JULI[0])
    {
        case 0:
            strcat(SN_RX_BUFFER, "----.-");
            strcat(SN_RX_BUFFER, " Km ");
            break;

        case '*':	  //小于10,000米
            strcat(SN_RX_BUFFER, UI_JULI + 1);
            strcat(SN_RX_BUFFER, " Km ");
            break;

        default:
            strcat(SN_RX_BUFFER, UI_JULI);
            strcat(SN_RX_BUFFER, " Km ");
            break;
    }




//	k=0;
//	tostring((uint)UI_JULI);
//	temp[k++]=bai;  		temp[k++]=shi;   	temp[k++]=ge;  	temp[k++]=0x00;
//	strcat(SN_RX_BUFFER,temp);
//	strcat(SN_RX_BUFFER," Km ");

    //--------------------------------------------------------

    strcat(SN_RX_BUFFER, " || elevation: ");	   //相对仰角

    k = 0;
    tostring(Elevation_angle);
    temp[k++] = bai;
    temp[k++] = shi;
    temp[k++] = ge;
    temp[k++] = 0x00;
    strcat(SN_RX_BUFFER, temp);

    strcat(SN_RX_BUFFER, " }\r\n");
    //--------------------------------------------------------
    UART2_SendString(SN_RX_BUFFER);

    //--------------------------------------------------------
    SN_RX_BUFFER[0] = 0;

    strcat(SN_RX_BUFFER, "{GS-232B/G5500 CMD: ");
    strcat(SN_RX_BUFFER, "W");

    k = 0;
    tostring(UI_Angle_N);	   //相对正北方位,最长3字节
    temp[k++] = bai;
    temp[k++] = shi;
    temp[k++] = ge;
    temp[k++] = 0x00;

    strcat(SN_RX_BUFFER, temp);
    //--------------------------------------------------------
    strcat(SN_RX_BUFFER, " ");
    //--------------------------------------------------------
    k = 0;						 //仰角
    tostring(Elevation_angle);
    temp[k++] = bai;
    temp[k++] = shi;
    temp[k++] = ge;
    temp[k++] = 0x00;
    strcat(SN_RX_BUFFER, temp);

    strcat(SN_RX_BUFFER, " }\r\n");

    UART2_SendString(SN_RX_BUFFER);
}


void G5500_OUT( )	//蓝牙输出数据	0=输出自己发出的信标  1=输出解码的信标
{
    uchar *p;
    uint  k ;
    uchar temp[20];

//	uchar GS232B[20];
//	if (EEPROM_Buffer[0x0128]==0) {return;}
//-----------------------------------------------------
    if(EEPROM_Buffer[0X15] != 2)
    {
        return;    //是否启用G5500控制
    }

    READ_TEMP_CALL(0x0120, 0x0127);	 //读取跟踪目标呼号-SSID

    p = strstr(UI_CALL, TEMP_Call);	//与接收到的呼号对比

    if  (p != NULL) 						//对比成功
    {
        SN_RX_BUFFER[0] = 0;

        strcat(SN_RX_BUFFER, "W");

        k = 0;
        tostring(UI_Angle_N);	   //相对正北方位,最长3字节
        temp[k++] = bai;
        temp[k++] = shi;
        temp[k++] = ge;
        temp[k++] = 0x00;

        strcat(SN_RX_BUFFER, temp);
        //--------------------------------------------------------
        strcat(SN_RX_BUFFER, " ");
        //--------------------------------------------------------
        k = 0;						 //仰角
        tostring(Elevation_angle);
        temp[k++] = bai;
        temp[k++] = shi;
        temp[k++] = ge;
        temp[k++] = 0x00;
        strcat(SN_RX_BUFFER, temp);

        strcat(SN_RX_BUFFER, "\r\n");

        UART2_SendString(SN_RX_BUFFER);
    }

}




void SETUP_BL_NAME()
{
    Delay_time_25ms(20);

    READ_TEMP_CALL(0X0008, 0X000F);
    UART2_SendString("AT+NAME");	 //初始化设置蓝牙名称
    UART2_SendString(TEMP_Call);	 //初始化设置蓝牙名称
// 	UART2_SendString("\r\n");	 //不用加回车
    Delay_time_25ms(20); //必须延时，等待蓝牙设置完成，下次通电有效
}




void BT_OUT(unsigned char STU)	//STU=0  未知格式，1=接收到有效信标	 2=自己发的信标
{
    uint i;

    if (EEPROM_Buffer[0x0016] == 0)
    {
        return;
    }

    if (EEPROM_Buffer[0x0016] == 3) 	//航点格式
    {
        if (STU == 0)
        {
            return;   //未知格式数据
        }

        if (STU == 2)
        {
            //自己发的信标输出GPWPL格式

            MY_BEACON_TO_GPWPL();	//自己的信标直接转成GPWPL航点格式

            UART2_SendString(GPWPL_BUF );  //串口2蓝牙输出航点数据
//			UART1_SendString(GPWPL_BUF );  //串口2蓝牙输出航点数据
            return;						   //自己发的信标，不需要显示分析数据
        }

        if (STU == 1)
        {
            UI_TO_GPWPL();			       //接收到的信标，解析成航点
            UART2_SendString(GPWPL_BUF );  //串口2蓝牙输出航点数据
//			UART1_SendString(GPWPL_BUF );  //串口2蓝牙输出航点数据

            Disp_Analytic_data();		   //显示分析接收到的数据

            return;
        }
    }

    if (EEPROM_Buffer[0x0016] == 1) 		 	//KISS HEX
    {
        UART2_SendData(0xC0);
        UART2_SendData(0x00); 		//蓝牙串口输出KISS数据

        for (i = 0; i < KISS_LEN; i++)
        {
            UART2_SendData(KISS_DATA[i]);
        }

        UART2_SendData(0xC0);

//		UART1_SendData(0xC0);	UART1_SendData(0x00); 		//蓝牙串口输出KISS数据
//		for (i=0;i<KISS_LEN;i++)    {  UART1_SendData(KISS_DATA[i]);}
//		UART1_SendData(0xC0);

        return;
    }

    if (EEPROM_Buffer[0x0016] == 2) 		 	 //2=UI
    {
        KISS_TO_ASCII(SN_RX_BUFFER, 0);	 	 //电台接收到的 KISS数据,RF解码后,转换ASCII UI格式,并取得UI数据长度	UI_DIGI_LEN
        UART2_SendString(SN_RX_BUFFER);	     //串口1监控
//		UART1_SendString(SN_RX_BUFFER);	     //串口1监控

        if (STU == 0)
        {
            return;   //未知格式数据
        }

        if (STU == 2)
        {
            return;   //自己发的信标，不需要显示分析数据
        }

        if (STU == 1)
        {
            Disp_Analytic_data();    //显示分析接收到的数据
        }

        return;
    }

    if (EEPROM_Buffer[0x0016] == 4) 	 		//4=KISS ASC
    {
        DEBUG_KISS(KISS_DATA, KISS_LEN);

        if (STU == 0)
        {
            return;   //未知格式数据
        }

        if (STU == 2)
        {
            return;   //自己发的信标，不需要显示分析数据
        }

        if (STU == 1)
        {
            Disp_Analytic_data();   //显示分析接收到的数据
        }

        return;
    }
}

