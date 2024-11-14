#include "STC8A8K64D4.H"
#include "STC_EEPROM.H"

//#include "UART1.H"
#include "UART2.H"
#include "UART4.H"

#include "IO.H"
#include "tostring.H"

#include "BEACON.H"

#include "AT24C512.h"
#include "FUN_B.h"

#include "PUBLIC_BUF.H"
#include "GPS_JULI.H"
#include "GPS2.H"
#include "KISS_Analysis.H"

#include  <string.H> //Keil library 



unsigned char  FUN_B_BUF[50] ;		  //缓冲尺寸

void disp_fw()	   //显示正北方位
{
    uint dat;

    if (ASC_TEMP[0] == 0)
    {
        strcat(SN_RX_BUFFER, "--");
        return;
    }

    dat = (uint)(ASC_TEMP[0] - 0x30) * 100 + (uint)(ASC_TEMP[1] - 0x30) * 10 + (uint)(ASC_TEMP[2] - 0x30);

    //列表方位显示方式  0=英文  1=0-12  2-0-36
    angle_to_txt(dat, ASC_TEMP);
    strcat(SN_RX_BUFFER, ASC_TEMP);

}
uchar FUN_B_GPS(uint idx)	 	//实时显示距离和方位，列表  $B00\r\n-$B19\r\n
{
    uint i, k;
    uint  add;
    uchar temp[20];
    uchar dat;	//uchar len;
    uchar no_sig;
//	uchar *p;

    if (idx > 19)
    {
        return 0;
    }

    //==============================================  插入页面编号00-19
    SN_RX_BUFFER[0] = 0;
    strcat(SN_RX_BUFFER, "$B");
    temp[0] = idx / 10 + 0x30;
    temp[1] = idx % 10 + 0x30;
    temp[2] = 0;
    strcat(SN_RX_BUFFER, temp);
    strcat(SN_RX_BUFFER, ",");

    //==============================================
    for(i = 0; i < 5; i++)
    {
        FUN_B_BUF[i] = AT24CXX_READ(0x6500 + (idx * 5) + i);
    }

//			UART1_SendString("read a:  ");	  DEBUG_KISS(FUN_B_BUF,5);		//调试
    //起始地址0X6500,一共100字节，每1个字节一段，每页5个索引
    //==============================================
    //	   	for (i=0;i<20;i++)  		{  call[i]=0; }  	//Source呼号填充空格
    for(k = 0; k < 5; k++)
    {
        if (FUN_B_BUF[k] > 99)
        {
            strcat(SN_RX_BUFFER, "------ -- ---.- --,");	   //24C512没写入数据
        }
        else
        {
            //==============================================
            add =   FUN_B_BUF[k] * 256; //列表起始地址0X6500，获取KISS数据存储索引
            //读出呼号、距离

//				for (i = 0; i<9; i++) { temp[i]=AT24CXX_READ(add+16+i); temp[i+1]=' '; temp[i+2]=' ';temp[i+3]=0; } 	  //呼号

            for (i = 0; i < 9; i++)  	 //读出呼号，始终是9位，不足9位空格填充
            {
                dat = AT24CXX_READ(add + 16 + i);

                if(dat == 0)
                {
                    dat = ' ';   //替换成空格
                }

                temp[i] = dat;
                temp[i + 1] = 0;
            }

            strcat(SN_RX_BUFFER, temp);

            //==============================================
            strcat(SN_RX_BUFFER, " ");

            //==============================================
            for (i = 0; i < 8; i++)
            {
                UI_WD[i] = AT24CXX_READ(add + 0x20 + i);      //纬度
                UI_WD[i + 1] = 0;
            }

            for (i = 0; i < 9; i++)
            {
                UI_JD[i] = AT24CXX_READ(add + 0x30 + i);      //纬度
                UI_JD[i + 1] = 0;
            }

            no_sig = 1;

            if ((UI_WD[0] == 0) | (UI_JD[0] == 0) | (GPS_LOCKED == 0))
            {
                no_sig = 0;    //检查信标是否包含经纬度数据
            }

            if(no_sig == 1)
            {
//
//				UART2_SendString("===========\r\n");
//		UART2_SendString(GPS_WD);    UART2_SendString("  ");
//		UART2_SendString(GPS_JD);	  UART2_SendString("\r\n");
//		UART2_SendString(UI_WD);  UART2_SendString("  ");
//		UART2_SendString(UI_JD);	 UART2_SendString("\r\n");


                GET_AB_JULI(1);	  //MODE=1 求本机GPS位置和对方的2地距离,精度1米

//		  	UART2_SendString(GPS_WD);    UART2_SendString("  ");
//		UART2_SendString(GPS_JD);	  UART2_SendString("\r\n");
//		UART2_SendString(UI_WD);  UART2_SendString("  ");
//		UART2_SendString(UI_JD);	 UART2_SendString("\r\n");
//
//		UART2_SendString(UI_JULI);	 UART2_SendString("\r\n");
//		UART2_SendString("===========\r\n");

                UI_Angle_N =  GET_AB_Angle(1); //求移动站和对方的2地，角度
                UI_Angle_CAR =   GET_AB_POINT_DIR(UI_Angle_N, GPS_NOW_DIR)	;	//相对车头方向
//			Elevation_angle=GET_AB_EL(1);  //相对仰角
            }



//	for (i = 0; i<6; i++) { temp[i]=AT24CXX_READ(add+64+i); temp[i+1]=0; }
//
//	if (temp[0]==0){strcat(SN_RX_BUFFER,"------");}else	   //海拔

            //==============================================
            if(no_sig == 1)
            {
                strcat(SN_RX_BUFFER, UI_JULI + 1);	 //显示实时距离
            }
            else
            {
                strcat(SN_RX_BUFFER, "---.-");  	//距离显示----.-
            }

            //==============================================
            strcat(SN_RX_BUFFER, " ");

            //==============================================
            //==============================================	//相对正北方位
            if(no_sig == 1)			  //检查信标是否包含经纬度数据
            {
                tostring(UI_Angle_N);
                ASC_TEMP[0] = bai;
                ASC_TEMP[1] = shi;
                ASC_TEMP[2] = ge;
                ASC_TEMP[3] = 0;		 //相对正北方位
            }
            else
            {
                ASC_TEMP[0] = 0;
            }

            disp_fw();	   //显示正北方位
            strcat(SN_RX_BUFFER, ",");

//for (i = 0; i<16; i++) { UART2_SendData(AT24CXX_READ(add+0x20+i));   } 	  //调试距离
//33 31 31 31 2E 30 31 4E 00 31 32 30 2E 33 00 00
//33 31 33 34 2E 38 30 4E 00 30 2E 30 00 00 00 00
//33 31 33 34 2E 38 30 4E 00 30 2E 30 00 00 00 00
//33 31 33 34 2E 38 30 4E 00 30 2E 30 00 00 00 00
//33 31 30 30 2E 30 30 4E 00 38 39 2E 34 00 00 00

//				for(i=0;i<6;i++)	 {call[i]=' ';	 } //填充6个空格呼号
//				for(i=0;i<6;i++)
//				{
//				 if (FUN_B_BUF[k*8+i]==0){break;} //遇0退出
//				 call[i]=FUN_B_BUF[k*8+i];
//				} //填充呼号
//				call[6]='-';
//				//============================================== //填充SSID
//				SSID=FUN_B_BUF[k*8+6];	call[7]=SSID/10+0x30;  call[8]=SSID%10+0x30; if(call[7]=='0'){call[7]=' ';}
//				call[9]=0;
        }
    }

    strcat(SN_RX_BUFFER, "\r\n");


    UART4_SendString(SN_RX_BUFFER);	    //UART2_SendString(SN_RX_BUFFER);


    return 1;


}


uchar FUN_B(uint idx)	 	//显示记忆的距离和方位列表  $B00\r\n-$B19\r\n
{
    uint i, k;
    uint  add;
    uchar temp[20];
    uchar dat;	//uchar len;
//	uchar *p;


//	 	EEPROM_write_one(0x00B6, 0); 	//0=被动导航  1=动态导航
    if ( EEPROM_Buffer[0XB6] == 1)
    {
        FUN_B_GPS( idx);
        return 0;
    }

    //==============================================
//	p=strstr(UART4_BUF_DATA,"$B");
//	if  (p>0)
//	{
//		if (UART4_BUF_LENTH==6)		//
//		{
//			idx=(UART4_BUF_DATA[2]-0X30)*10+(UART4_BUF_DATA[3]-0X30);

    if (idx > 19)
    {
        return 0;
    }

    //==============================================  插入页面编号00-19
    SN_RX_BUFFER[0] = 0;
    strcat(SN_RX_BUFFER, "$B");
    temp[0] = idx / 10 + 0x30;
    temp[1] = idx % 10 + 0x30;
    temp[2] = 0;
    strcat(SN_RX_BUFFER, temp);
    strcat(SN_RX_BUFFER, ",");

    //==============================================
    for(i = 0; i < 5; i++)
    {
        FUN_B_BUF[i] = AT24CXX_READ(0x6500 + (idx * 5) + i);
    }

//				UART1_SendString("read a:  ");	  DEBUG_KISS(FUN_B_BUF,5);		//调试
    //起始地址0X6500,一共100字节，每1个字节一段，每页5个索引
    //==============================================
    //	   	for (i=0;i<20;i++)  		{  call[i]=0; }  	//Source呼号填充空格
    for(k = 0; k < 5; k++)
    {
        if (FUN_B_BUF[k] > 99)
        {
            strcat(SN_RX_BUFFER, "------ -- ---.- --,");	   //24C512没写入数据
        }
        else
        {
            //==============================================
            add =   FUN_B_BUF[k] * 256; //列表起始地址0X6500，获取KISS数据存储索引
            //读出呼号、距离




//				for (i = 0; i<9; i++) { temp[i]=AT24CXX_READ(add+16+i); temp[i+1]=' '; temp[i+2]=' ';temp[i+3]=0; } 	  //呼号

            for (i = 0; i < 9; i++)  	 //读出呼号，始终是9位，不足9位空格填充
            {
                dat = AT24CXX_READ(add + 16 + i);

                if(dat == 0)
                {
                    dat = ' ';   //替换成空格
                }

                temp[i] = dat;
                temp[i + 1] = 0;
            }

            strcat(SN_RX_BUFFER, temp);

            //==============================================
            strcat(SN_RX_BUFFER, " ");

            //==============================================
//				for (i = 0; i<6; i++) 	{ temp[i]=AT24CXX_READ(add+41+i);   temp[i+1]=0;} 	    //距离
//
//			  	switch (temp[0])
//				{
//					case 0:
//						strcat(SN_RX_BUFFER,"----.-");	//strcat(SN_RX_BUFFER,"Km,");	   	//如果没有距离则显示----.-
//						break;
//					case '*':	  //小于10,000米
//						temp[0]=' ';	strcat(SN_RX_BUFFER,temp);	//strcat(SN_RX_BUFFER,"Km,");
//						break;
//					default:
//						strcat(SN_RX_BUFFER,temp);	//strcat(SN_RX_BUFFER,"Km");
//						break;
//				}

//			  strcat(SN_RX_BUFFER,"Km,");

            //==============================================
            for (i = 0; i < 6; i++)
            {
                ASC_TEMP[i] = AT24CXX_READ(add + 41 + i);      //距离
                ASC_TEMP[i + 1] = 0;
            }

            switch (ASC_TEMP[0])
            {
                case 0:
                    strcat(SN_RX_BUFFER, "---.-");	//strcat(SN_RX_BUFFER,"Km,");	   	//如果没有距离则显示----.-
                    break;

                case '*':	  //小于10,000米
                    strcat(SN_RX_BUFFER, ASC_TEMP + 1);	//	strcat(SN_RX_BUFFER,"Km,");
                    break;

                default:
                    strcat(SN_RX_BUFFER, ASC_TEMP + 1);	//strcat(SN_RX_BUFFER,"Km,");
                    break;
            }

            strcat(SN_RX_BUFFER, " ");

            //==============================================	 方位
            for (i = 0; i < 3; i++)
            {
                ASC_TEMP[i] = AT24CXX_READ(add + 0x1a + i);    //相对正北方位
                ASC_TEMP[i + 1] = 0;
            }

            disp_fw();	   //显示正北方位
            strcat(SN_RX_BUFFER, ",");

//for (i = 0; i<16; i++) { UART2_SendData(AT24CXX_READ(add+0x20+i));   } 	  //调试距离
//33 31 31 31 2E 30 31 4E 00 31 32 30 2E 33 00 00
//33 31 33 34 2E 38 30 4E 00 30 2E 30 00 00 00 00
//33 31 33 34 2E 38 30 4E 00 30 2E 30 00 00 00 00
//33 31 33 34 2E 38 30 4E 00 30 2E 30 00 00 00 00
//33 31 30 30 2E 30 30 4E 00 38 39 2E 34 00 00 00

//				for(i=0;i<6;i++)	 {call[i]=' ';	 } //填充6个空格呼号
//				for(i=0;i<6;i++)
//				{
//				 if (FUN_B_BUF[k*8+i]==0){break;} //遇0退出
//				 call[i]=FUN_B_BUF[k*8+i];
//				} //填充呼号
//				call[6]='-';
//				//============================================== //填充SSID
//				SSID=FUN_B_BUF[k*8+6];	call[7]=SSID/10+0x30;  call[8]=SSID%10+0x30; if(call[7]=='0'){call[7]=' ';}
//				call[9]=0;
        }
    }

    strcat(SN_RX_BUFFER, "\r\n");


    UART4_SendString(SN_RX_BUFFER);	    //UART2_SendString(SN_RX_BUFFER);


    return 1;
//	}

    return 0;
}
//  0x0000-0x6400  （0-25600）,每条信标占用256字节，一共100条记录，索引0-99存储在 0x8000

//  0x6500-0x6600    ,列表存储区，每1字节一套索引，一共100条索引
//  0-5字节 呼号BH4TDV
//  6字节  SSID-XX 99
//	7字节  信标KISS存储地址索引	   0-99