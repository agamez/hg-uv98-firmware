#include<string.h>
#include "STC8A8K64D4.H"
#include "STC_EEPROM.H"
#include "UART1.H"
#include "UART2.H"
#include "UART4.H"

#include "IO.H"
#include "tostring.H"
#include "BEACON.H"
#include "AT24C512.h"
#include "KISS2ASC.H"
#include "FUN_C.h"
#include "GPS2.h"
#include "GPS_JULI.H"
#include "PUBLIC_BUF.H"
#include "KISS_Analysis.H"
#include  <string.H> //Keil library 


uchar WX_BUF[50] ;		  //缓冲尺寸
uchar WX_LINE1[30];
uchar WX_LINE2[30];


uchar read_hx(uchar *p)	   //显示8个方位
{
    uint dat;

    if (*p == 0)
    {
        strcat(SN_RX_BUFFER, "--");
        return 0;
    }

    dat = (uint)(*p - 0x30) * 100 + (uint)(*(p + 1) - 0x30) * 10 + (uint)(*(p + 2) - 0x30);

    //列表方位显示方式  0=英文  1=0-12  2-0-36
    angle_to_txt(dat, p);
    strcat(SN_RX_BUFFER, p);
    return 1;
}



void disp_wide_space(uchar len)
{
    uchar i;

    if(len == 0)
    {
        return;
    }

    for (i = 0; i < len; i++)
    {
        strcat(SN_RX_BUFFER, "------ --");
        strcat(SN_RX_BUFFER, ",");	 //路径n
    }

}


void disp_wide()
{

    uint i, n;
    uchar temp[20];
    uchar dat;
    uchar idx;
    uchar path_count;

//BH4TDV-10>APET51,WIDE1-1:!3134.31N/12020.22Er090/050/A=003080 12.1V
//BH4TDV-6>AP6688,BH4TDV-10*,WIDE1*:!3135.90N/12021.80E[668 9.7V  34.7C 1020.2hPa

    idx = 0;

    path_count = 0;

    for (i = 0; i < 20; i++)
    {
        dat = ASC_TEMP[idx];
        idx++;

        if (dat == ':')
        {
            disp_wide_space(5);
            return;
        }

        if (dat == ',')
        {
            break;   //找到第1个逗号
        }
    }


    for (n = 0; n < 5; n++) 		//最多含有5个路径
    {
        for (i = 0; i < 12; i++)
        {
            dat = ASC_TEMP[idx];
            idx++;

            if (dat == ':')
            {
                strcat(SN_RX_BUFFER, temp);
                strcat(SN_RX_BUFFER, ",");
                path_count++;
                disp_wide_space(5 - path_count);
                return;
            }

            if (dat == ',')
            {
                strcat(SN_RX_BUFFER, temp);	      //找到第5个逗号
                strcat(SN_RX_BUFFER, ",");
                path_count++;
                break;
            }

            temp[i] = dat;
            temp[i + 1] = 0;
        }
    }

//超过5个路径，则忽略
//
//	for (i = 0; i<10; i++)
//	{
//		dat= ASC_TEMP[idx];   idx++;
//		if (dat==':'){strcat(SN_RX_BUFFER,temp);	   strcat(SN_RX_BUFFER,",");	         return;}
//		if (dat==','){strcat(SN_RX_BUFFER,temp);	   strcat(SN_RX_BUFFER,",");	  break;}	 //找到第5个逗号
//	   	temp[i] =dat;	temp[i+1]=0;
//	}
}

void FUN_C_KISS_TO_ASC(uint add)  //读出存储的KISS数据，转成文本，存入ASC_TEMP
{
    uint i;
    uchar dat;
    KISS_LEN = 0;	//显示KISS

    for (i = 0; i < 128; i++)
    {
        dat =	AT24CXX_READ(add + i + 128); 	 //KISS在后128字节内

        if (dat == 0x00)
        {
            break;
        }

        KISS_DATA[i] = dat;
        KISS_LEN++;
    }

    KISS_TO_ASCII(ASC_TEMP, 0);	//KISS数据转换ASCII UI格式,并取得UI数据长度	UI_DIGI_LEN
    //==============================================
}


//	//==============================================


void FUN_C_DISP_WX()  //显示 气象板数据
{
    uchar  i  ;
    uchar temp[10];
    float dat;	//带小数点、带正负

    //--------------------------------------------
    //UART1_SendString(WX_BUF);  //调试
    //wxdata= "000/000g000t078r000p000h99b09966";
    //提取的外部接口板数据，格式：  //000/000g000t078r000p000h99b09966
    //提取的气象数据，32字节长度 格式：  //c000s000g000t093r000p000h48b10016

    //--------------------------------------------
    WX_LINE1[0] = 0;	 	//
    strcat(WX_LINE1, " ");	//2个空格

    //--------------------------------------------	 //风向
    for (i = 0; i < 3; i++)
    {
        temp[i] = WX_BUF[0 + i];
        temp[i + 1] = 0;
    }

    if (temp[0] == '0')		//前1-2位消隐处理
    {
        temp[0] = ' '	;

        if (temp[1] == '0')  temp[1] = ' '	;
    }

    strcat(WX_LINE1, temp);
    strcat(WX_LINE1, "  "); //2个空格

    //--------------------------------------------	 //前1分钟平均风速 地址在4
    for (i = 0; i < 3; i++)
    {
        temp[i] = WX_BUF[4 + i];
        temp[i + 1] = 0;
    }

    //风速，英制转公制,英里/小时*1.61*1000/3600=M/S
    dat = (temp[0] - 0x30) * 100 + (temp[1] - 0x30) * 10 + (temp[2] - 0x30);
    //	WD=30;
//	WD=WD*1.61/3.6;
//	temp[0]=(uint)WD/10%10+0X30 ;  temp[1]=(uint)WD%10+0X30 ;    temp[2] =0x00;
    dat = dat * 1.61 / 3.6 * 10;
    temp[0] = (uint)dat / 100 % 10 + 0X30 ;
    temp[1] = (uint)dat / 10 % 10 + 0X30 ;
    temp[2] = '.' ;
    temp[3] = (uint)dat % 10 + 0X30 ;
    temp[4] = 0x00;

    if (temp[0] == '0')
    {
        temp[0] = ' '	;    //前1位消隐处理
    }

    strcat(WX_LINE1, temp);
    strcat(WX_LINE1, "m/s  "); //

    //--------------------------------------------	 //湿度
    for (i = 0; i < 2; i++)
    {
        temp[i] = WX_BUF[24 + i];
        temp[i + 1] = 0;
    }

    strcat(WX_LINE1, temp);
    strcat(WX_LINE1, "% "); //1个空格

    //--------------------------------------------	 //温度
    for (i = 0; i < 3; i++)
    {
        temp[i] = WX_BUF[12 + i];
        temp[i + 1] = 0;
    }

    if (temp[0] == '-')
    {
        dat = -((float)(temp[1] - 0x30) * 10 + (float)(temp[2] - 0x30)) ;
    }
    else
    {
        dat = (float)(temp[0] - 0x30) * 100 + (float)(temp[1] - 0x30) * 10 + (float)(temp[2] - 0x30);
    }

    dat = (dat - 32) / 1.8 * 10;	//华氏转摄氏度//保留一位小数点

    if (dat < 0)	 //零下 //转成正数
    {
        dat = -dat;
        temp[0] = '-';
    }
    else
    {
        temp[0] = ' ';
    }

    temp[1] = (uint)dat / 100 % 10 + 0X30 ;
    temp[2] = (uint)dat / 10 % 10 + 0X30 ;
    temp[3] = '.' ;
    temp[4] = (uint)dat % 10 + 0X30 ;
    temp[5] = 0x00;

    strcat(WX_LINE1, temp);
    strcat(WX_LINE1, "C"); //

    //--------------------------------------------	 //
    strcat(WX_LINE1, ","); //

    //--------------------------------------------	 // 第2行
    //--------------------------------------------	 //


    //--------------------------------------------
    WX_LINE2[0] = 0;	 	//

//	strcat(WX_LINE2," ");	//1个空格
    //--------------------------------------------	  //前1小时雨量
    for (i = 0; i < 3; i++)
    {
        temp[i] = WX_BUF[16 + i];
        temp[i + 1] = 0;
    }

    //雨量，英制转公制，0.01英寸*0.254mm
    dat	= (float)(temp[0] - 0x30) * 100 + (float)(temp[1] - 0x30) * 10 + (float)(temp[2] - 0x30);
    dat = dat * 0.254 * 10;
    temp[0] = (uint)dat / 100 % 10 + 0X30 ;
    temp[1] = (uint)dat / 10 % 10 + 0X30 ;
    temp[2] = '.' ;
    temp[3] = (uint)dat % 10 + 0X30 ;
    temp[4] = 0x00;

    if (temp[0] == '0')
    {
        temp[0] = ' '	;    //前1位消隐处理
    }

    strcat(WX_LINE2, temp);
    strcat(WX_LINE2, "mm "); //


    //--------------------------------------------
    //--------------------------------------------	  //前24小时雨量
    for (i = 0; i < 3; i++)
    {
        temp[i] = WX_BUF[20 + i];
        temp[i + 1] = 0;
    }

    //雨量，英制转公制，0.01英寸*0.254mm
    dat	= (float)(temp[0] - 0x30) * 100 + (float)(temp[1] - 0x30) * 10 + (float)(temp[2] - 0x30);
    dat = dat * 0.254 * 10;
    temp[0] = (uint)dat / 100 % 10 + 0X30 ;
    temp[1] = (uint)dat / 10 % 10 + 0X30 ;
    temp[2] = '.' ;
    temp[3] = (uint)dat % 10 + 0X30 ;
    temp[4] = 0x00;

    if (temp[0] == '0')
    {
        temp[0] = ' '	;    //前1位消隐处理
    }

    strcat(WX_LINE2, temp);
    strcat(WX_LINE2, "mm "); //

    //--------------------------------------------

    //--------------------------------------------	//前5分钟瞬间最高风速 阵风
    for (i = 0; i < 3; i++)
    {
        temp[i] = WX_BUF[8 + i];
        temp[i + 1] = 0;
    }

    //风速，英制转公制,英里/小时*1.61*1000/3600=M/S
    dat = (temp[0] - 0x30) * 100 + (temp[1] - 0x30) * 10 + (temp[2] - 0x30);
    //	WD=30;
//	WD=WD*1.61/3.6;
//	temp[0]=(uint)WD/10%10+0X30 ;  temp[1]=(uint)WD%10+0X30 ;    temp[2] =0x00;
    dat = dat * 1.61 / 3.6 * 10;
    temp[0] = (uint)dat / 100 % 10 + 0X30 ;
    temp[1] = (uint)dat / 10 % 10 + 0X30 ;
    temp[2] = '.' ;
    temp[3] = (uint)dat % 10 + 0X30 ;
    temp[4] = 0x00;

    if (temp[0] == '0')
    {
        temp[0] = ' '	;    //前1位消隐处理
    }

    strcat(WX_LINE2, temp);
    strcat(WX_LINE2, " "); //	//strcat(WX_LINE2,"m/s ");//

    //-------------------------------------------- 	//气压 共5位
    for (i = 0; i < 5; i++)
    {
        temp[i] = WX_BUF[27 + i];
        temp[i + 1] = 0;
    }

    temp[5] = temp[4];
    temp[4] = '.';
    temp[6] = 0;	 //增加小数点

    if (temp[0] == '0')
    {
        temp[0] = ' '	;    //前1位消隐处理
    }

    strcat(WX_LINE2, temp);
    //--------------------------------------------	 //
    strcat(WX_LINE2, ","); //

    //--------------------------------------------
    //--------------------------------------------
    //--------------------------------------------
    //--------------------------------------------
}




void FUN_C_WX(uint add)
{
    uint i ;
    uchar dat;
    uchar idx;

    if (	AT24CXX_READ(add + 0X3F) != '_'	)
    {
        return;   // 图标  //气象信标单独处理
    }

//	if (	AT24CXX_READ(add+0X17)!='1'	){return;}	// 图标  //气象信标单独处理
//	if (	AT24CXX_READ(add+0X18)!='3'	){return;}	// 图标  //气象信标单独处理


    //显示自定义信息
    idx = 0;

    for (i = 0; i < 200; i++) 	//检索气象图标
    {
        dat = ASC_TEMP[idx];
        idx++;

        if (dat == '_')
        {
            break;   //包含气象图标
        }

        if (i > 198)
        {
            return;
        }
    }

    //提取的气象数据，32字节长度 格式：  //c000s000g000t093r000p000h48b10016
    for (i = 0; i < 32; i++)
    {
        WX_BUF[i] = ASC_TEMP[idx + i];
        WX_BUF[i + 1] = 0;
    }

    FUN_C_DISP_WX(); //显示 气象板数据


    strcat(SN_RX_BUFFER, "W,"); //气象标志
    strcat(SN_RX_BUFFER, WX_LINE1); //气象第1行
    strcat(SN_RX_BUFFER, WX_LINE2); //气象第2行

//  	strcat(SN_RX_BUFFER,"  120    1.3M/S 65% 17.3C,"); //气象显示第1行
//
//  	strcat(SN_RX_BUFFER," 0.0mm  0.0mm  0000  1000,"); //气象显示第3行


}

// 	uchar WX_DATA[200]={"c000s000g000t...r000p000h..b.....\r\n"};

//	c000(0-3)  s000(4-7)  g000(8-11)  t...(12-15)  r000(16-19)  p000(20-23)   h00(24-26)  b.....(27-32)

//输出数据，格式：c000s000g000t093r000p000h48b10016	 ,含换行符号，一共35个字节长度
//uchar WX_DATA[40]={"c000s000g000t032r000p000h00b.....\r\n"};
//c=报告风向
//s=报告前一分钟持续的风速（英里每小时）  10秒累计采样
//g=前5分钟,峰值风速,英里每小时
//t=华氏温度
//r=在过去一小时雨量（以百分之一英寸）。10分钟累计采样
//p=在过去24小时内的降雨量（以百分之一英寸）。

//P=雨量（以百分之一英寸）自午夜开始。不用
//h= 湿度（00％= 100％）。 //发DTH11湿度数据
//b=气压 0.1pa

//由正北方位和车头相对方位，倒推还原出自己的航向
//my_dir 我的航向（箭头方向），   UI_Angle_N正北方位（圆点位置）  UI_Angle_CAR相对车头方位
void get_my_dir()
{
    uint my_dir;

    if (UI_Angle_N < UI_Angle_CAR)
    {
        my_dir = 360 - (UI_Angle_CAR - UI_Angle_N);
    }
    else
    {
        my_dir = UI_Angle_N - UI_Angle_CAR;
    }

}


//呼号,速度
void FUN_C_READ(uint add)	   //
{
    uint i ;
    uchar temp[20];
    uchar dat;
    uchar idx;
    uchar no_wd_jd;

//	if ( EEPROM_Buffer[0XB6]==1) {FUN_C_GPS( add); return;}
//
    FUN_C_KISS_TO_ASC( add) ; //读出存储的KISS数据，转成文本，存入ASC_TEMP
    FUN_C_WX(add);	//气象处理


//呼号,速度,距离,海拔,纬度,NS,经度,WE,对方航向,相对正北方位,相对车头方位,日期,时间,路径1,路径2,路径3,路径4,路径5,自定义信息
    for (i = 0; i < 9; i++)
    {
        temp[i] = AT24CXX_READ(add + 16 + i);    //呼号
        temp[i + 1] = 0;
    }

    strcat(SN_RX_BUFFER, temp);
    strcat(SN_RX_BUFFER, ",");

//  for (i = 0; i<6; i++) { temp[i]=AT24CXX_READ(add+71+i); temp[i+1]=0; } 	  //速度
//	if (temp[0]==0){strcat(SN_RX_BUFFER,"---.-");}else{strcat(SN_RX_BUFFER,temp);} strcat(SN_RX_BUFFER,"Km/h");  strcat(SN_RX_BUFFER,",");

    for (i = 0; i < 5; i++)
    {
        temp[i] = AT24CXX_READ(add + 0x48 + i);    //速度,只读5位
        temp[i + 1] = 0;
    }

    if (temp[0] == 0)
    {
        strcat(SN_RX_BUFFER, "---.-");
    }
    else
    {
        for (i = 0; i < 2; i++)
        {
            if (temp[i] == '0')
            {
                temp[i] = ' ';   //替换成空格
            }
            else
            {
                break;
            }
        }

        strcat(SN_RX_BUFFER, temp);
    }

    strcat(SN_RX_BUFFER, "Kmh ");



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

    no_wd_jd = 1;

    if ((UI_WD[0] == 0) | (UI_JD[0] == 0) | (GPS_LOCKED == 0) )
    {
        no_wd_jd = 0;    //检查信标是否包含经纬度数据
    }

//	if(no_wd_jd==0)	  //检查信标不是否包含经纬度数据
//	{
//
//	}

    for (i = 0; i < 3; i++)
    {
        temp[i] = AT24CXX_READ(add + 0x3a + i);    //对方航向
        temp[i + 1] = 0;
    }

    read_hx(temp) ;
    strcat(SN_RX_BUFFER, " ");


    if ( EEPROM_Buffer[0XB6] == 1)	 //开启动态，并且包含经纬度，并且GPS有效定位，则计算实时距离
    {
        if(no_wd_jd == 1)
        {
            GET_AB_JULI(1);	  //MODE=1 求本机GPS位置和对方的2地距离,精度1米
            UI_Angle_N =  GET_AB_Angle(1); //求移动站和对方的2地，角度
            UI_Angle_CAR =   GET_AB_POINT_DIR(UI_Angle_N, GPS_NOW_DIR)	;	//相对车头方向
        }
    }

    if ( EEPROM_Buffer[0XB6] == 1)		//开启动态实时计算相对车头方向
    {

        if(no_wd_jd == 1)	 //检查信标是否包含经纬度数据
        {
            tostring(UI_Angle_CAR);
            temp[0] = bai;
            temp[1] = shi;
            temp[2] = ge;
            temp[3] = 0;		 //相对车头方位
            read_hx(temp);
        }
        else
        {
            strcat(SN_RX_BUFFER, "--");
        }

        strcat(SN_RX_BUFFER, ",");
        //==============================================

        if(no_wd_jd == 1)					   	//距离显示6位
        {
            strcat(SN_RX_BUFFER, UI_JULI + 1);	 //显示实时距离
        }
        else
        {
            strcat(SN_RX_BUFFER, "---.-");	   //如果没有距离则显示----.-
        }

        strcat(SN_RX_BUFFER, "Km,");

    }
    else
    {

        for (i = 0; i < 3; i++)
        {
            temp[i] = AT24CXX_READ(add + 0x1D + i);    //相对车头方位
            temp[i + 1] = 0;
        }

        read_hx(temp) ;
        strcat(SN_RX_BUFFER, ",");

        //==============================================
        for (i = 0; i < 6; i++)
        {
            temp[i] = AT24CXX_READ(add + 41 + i);      //距离显示6位
            temp[i + 1] = 0;
        }

        switch (temp[0])
        {
            case 0:
                strcat(SN_RX_BUFFER, "----.-");
                strcat(SN_RX_BUFFER, "Km,");	   	//如果没有距离则显示----.-
                break;

            case '*':	  //小于10,000米
                temp[0] = ' ';
                strcat(SN_RX_BUFFER, temp + 1);
                strcat(SN_RX_BUFFER, "Km,");
                break;

            default:
                strcat(SN_RX_BUFFER, temp + 1);
                strcat(SN_RX_BUFFER, "Km,");
                break;
        }

    }



    //==============================================


//	for (i = 0; i<6; i++) { temp[i]=AT24CXX_READ(add+64+i); temp[i+1]=0; } 	  //海拔
//	if (temp[0]==0){strcat(SN_RX_BUFFER,"------");}else{strcat(SN_RX_BUFFER,temp);}  strcat(SN_RX_BUFFER,"m");  strcat(SN_RX_BUFFER,",");

    for (i = 0; i < 6; i++)
    {
        temp[i] = AT24CXX_READ(add + 64 + i);
        temp[i + 1] = 0;
    }

    if (temp[0] == 0)
    {
        strcat(SN_RX_BUFFER, "------");
    }
    else	    //海拔
    {
        for (i = 0; i < 5; i++)
        {
            if (temp[i] == '0')
            {
                temp[i] = ' ';   //替换成空格
            }
            else
            {
                break;
            }
        }

        strcat(SN_RX_BUFFER, temp);
    }

    strcat(SN_RX_BUFFER, "M");
    strcat(SN_RX_BUFFER, ",");


    //==============================================
    strcat(SN_RX_BUFFER, " ");
    temp[0] = AT24CXX_READ(add + 0x20);
    temp[1] = AT24CXX_READ(add + 0x21);
    temp[2] = '@';	//	strcat(SN_RX_BUFFER,temp);		strcat(SN_RX_BUFFER,"@");			//纬度
    temp[3] = AT24CXX_READ(add + 0x22);
    temp[4] = AT24CXX_READ(add + 0x23);
    temp[5] = '.';	//		strcat(SN_RX_BUFFER,temp);		strcat(SN_RX_BUFFER,".");
    temp[6] = AT24CXX_READ(add + 0x25);
    temp[7] = AT24CXX_READ(add + 0x26);
    temp[8] = '\'';	//		strcat(SN_RX_BUFFER,temp);		strcat(SN_RX_BUFFER,"'");
//    strcat(SN_RX_BUFFER," ");
    temp[9] = AT24CXX_READ(add + 0x27);
    temp[10] = 0;	 // NS

    if (temp[0] == 0)
    {
        strcat(SN_RX_BUFFER, "--@--.--'-");
    }
    else
    {
        strcat(SN_RX_BUFFER, temp);
    }

    strcat(SN_RX_BUFFER, ",");
    //==============================================
    temp[0] = AT24CXX_READ(add + 0x30);
    temp[1] = AT24CXX_READ(add + 0x31);
    temp[2] = AT24CXX_READ(add + 0x32);
    temp[3] = '@';
    temp[4] = AT24CXX_READ(add + 0x33);
    temp[5] = AT24CXX_READ(add + 0x34);
    temp[6] = '.';
    temp[7] = AT24CXX_READ(add + 0x36);
    temp[8] = AT24CXX_READ(add + 0x37);
    temp[9] = '\'';
    temp[10] = AT24CXX_READ(add + 0x38);
    temp[11] = 0;	 // WE

// 	for (i = 0; i<3; i++) { temp[i]=AT24CXX_READ(add+0x30+i); temp[i+1]=0; } 	strcat(SN_RX_BUFFER,temp);	   	strcat(SN_RX_BUFFER,"@");	  	//经度
//	for (i = 0; i<2; i++) { temp[i]=AT24CXX_READ(add+0x33+i); temp[i+1]=0; } 	strcat(SN_RX_BUFFER,temp);	   	strcat(SN_RX_BUFFER,".");
//	for (i = 0; i<2; i++) { temp[i]=AT24CXX_READ(add+0x36+i); temp[i+1]=0; } 	strcat(SN_RX_BUFFER,temp);	   	strcat(SN_RX_BUFFER,"'");
// 	strcat(SN_RX_BUFFER," ");
//	temp[0]=AT24CXX_READ(add+0x38); temp[1]=0;

    if (temp[0] == 0)
    {
        strcat(SN_RX_BUFFER, "---@--.--'-");
    }
    else
    {
        strcat(SN_RX_BUFFER, temp);
    }

    strcat(SN_RX_BUFFER, ",");
    //==============================================

    for (i = 0; i < 3; i++)
    {
        temp[i] = AT24CXX_READ(add + 0x3a + i);    //对方航向
        temp[i + 1] = 0;
    }

    if (temp[0] == 0)
    {
        strcat(SN_RX_BUFFER, "---");
    }
    else
    {
        strcat(SN_RX_BUFFER, temp);
    }

    strcat(SN_RX_BUFFER, ",");



    if ( EEPROM_Buffer[0XB6] == 1)		//开启动态实时计算相对车头方向
    {

        if(no_wd_jd == 1)	 //检查信标是否包含经纬度数据
        {
            tostring(UI_Angle_N);				 //相对正北方位
            temp[0] = bai;
            temp[1] = shi;
            temp[2] = ge;
            temp[3] = 0;
            strcat(SN_RX_BUFFER, temp);
        }
        else
        {
            strcat(SN_RX_BUFFER, "---");
        }

        strcat(SN_RX_BUFFER, ",");

        if(no_wd_jd == 1)	 //检查信标是否包含经纬度数据
        {
            tostring(UI_Angle_CAR);	  //相对车头方位
            temp[0] = bai;
            temp[1] = shi;
            temp[2] = ge;
            temp[3] = 0;
            strcat(SN_RX_BUFFER, temp);
        }
        else
        {
            strcat(SN_RX_BUFFER, "---");
        }

        strcat(SN_RX_BUFFER, ",");
    }
    else
    {
        for (i = 0; i < 3; i++)
        {
            temp[i] = AT24CXX_READ(add + 0x1a + i);    //相对正北方位
            temp[i + 1] = 0;
        }

        if (temp[0] == 0)
        {
            strcat(SN_RX_BUFFER, "---");
        }
        else
        {
            strcat(SN_RX_BUFFER, temp);
        }

        strcat(SN_RX_BUFFER, ",");

        for (i = 0; i < 3; i++)
        {
            temp[i] = AT24CXX_READ(add + 0x1D + i);    //相对车头方位
            temp[i + 1] = 0;
        }

        if (temp[0] == 0)
        {
            strcat(SN_RX_BUFFER, "---");
        }
        else
        {
            strcat(SN_RX_BUFFER, temp);
        }

        strcat(SN_RX_BUFFER, ",");
    }


    for (i = 0; i < 8; i++)
    {
        temp[i] = AT24CXX_READ(add + 8 + i);    //日期
        temp[i + 1] = 0;
    }

    strcat(SN_RX_BUFFER, temp);
    strcat(SN_RX_BUFFER, ",");

    for (i = 0; i < 8; i++)
    {
        temp[i] = AT24CXX_READ(add + 0 + i);    //时间
        temp[i + 1] = 0;
    }

    strcat(SN_RX_BUFFER, temp);
    strcat(SN_RX_BUFFER, ",");

    //==============================================

//		KISS_LEN =0;	//显示KISS
//		for (i = 0; i<128; i++)
//		{ dat=	AT24CXX_READ(add+i+128); 	  //KISS在后128字节内
//		if (dat==0x00){break;}
//		KISS_DATA[i]=dat;	KISS_LEN++;
//		}
//		KISS_TO_ASCII(ASC_TEMP,0);	//KISS数据转换ASCII UI格式,并取得UI数据长度	UI_DIGI_LEN
//		//==============================================

//	   	 FUN_C_KISS_TO_ASC( add) ; //读出存储的KISS数据，转成文本，存入ASC_TEMP

    disp_wide();	   	//显示路径
    //==============================================
    //显示自定义信息
    idx = 0;

    for (i = 0; i < 200; i++) 	//检索冒号
    {
        dat = ASC_TEMP[idx];
        idx++;

        if (dat == ':')
        {
            break;
        }

        if (i > 198)
        {
            strcat(SN_RX_BUFFER, "----,");
            return;
        }
    }

    for (i = 0; i < 200 - idx; i++) 	//分析剩余字符
    {
        dat = ASC_TEMP[idx];
        idx++;

        if (dat == 0x00)
        {
            break;
        }

        if (dat == 0x0D)
        {
            break;
        }

        if ((dat > '~') | (dat < ' '))
        {
            dat = 'x';
        }

        ASC_TEMP[i] = dat;
        ASC_TEMP[i + 1] = 0;
    }

    strcat(SN_RX_BUFFER, ASC_TEMP);
    strcat(SN_RX_BUFFER, ",");
}

















void DISP_A08()
{
    uint  add;
    SN_RX_BUFFER[0] = 0;
    strcat(SN_RX_BUFFER, "$A08,");
    add =  AT24CXX_READ(0x6500 ) * 256; //列表起始地址0X6500，获取首行KISS数据存储索引
    FUN_C_READ(add);
    strcat(SN_RX_BUFFER, "\r\n");

    UART4_SendString(SN_RX_BUFFER);	   //	UART2_SendString(SN_RX_BUFFER);
}



uchar FUN_C(uint idx)	   //详细信标
{
    uint  add;
    uchar temp[8];	  //uint i;

    if (idx > 99)
    {
        return 0;
    }

    //==============================================  插入索引编号00-99
    SN_RX_BUFFER[0] = 0;
    strcat(SN_RX_BUFFER, "$C");
    temp[0] = idx / 10 + 0x30;
    temp[1] = idx % 10 + 0x30;
    temp[2] = 0;
    strcat(SN_RX_BUFFER, temp);
    strcat(SN_RX_BUFFER, ",");
    //==============================================

    add =  AT24CXX_READ(0x6500 + idx ) * 256; //列表起始地址0X6500，获取KISS数据存储索引

    if (AT24CXX_READ(add) == 0xff)
    {
        strcat(SN_RX_BUFFER, "NO DATA\r\n");		   //24C512没写入数据
    }
    else
    {
        FUN_C_READ(add);
        strcat(SN_RX_BUFFER, "\r\n");
    }

    UART4_SendString(SN_RX_BUFFER);	   //	UART2_SendString(SN_RX_BUFFER);

//		for (i = 0; i<256; i++) { UART2_SendData(AT24CXX_READ(add+0+i));   } 	  //调试
    return 0;
}

//  0x0000-0x6400  （0-25600）,每条信标占用256字节，一共100条记录，索引0-99存储在 0x8000

//  0x6500-0x6600    ,列表存储区，每1字节一套索引，一共100条索引
//  0-5字节 呼号BH4TDV
//  6字节  SSID-XX 99
//	7字节  信标KISS存储地址索引	   0-99

