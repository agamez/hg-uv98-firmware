#include "STC8A8K64D4.H"
#include "STC_EEPROM.H"
// #include "HMI.H"

//#include "KISS2ASC.H"
#include "MICE_DECODE.H"
#include "UART1.H"
#include "GPS2.H"
#include "GPS_JULI.H"
#include "tostring.H"

#include "KISS_Analysis.H"

//#include "BEACON.H"
#include "KISS2ASC.H"
#include "BT.H"

#include "BEACON_SAVE.H"
#include "PUBLIC_BUF.H"



uchar UI_CALL[10];
//uchar UI_SSID[3];
uchar UI_DEST[10];
uchar UI_TIME[10];
uchar UI_WD[10];
uchar UI_WD_DIR;
uchar UI_JD[10];
uchar UI_JD_DIR;
uchar UI_ICON;

uchar UI_DIR[10];
uchar UI_SPEED[10];
uchar UI_ALT[10];
uchar UI_JULI[10];

uint UI_Angle_N;	//相对正北方位
uint UI_Angle_CAR;  //相对车头方位

float UI_ALT_MI;	 //信标海拔高度

//uchar W25Q64_BUF_A[128];


uint Elevation_angle; //相对仰角


//void CLEAN_UI_DATA();

//uchar CONV_JD( uint JD);	 //角度指向






void CLEAN_UI_DATA()
{
    uchar i;

    for(i = 0; i < 10; i++)
    {
        UI_CALL[i] = 0x00;
    }

    for(i = 0; i < 10; i++)
    {
        UI_WD[i] = 0x00;   //8字节
    }

    for(i = 0; i < 10; i++)
    {
        UI_JD[i] = 0x00;   //9字节
    }

    for(i = 0; i < 10; i++)
    {
        UI_DIR[i] = 0x00;   //航向3字节
    }

    for(i = 0; i < 10; i++)
    {
        UI_SPEED[i] = 0x00;   //速度6字节
    }

    for(i = 0; i < 10; i++)
    {
        UI_ALT[i] = 0x00;   //6字节
    }

    for(i = 0; i < 10; i++)
    {
        UI_JULI[i] = 0x00;   //6字节
    }

    UI_Angle_N = 0;	 //相对正北方位
    UI_Angle_CAR = 0; //相对车头方位
    UI_ALT_MI = 0;	 //信标海拔高度
    Elevation_angle = 0; //相对仰角

//	for(i=0;i<10;i++){W25Q64_BUF_A[i]=0x00;}

    for(i = 0; i < 10; i++)
    {
        UI_DEST[i] = 0x00;
    }

    for(i = 0; i < 10; i++)
    {
        UI_TIME[i] = 0x00;   //
    }
}





void SPEED_MI()	 //速度海里转成公制米
{
    unsigned long SPEED;
    uchar i;	  //	uchar i,n;
//		uint	txt;

    for(i = 0; i < 3; i++)
    {
        UI_SPEED[i] = UI_SPEED[i] - 0X30;   //文本转成数字
    }

    SPEED = (long)UI_SPEED[0] * 100 + UI_SPEED[1] * 10 + UI_SPEED[2];
    SPEED = SPEED * 1.852 * 10;	//转换成米,0.1KM

//		txt=(uint)SPEED;
//		UI_SPEED[0]=txt/10000+0x30;
//		UI_SPEED[1]=txt/1000%10+0x30;
//		UI_SPEED[2]=txt/100%10+0x30;
//		UI_SPEED[3]=txt/10%10+0x30;
//		UI_SPEED[4]='.';
//		UI_SPEED[5]=txt%10+0x30;
//		UI_SPEED[6]=0x00;

    tostring((uint)SPEED);
    UI_SPEED[0] = wan;
    UI_SPEED[1] = qian;
    UI_SPEED[2] = bai;
    UI_SPEED[3] = shi;
    UI_SPEED[4] = '.';
    UI_SPEED[5] = ge;
    UI_SPEED[6] = 0x00;


//		n=0;
//		while(UI_SPEED[0]=='0')	 //首位0消隐 ,最少保留一位
//		{
//		for(i=0;i<7;i++) 	{ UI_SPEED[i]=UI_SPEED[i+1]; }
//		n++; if (n>2){break;}
//		}
}



void ALT_MI()	 //海拔英制转成公制米
{
    unsigned long temp_ALT;
    //	uchar  GPS_ALT_ZERO;	//小于0标志
    //	uchar  GPS_ALT_ERR;		//GPS没有效定位，无海拔数据，则数据无效
    uchar i;		 //uchar i,n;
    uint txt;

    if (UI_ALT[0] == '-')
    {
        for(i = 1; i < 6; i++)
        {
            UI_ALT[i] = UI_ALT[i] - 0X30;   //文本转成数字
        }

        temp_ALT = (long)UI_ALT[1] * 10000 + (long)UI_ALT[2] * 1000 + (long)UI_ALT[3] * 100 + UI_ALT[4] * 10 + UI_ALT[5];
        temp_ALT = temp_ALT * 0.3048;	//转换成米

        UI_ALT_MI = 0; //低于海平面，信标海拔直接设0

//		txt=(uint)GPS_ALT;
//		UI_ALT[1]=txt/10000+0x30;
//		UI_ALT[2]=txt/1000%10+0x30;
//		UI_ALT[3]=txt/100%10+0x30;
//		UI_ALT[4]=txt/10%10+0x30;
//		UI_ALT[5]=txt%10+0x30;
//		UI_ALT[6]=0x00;

        tostring((uint)temp_ALT);
        UI_ALT[1] = wan;
        UI_ALT[2] = qian;
        UI_ALT[3] = bai;
        UI_ALT[4] = shi;
        UI_ALT[5] = ge;
        UI_ALT[6] = 0x00;

//		n=0;
//		while(UI_ALT[1]=='0')	 //首位0消隐 ,最少保留一位
//		{
//		for(i=1;i<7;i++) 	{ UI_ALT[i]=UI_ALT[i+1]; }
//		n++; if (n>3){break;}
//		}

    }
    else
    {
        for(i = 0; i < 6; i++)
        {
            UI_ALT[i] = UI_ALT[i] - 0X30;	   //文本转成数字
        }

        temp_ALT = (long)UI_ALT[0] * 100000 + (long)UI_ALT[1] * 10000 + (long)UI_ALT[2] * 1000 + (long)UI_ALT[3] * 100 + UI_ALT[4] * 10 + UI_ALT[5];
        temp_ALT = temp_ALT * 0.3048;	//转换成米

        UI_ALT_MI = (float)temp_ALT;	 // 信标海拔

        txt = (uint)temp_ALT;
        UI_ALT[0] = txt / 100000 + 0x30;
        UI_ALT[1] = txt / 10000 % 10 + 0x30;
        UI_ALT[2] = txt / 1000 % 10 + 0x30;
        UI_ALT[3] = txt / 100 % 10 + 0x30;
        UI_ALT[4] = txt / 10 % 10 + 0x30;
        UI_ALT[5] = txt % 10 + 0x30;
        UI_ALT[6] = 0x00;


//		n=0;
//		while(UI_ALT[0]=='0')	 //首位0消隐 ,最少保留一位
//		{
//		for(i=0;i<7;i++) 	{ UI_ALT[i]=UI_ALT[i+1]; }
//		n++; if (n>4){break;}
//		}
    }
}





unsigned char READ_UI_indx(uchar *p)	 //检索冒号起始置
{
    unsigned char  k ;

    for (k = 0; k < 125; k++)		 //最多检索125
    {
        if (*p  == ':')
        {
            return k + 1;	   //返回数据起始地址
        }

        p++;
    }

    return 0; //检索错误
}

void READ_UI_CALL(uchar *p)	 //呼号、ID	、SOURCE
{
    uchar i, txt;

    for(i = 0; i < 10; i++)
    {
        txt = *p;
        p++;

        if (txt == '>')
        {
            break;
        }

        UI_CALL[i] = txt ;
    }

    for(i = 0; i < 10; i++)
    {
        txt = *p;
        p++;

        if ((txt == ':') | (txt == ','))
        {
            break;
        }

        UI_DEST[i] = txt ;
    }
}

//BG8TFC-12>AP51G2:!2502.07N/10134.94E>167/029/A=005917APRS 51Track 51G2 Tel 13577841761 14.6V S06


void Resolution_UI_A(uchar *p, uchar idx)	  //解析 ! = 类型数据
{


    uchar i, txt;

    p += idx;

    txt = *p;
    p++;

    if ((txt == '/') | (txt == '@'))
    {
        for(i = 0; i < 7; i++)
        {
            UI_TIME[i] = *p;	   //解析 / @ 带时间日期类型数据
            p++;
        }
    }

//   BH4TDV-7>APUV98,WIDE1-1:!3133.90S/12022.80E[HG-UV98 9.7V  32.0C 1018.1hPa
    for(i = 0; i < 8; i++)
    {
        UI_WD[i] = *p;	   //纬度
        p++;
    }

    UI_WD_DIR = UI_WD[7];  	 // NS


    p++;		//跳过图标集 /

    for(i = 0; i < 9; i++)
    {
        UI_JD[i] = *p;
        p++;
    }

    UI_JD_DIR = UI_JD[8];   	 // WE





    UI_ICON = *p;
    p++;	  //图标

    //判断是否是气象站数据
    if (UI_ICON == '_')
    {
        return;
    }	//

    //判断是否有速度航向海拔数据


    UI_ALT_MI = 0;	 // 信标海拔

    if (*(p + 3) == '/')
    {
        for(i = 0; i < 3; i++)
        {
            UI_DIR[i] = *p;	   //航向
            p++;
        }

        p++;

        for(i = 0; i < 3; i++)
        {
            UI_SPEED[i] = *p;	   //速度
            p++;
        }

        SPEED_MI();	 //速度海里转成公制米

        if ((*p == '/') && (*(p + 1) == 'A') && (*(p + 2) == '=')) //判断是否有海拔数据
        {
            p++;
            p++;
            p++;

            for(i = 0; i < 6; i++)
            {
                UI_ALT[i] = *p;	   //海拔英制
                p++;
            }

            ALT_MI();	 //海拔英制转成公制米
        }
    }
    else  //没有航向
    {


    }

    //自定信息
//	i=0;
//	while(*p!=0x00)
//	{UI_MSG[i]=*p; p++;	i++;
//	if (i>70){UI_MSG[i]=0x00;	break;}	//限制长度
//	}

}


//常见数据格式
//BG8TFC-12>AP51G2:!2502.07N/10134.94E>167/029/A=005917APRS 51Track 51G2 Tel 13577841761 14.6V S06
//BH1PEZ-7>SYUUQ1,WIDE1-1,WIDE2-1:`,<|l+Z[/`"4D}Welcome to QSO with me!._(
//BG6KOY-5>APDR13,TCPIP*::BH6MCZ-11:能收到不？{1
//BH7PCT-15>APLM3D,TCPIP*:@111031h2304.63N/11321.44E>246/009/A=000042http://www.aprs.cn G07 M17 11.4V
//BH4TDV-6>AP51TT:>HELLO 14.30V 15.6C
uchar Resolution_UI_DATA()	 //解析UI数据类型
{
    uchar idx;

    READ_UI_CALL(SN_RX_BUFFER);	 //呼号、ID	、SOURCE


    idx = READ_UI_indx(SN_RX_BUFFER); //检索数据的第一个冒号

    if (idx == 0)
    {
        return 0;   //检索错误
    }

    switch (SN_RX_BUFFER[idx]) //判断数据类型符号
    {
        case '!':	//A不带时间
            Resolution_UI_A(SN_RX_BUFFER, idx);
            break;

        case '=':	//A不带时间
            Resolution_UI_A(SN_RX_BUFFER, idx);
            break;


        case '/':	 //B带时间
            Resolution_UI_A(SN_RX_BUFFER, idx);
            break;

        case '@':	 //B带时间
            Resolution_UI_A(SN_RX_BUFFER, idx);
            break;

        case 0x27:	//C压缩
            return 2;
            break;

        case 0x60:	//C压缩
            return 2;
            break;

        case ':':	//D信息
            return 3;
            break;

        case '>':	 //状态
            return 4;
            break;


        default:   //G未知
            return 5;
            break;
    }

    return 1;
}




void GET_JULI_DIR_EL()	  //求距离个、相对正北方向 、相对车头方向
{
    if (EEPROM_Buffer[0x002a] == 0)	 //0=固定站，设定的经纬度  1=移动站,GPS经纬度
    {
        GET_AB_JULI(0);			   //求固定站和对方2地，距离
        UI_Angle_N =	GET_AB_Angle(0); //求固定站和对方2地，角度
        UI_Angle_CAR = UI_Angle_N; //相对车头方向

        Elevation_angle = GET_AB_EL(0); //相对仰角
    }
    else
    {
        if (GPS_LOCKED == 1)
        {
            GET_AB_JULI(1);			   //求移动站和对方的2地，距离
            UI_Angle_N =	GET_AB_Angle(1); //求移动站和对方的2地，角度
//			UI_Angle_CAR=UI_Angle_N+(360-GPS_NOW_DIR);	//相对车头方向

// 		   if  (UI_Angle_N>GPS_NOW_DIR)	 {UI_Angle_CAR=UI_Angle_N-GPS_NOW_DIR;}	   	//相对车头方向
//		   else	   { UI_Angle_CAR=360-(GPS_NOW_DIR-UI_Angle_N);	 }

            UI_Angle_CAR =   GET_AB_POINT_DIR(UI_Angle_N, GPS_NOW_DIR)	;	//相对车头方向

            Elevation_angle = GET_AB_EL(1); //相对仰角
        }
    }
}

void DISP_KISS_DATA()   //解析并显示对方的定位数据,并显示
{
    uchar STU;
    CLEAN_UI_DATA();

    if (KISS_TO_MICE() == 1)
    {
        GET_JULI_DIR_EL();	   //按压缩数据解析,分解呼号、ID	、SOURCE解析经纬度、航向、速度、海拔、信息
        BEACON_SAVE();
        BT_OUT(1);
        return;
    }

    KISS_TO_ASCII(SN_RX_BUFFER, 0);	//KISS转成ASC文本
    STU = Resolution_UI_DATA();  	//status  1= 未压缩常规数据格式  2=`压缩  3=:短消息 4=>状态  5=未知


    if (STU == 0x01)
    {
        GET_JULI_DIR_EL();    //按普通数据解析,分解呼号、ID	、SOURCE解析经纬度、航向、速度、海拔、信息
        BEACON_SAVE();
        BT_OUT(1);
    }
    else
    {
        BEACON_SAVE();    //未知格式，禁止输出航点，按普通数据解析,分解呼号、ID	、SOURCE解析经纬度、航向、速度、海拔、信息
        BT_OUT(0);
    }

}




//
//写入信标列表和存储信标

//	uchar code KISS_DEMO[]=
//	{
//	 0x82, 0xA0, 0x9E, 0xA8, 0x86, 0x62, 0xE0, 				//	目标地址 +SSID
//	 0x84, 0x90, 0x68, 0xA8, 0x88, 0xAC, 0xF2, 				//	源地址	 +SSID
//	 0xAE, 0x92, 0x88, 0x8A, 0x62, 0x40, 0x63, 				//	路径	 +SSID
//	 0x03, 0xF0, 											//	03f0
//	 0x21, 													//	类型
//	 0x33, 0x31, 0x30, 0x30, 0x2E, 0x30, 0x30, 0x4E, 		//	经纬度
//	 0x2F, 													//	分隔符	"/"
//	 0x31, 0x32, 0x31, 0x30, 0x30, 0x2E, 0x30, 0x30, 0x45, 	//	经纬度
//	 0x3E, 													//	图标类型
//	 0x20, 0x30, 0x37, 0x2E, 0x38, 0x56, 0x20, 0x32, 0x31, 0x43, 0x20, 0x6F, 0x74,		 //信息
//	};

