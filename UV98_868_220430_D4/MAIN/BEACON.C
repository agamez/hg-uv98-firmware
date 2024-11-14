#include "STC8A8K64D4.H"
#include "STC_EEPROM.H"
#include "tostring.H"

#include "UART2.H"
#include "UART4.H"

#include "IO.H"
#include "ADC.H"

#include "GPS2.H"

#include "MICE_CODE.H"

#include "CMX865A_CODE.H"
#include "BEACON.H"
#include "DIGI.H"

#include "DELAY.H"
#include "CHx.H"

#include "bmp280.h"

#include "PUBLIC_BUF.H"


uint GPS_BEACON_TIME; 		   //GPS定时信标计时



////网关电台RF信标，KISS格式，不含C0 00 ..C0，不含校验值，末尾补结束符0x00 ,结束符0X00不发送
//unsigned char xdata Gate_beacen[128]=
//{	 0x82, 0xA0, 0x9E, 0xA8, 0x86, 0x62, 0xE0, 				//	目标地址 +SSID
//	 0x84, 0x90, 0x68, 0xA8, 0x88, 0xAC, 0xF4, 				//	源地址	 +SSID
//	 0xAE, 0x92, 0x88, 0x8A, 0x62, 0x40, 0x63, 				//	路径	 +SSID
//	 0x03, 0xF0, 											//	03f0
//	 0x21, 													//	类型
//	 0x33, 0x31, 0x30, 0x30, 0x2E, 0x30, 0x30, 0x4E, 		//	经纬度
//	 0x2F, 													//	分隔符	"/"
//	 0x31, 0x32, 0x31, 0x30, 0x30, 0x2E, 0x30, 0x30, 0x45, 	//	经纬度
//	 0x3E, 													//	图标类型
//	 0x20, 0x30, 0x37, 0x2E, 0x38, 0x56, 0x20, 0x32, 0x31, 0x43, 0x20, 0x6F, 0x74,		 //信息
//	 0x00, };	//非文本字符串尾部编译器不会补上00H，需手动补上，结束符0X00不发送

//网关网络IS信标
//BH4TDV-10>APET51:!3134.31N/12020.22E>000/000/A=000027	  //14+22  22x833x8	=147ms
//BH4TDV-10>SYUUQ1:`,<|l+Z[/`"4D}      //14字节

uchar GPS_TO_KISS()		//0=网关信标KISS,1=气象接口板KISS,2=WS1气象信标KISS,3=GPS信标KISS	 //GPS转KISS数据
{
    uchar i, k, temp;
    uchar MICE_EN;
    long ALT_TEMP;

    uchar B_SSID, PATH1, PATH2;

//	uchar code  WX_DST_NAME[]  ={"WX51  "} ;	 //目标地址6位
    uchar code   DST_NAME[]    = {"APUV98"} ; //目标地址6位
//	uchar code   KISS_WIDE1[]  ={"WIDE1 "} ;	 //路径地址6位，含空格
//	uchar code   KISS_WIDE2[]  ={"WIDE2 "} ;	 //路径地址6位，含空格
//	uchar code   LOGO[]        ={"51G3"} ;

//BH4TDV-10>APET51:!3134.31N/12020.22E>000/000/A=000027	  //14+22  22x833x8	=147ms
//BH4TDV-10>SYUUQ1:`,<|l+Z[/`"4D}      //14字节

    MICE_EN = EEPROM_Buffer[0X10];

    if (MICE_EN == 1)
    {
        Encoding_MICE();
    }


    k = 0;

    if (EEPROM_Buffer[0X2A] == 1) //0=固定站，设定的经纬度  1=移动站,GPS经纬度
    {
        if (MICE_EN == 1)
        {
            for (i = 0; i < 6; i++)
            {
                KISS_DATA[k] = (MICE_WD[i] << 1 );     //转换DST_NAME
                k++;
            }
        }
        else
        {
            for (i = 0; i < 6; i++)
            {
                KISS_DATA[k] = (DST_NAME[i] << 1 );     //转换DST_NAME
                k++;
            }
        }
    }
    else
    {
        for (i = 0; i < 6; i++)
        {
            KISS_DATA[k] = (DST_NAME[i] << 1 );     //转换DST_NAME
            k++;
        }
    }


    KISS_DATA[k] = 0x60;
    k++;	//插入 DST_NAME SSID 或WX_DST_NAME SSID
    //--------------------------------

    for (i = 0; i < 6; i++)
    {
        KISS_DATA[k + i]	= 0x40;    //Source呼号填充空格
    }

    for (i = 0; i < 6; i++)  					  	//Source呼号填充空格
    {
        if (EEPROM_Buffer[0x08 + i] == 0x00)
        {
            break;
        }

        KISS_DATA[k] = EEPROM_Buffer[0x08 + i] << 1;
        k++;
    }

    //--------------------------------

    B_SSID = EEPROM_Buffer[0X0F];	 //信标SSID

    PATH1 = EEPROM_Buffer[PATH1_COUNT]; //插入转发路径1
    PATH2 = EEPROM_Buffer[PATH2_COUNT]; //插入转发路径2

    k = 13;

    if ((PATH1 == 0) && (PATH2 == 0))
    {
        KISS_DATA[k] = 0x60 | (B_SSID << 1) | 0x01;    //插入路径结束符=1
        k++;
    }
    else
    {
        KISS_DATA[k] = 0xE0 | (B_SSID << 1) ;
        k++;	  //插入 Source SSID +路径结束符=0	 ,高位=1

        if (PATH1 != 0)
        {
            for (i = 0; i < 6; i++)
            {
                KISS_DATA[k + i]	= 0x40;    //转换路径WIDE1
            }

            for (i = 0; i < 6; i++)  		//Source呼号填充空格
            {
                if (EEPROM_Buffer[PATH1_NAME + i] == 0x00)
                {
                    break;
                }

                KISS_DATA[k + i] = EEPROM_Buffer[PATH1_NAME + i] << 1;
            }

            k = k + 6;
            KISS_DATA[k] = 0x60 | (EEPROM_Buffer[PATH1_COUNT] << 1);  	//插入 WIDE1 SSID=1  + 路径结束符=0或1

            if(PATH2 == 0)
            {
                KISS_DATA[k] |= 0x01;    //插入路径结束符=1
                k++;
            }
            else
            {
                k++;
            }
        }


        if (PATH2 != 0)
        {
            for (i = 0; i < 6; i++)
            {
                KISS_DATA[k + i]	= 0x40;    //转换路径WIDE1
            }

            for (i = 0; i < 6; i++)  		//Source呼号填充空格
            {
                if (EEPROM_Buffer[PATH2_NAME + i] == 0x00)
                {
                    break;
                }

                KISS_DATA[k + i] = EEPROM_Buffer[PATH2_NAME + i] << 1;
            }

            k = k + 6;
            KISS_DATA[k] = 0x60 | (EEPROM_Buffer[PATH2_COUNT] << 1) | 0x01;
            k++;	//插入 WIDE2 SSID=1  + 路径结束符=1
        }

    }

    KISS_DATA[k] = 0x03 ;
    k++;	//插入控制码03
    KISS_DATA[k] = 0xF0 ;
    k++;	//插入PRO码F0


//for(i=0;i<18;i++)  { KISS_DATA[k]= GPS_DATA[i]; k++;} //插入GPS经纬度
//GPS提取出的经纬度数据，长度=18字节，格式：3134.03N/12020.19E


    if (EEPROM_Buffer[0X2A] == 1) //0=固定站，设定的经纬度  1=移动站,GPS经纬度
    {

        if (MICE_EN == 1)
        {
            for (i = 0; i < 14; i++)
            {
                KISS_DATA[k]	= (MICE_JD[i] );     //压缩数据
                k++;
            }

//		for (i=0;i<4;i++)  	{  	KISS_DATA[k]	=LOGO[i];  k++; 	}  	//LOGO
        }
        else
        {
            KISS_DATA[k] = EEPROM_Buffer[0X12];
            k++;	//插入类型符	   、31字节

            //GPS原始纬度 或GPS纠偏后的纬度
            for(i = 0; i < 8; i++)
            {
                KISS_DATA[k] = GPS_WD[i];
                k++;
            }

            //插入图标集
            if (SMART_MODE == 2)
            {
                KISS_DATA[k] = EEPROM_Buffer[0XBC];
                k++;
            }
            else
            {
                KISS_DATA[k] = EEPROM_Buffer[0X13];
                k++;
            }

//			KISS_DATA[k]=EEPROM_Buffer[0X13]; k++;
            //GPS原始经度 或GPS纠偏后的经度
            for(i = 0; i < 9; i++)
            {
                KISS_DATA[k] = GPS_JD[i];
                k++;
            }

            //插入图标符号
            if (SMART_MODE == 2)
            {
                KISS_DATA[k] = EEPROM_Buffer[0XBD];
                k++;
            }
            else
            {
                KISS_DATA[k] = EEPROM_Buffer[0X14];
                k++;
            }

//			KISS_DATA[k]=EEPROM_Buffer[0X14]; k++;

            tostring(GPS_NOW_DIR);	  //航向
            KISS_DATA[k] = bai;
            k++;
            KISS_DATA[k] = shi;
            k++;
            KISS_DATA[k] = ge;
            k++;
            KISS_DATA[k] = '/';
            k++;

            tostring(GPS_NOW_SPEED_KNOT);	  //速度
            KISS_DATA[k] = bai;
            k++;
            KISS_DATA[k] = shi;
            k++;
            KISS_DATA[k] = ge;
            k++;
            //插入海拔	   一共9个字节		//    /A=000027
            KISS_DATA[k] = '/';
            k++;
            KISS_DATA[k] = 'A';
            k++;
            KISS_DATA[k] = '=';
            k++;

            for(i = 0; i < 6; i++)
            {
                KISS_DATA[k] = GPS_HEIGHT[i];
                k++;
            }
        }
    }
    else	//0=固定站，设定的经纬度，信标使用设置的固定经纬度发
    {
        KISS_DATA[k] = EEPROM_Buffer[0X12];
        k++;	//插入类型符	   、31字节

        //固定站点纬度
        for(i = 0; i < 8; i++)
        {
            KISS_DATA[k] = EEPROM_Buffer[0x20 + i];
            k++;
        }

        KISS_DATA[k] = EEPROM_Buffer[0X13];
        k++;	//插入图标集

        for(i = 0; i < 9; i++)
        {
            KISS_DATA[k] = EEPROM_Buffer[0x30 + i];     //固定站点经度
            k++;
        }

        KISS_DATA[k] = EEPROM_Buffer[0X14];
        k++;	//插入图标符号


        tostring(0);	  //航向
        KISS_DATA[k] = bai;
        k++;
        KISS_DATA[k] = shi;
        k++;
        KISS_DATA[k] = ge;
        k++;
        KISS_DATA[k] = '/';
        k++;

        tostring(0);	  //速度
        KISS_DATA[k] = bai;
        k++;
        KISS_DATA[k] = shi;
        k++;
        KISS_DATA[k] = ge;
        k++;
        //插入海拔	   一共9个字节		//    /A=000027
        KISS_DATA[k] = '/';
        k++;
        KISS_DATA[k] = 'A';
        k++;
        KISS_DATA[k] = '=';
        k++;

        ALT_TEMP =	(float) ( EEPROM_Buffer[0x0129] * 256 + EEPROM_Buffer[0x012A]) * 3.2808;	 //转换成英制

        GPS_HEIGHT[0] = ALT_TEMP / 100000 % 10 + 0x30;
        GPS_HEIGHT[1] = ALT_TEMP / 10000 % 10 + 0x30;
        GPS_HEIGHT[2] = ALT_TEMP / 1000 % 10 + 0x30;
        GPS_HEIGHT[3] = ALT_TEMP / 100 % 10 + 0x30;
        GPS_HEIGHT[4] = ALT_TEMP / 10 % 10 + 0x30;
        GPS_HEIGHT[5] = ALT_TEMP % 10 + 0x30;
        GPS_HEIGHT[6] = 0x00; //结束符号

        for(i = 0; i < 6; i++)
        {
            KISS_DATA[k] = GPS_HEIGHT[i];
            k++;
        }

    }

    for(i = 0; i < 60; i++) //插入自定义信息,限制自定义信息长度
    {
        temp = EEPROM_Buffer[0x40 + i];

        if (temp == 0x00)
        {
            break;
        }

        KISS_DATA[k] = temp;
        k++;

        if (k > 100)
        {
            KISS_DATA[k] = '~';      //限定字符长度
            k++;
            break;
        }
    }


    GET_LC();	  //里程计算

    if ((EEPROM_Buffer[0X3E] == 1) && (EEPROM_Buffer[0X2A] == 1) )  	 //插入里程	 5个字节	  0-650.00km		 //moto 1-4
    {

        KISS_DATA[k] = ' ';
        k++;	  //	KISS_DATA[k]=LC_TIME;   k++;
        tostring((uint)(TOTAL_LC / 100));
        KISS_DATA[k] = wan ;
        k++;
        KISS_DATA[k] = qian ;
        k++;
        KISS_DATA[k] = bai ;
        k++;
        KISS_DATA[k] = shi ;
        k++;
        KISS_DATA[k] = '.';
        k++;
        KISS_DATA[k] = ge ;
        k++;

        KISS_DATA[k] = 'K';
        k++;
        KISS_DATA[k] = 'm';
        k++;
    }

    //---------------------------------
    if (EEPROM_Buffer[0X3C] == 1)   	//插入电压
    {
        KISS_DATA[k] = ' ';
        k++; //插入一个空格					   6个字节
        READ_ADC();	//读取电压值
        i = 0;

        while (DY[i] != 0x00)
        {
            KISS_DATA[k] = DY[i];    //插入电压
            i++;
            k++;
        }

        KISS_DATA[k] = 'V';
        k++;
    }

//	if (EEPROM_Buffer[0X3D]==1)   	//插入温度
//	{
//		if (DS18B20_READTEMP()==1)
//		{
//		i=0;
//		while (DS18B20_TEMP[i]!=0x00)	  	{KISS_DATA[k]=DS18B20_TEMP[i]; i++; k++;}  //插入温度
//		KISS_DATA[k]='C';   k++;
//		}
//	}


//
//	if (EEPROM_Buffer[0X3D]==1)
//	{
//		//插入DS18B20
//		if (DS18B20_READTEMP()==1)	 //如果检测到DS18B20，则插入第2温度			 6个字节
//		{
//		KISS_DATA[k]=' ';   k++; //插入一个空格
//		i=0;
//		while (DS18B20_TEMP[i]!=0x00)	  	{KISS_DATA[k]=DS18B20_TEMP[i]; i++; k++;}  //插入温度
//
//		KISS_DATA[k]='C';   k++;
//		}
//	}

//
//  	if (EEPROM_Buffer[0X3D]==1)
//	{
//		//插入DS18B20
//		if (read_sht2x()==1)	 //如果检测到sht2x，则插入第2温度			 6个字节
//		{
//		KISS_DATA[k]=' ';   k++; //插入一个空格
//		i=0;
//		while (SHT2X_DATA[i]!=0x00)	  	{KISS_DATA[k]=SHT2X_DATA[i]; i++; k++;}  //插入温度
//
//		}
//	}
//



    READ_BMP280();

    //---------------------------------
    if (EEPROM_Buffer[0x003D] == 1) 		//报告温度
    {
        if( BMP280_LINK == 1)
        {
            KISS_DATA[k] = ' ';
            k++;
            i = 0;

            while (BMP280_TEMP[i] != 0x00)
            {
                KISS_DATA[k] = BMP280_TEMP[i];    //插入温度
                i++;
                k++;
            }

            KISS_DATA[k] = 'C';
            k++;
        }

    }

    //---------------------------------

    if (EEPROM_Buffer[0x003b] == 1)
    {
        if( BMP280_LINK == 1)	//0=没安装	1=安装	 		//气压（0.1 hpa）		9个字节
        {
            KISS_DATA[k] = ' ';
            k++;

            i = 0;

            while (BMP280_QY[i] != 0x00)
            {
                KISS_DATA[k] = BMP280_QY[i];    //插入气压
                i++;
                k++;
            }

            KISS_DATA[k] = 'h';
            k++;
            KISS_DATA[k] = 'P';
            k++;
            KISS_DATA[k] = 'a';
            k++;
        }
    }




    if ((EEPROM_Buffer[0X2A] == 1) && (EEPROM_Buffer[0X3F] == 1)) //0=固定站，设定的经纬度  1=移动站,GPS经纬度	 4个字节
    {
        //---------------------------------
        KISS_DATA[k] = ' ';
        k++;   //插入定位的卫星数量
        KISS_DATA[k] = 'S';
        k++;
        KISS_DATA[k] = GPS_SIG[0];
        k++;
        KISS_DATA[k] = GPS_SIG[1];
        k++;
    }


    KISS_LEN = k;
    return 1;
}






void BEACON_GPS_TXD()	 //发送移动站/固定站RF信标
{
    //如果是省电模式下，并且GPS是关闭状态，则打开GPS
    if ((EEPROM_Buffer[0X2D] == 1) && (GPS_EN == 0))
    {
        GPS_EN = 1;
        UART2_SendString("GPS ON\r\n");
        return;
    }

    if (EEPROM_Buffer[0X2A] == 1)
    {
        if (GPS_LOCKED == 0)
        {
            UART2_SendString("Wait GPS Lock \r\n");    //移动站时，GPS必须锁定才允许发射
            return;
        }
    }

    GPS_TO_KISS();	   //信标数据组合

    BEACON_TX_CHX(0);
}




void BECON_MODE()
{
    if (EEPROM_Buffer[0X02] == 1)			//定时信标
    {
        if (GPS_BEACON_TIME > (EEPROM_Buffer[0X00] * 256 +	EEPROM_Buffer[0X01]) - 1)
        {
            GPS_BEACON_TIME = 0;
            BEACON_GPS_TXD();	 	 //发送移动站/固定站RF信标
        }
    }

    //GPS省电，发射信标后，自动关闭
    if ((EEPROM_Buffer[0X03] == 1) | (EEPROM_Buffer[0X02] == 1))	//当开启了定时发射或开启了手动发射模式
    {
        //当移动站时，当GPS省电功能开启，当GPS有效定位，则发送信标，关闭GPS
        if ((EEPROM_Buffer[0X2D] == 1) && (EEPROM_Buffer[0X2A] == 1) && (GPS_EN == 1) && (GPS_LOCKED == 1))
        {
            BEACON_GPS_TXD();
            GPS_EN = 0;
            GPS_LOCKED = 0; //LED_STU=1;
            UART2_SendString("GPS OFF\r\n");
        }
    }
}

