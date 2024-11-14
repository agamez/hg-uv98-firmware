
#include "STC8A8K64D4.H"
#include "STC_EEPROM.H"
#include "IO.H"
//#include "DELAY.H"
//#include "ADC.H"
#include "tostring.H"

#include "GPS2.H"
#include "GPS_CHECKSUM.H"
#include "GPS_JULI.H"

#include "BEACON.H"
#include "UART2.H"
#include "UART3.H"
#include "UART4.H"

#include "AT24C512.h"
#include "PUBLIC_BUF.H"
#include "DISP_GPS.H"

#include<string.h>



uchar GPS_TIME[10] = 0;	//时间
uint AUTO_TIME;



uchar GPS_DATE[13] = 0;	//日期

uchar GPS_WD[15] = 0;	//纬度
uchar GPS_JD[15] = 0; //经度

bit GPS_LOCKED;

uint GPS_NOW_DIR;		//GPS当前方向0-359度
uchar GPS_NOW_DIR_TXT[4];



uint GPS_OLD_DIR = 0;		//GPS前次方向0-359度
bit GPS_DIR_NOTHING;	//0=静止，无航向数据  1=移动中，有航向数据

uint GPS_NOW_SPEED_KNOT = 0;	//提取的GPS当前速度0-255海里  1海里=1.852公里
uchar GPS_NOW_SPEED_KNOT_TXT[10];

uint GPS_SPEED_KM = 0;	//GPS速度 单位公里
//uchar GPS_SPEED_KM_TXT[10];


uchar  GPS_ALT_ERR;		//GPS没有效定位，1=无海拔数据，则数据无效  0=海拔数据正常

uchar GPS_HEIGHT[7] = 0;	//海拔高度度
long GPS_ALT_MI;

uchar GPS_SIG[3] = 0; //定位的卫星数量	00-12

uint GPS_WAIT_TIME;   //(0-255)*10秒





//uchar LC_TIME;   //里程周期A-Z
uchar LAST_WD[10] = 0;	//上次纬度
uchar LAST_JD[10] = 0; //上次经度


uchar TIME_GPS_SEC;
uchar SMART_MODE;



bit QUICK_SET_EN;  //0= 禁止快速设置固定站		 1=允许快速设置固定站

//uchar GET_LC();

//里程
uchar lc_start;  //0=首次发射，里程累计清0  1=非首次清0，继续累计里程
unsigned  long  TOTAL_LC, AB_LC;   // 0-6500.0km		unsigned  long 四字节 //0～4294967295




//	0xFE00  最后512 存储设置数据
void LC_MEMORY()  //记录里程
{
    if (EEPROM_Buffer[0x00BB] == 0)
    {
        return;
    }

    //累计里程记忆	  0=OFF 1=ON

//	EEPROM_Buffer[0x00B7]=	 (uchar)(TOTAL_LC/256/256/256);
//	EEPROM_Buffer[0x00B8]=	 (uchar)(TOTAL_LC/256/256);
//	EEPROM_Buffer[0x00B9]=	 (uchar)(TOTAL_LC/(256));
//	EEPROM_Buffer[0x00Ba]=	 (uchar)(TOTAL_LC%(256));
//
//	EEPROM_UPDATA();

    AT24CXX_WRITE(0x0FE00, (uchar)(TOTAL_LC / 256 / 256 / 256)) ;
    AT24CXX_WRITE(0x0FE01, (uchar)(TOTAL_LC / 256 / 256)) ;
    AT24CXX_WRITE(0x0FE02, (uchar)(TOTAL_LC / (256)));
    AT24CXX_WRITE(0x0FE03, (uchar)(TOTAL_LC % (256))) ;

}


void LC_CLEAN()  //里程清0
{
    TOTAL_LC = 0;
    AT24CXX_WRITE(0x0FE00, 0 );
    AT24CXX_WRITE(0x0FE01, 0) ;
    AT24CXX_WRITE(0x0FE02, 0);
    AT24CXX_WRITE(0x0FE03, 0) ;
}




uchar GET_LC()
{
    uchar i;

    if (lc_start == 0)
    {
        lc_start = 1; //开始计里程

        AB_LC = 0;	 //首次发里程清0
    }
    else
    {
        AB_LC = GET_JULI(LAST_WD, LAST_JD, GPS_WD, GPS_JD);	 //计算里程
        TOTAL_LC = TOTAL_LC + AB_LC;	//里程累计

        //超过5000KM，里程清0，里程单位增1（字母A-Z,a-z）
        if (TOTAL_LC > 500000000)
        {
            TOTAL_LC = 0;   //大于50万公里，清0
        }

        LC_MEMORY();
    }

//   	UART1_SendString("============\r\n");
//		UART1_DEBUG(GPS_NOW_SPEED_KNOT); UART1_SendString("\r\n");
//
//	UART1_SendString(LAST_WD);	 UART1_SendString(LAST_JD);	 UART1_SendString("\r\n");
//	UART1_SendString(GPS_WD);	 UART1_SendString(GPS_JD);	 UART1_SendString("\r\n");
//	UART1_DEBUG((uint)(AB_LC));	  UART1_SendString(" ");
//	UART1_DEBUG((uint)(TOTAL_LC));


    for (i = 0; i < 9; i++)
    {
        LAST_WD[i] = GPS_WD[i];     //更新纬度位置
    }

    for (i = 0; i < 10; i++)
    {
        LAST_JD[i] = GPS_JD[i];     //更新经度位置
    }


    return 1;
}


void INIT_LC()	  //里程初始化
{
    lc_start = 0;	 //停止计里程

    if (EEPROM_Buffer[0x00BB] == 0)
    {
        TOTAL_LC = 0;   //累计里程记忆	  0=OFF 1=ON
    }
    else
    {
        //读出记忆的里程
//	 	TOTAL_LC=(long)(EEPROM_Buffer[0x00B7])*256*256*256+(long)(EEPROM_Buffer[0x00B8])*256*256+(long)(EEPROM_Buffer[0x00B9])*256+(long)(EEPROM_Buffer[0x00Ba]);

//	 	TOTAL_LC=(long)(AT24CXX_READ(0XFE00))*256*256*256+(long)(AT24CXX_READ(0XFE01))*256*256+(long)(AT24CXX_READ(0XFE02))*256+(long)(AT24CXX_READ(0XFE03));

        if (AT24CXX_READ(0XFE00) == 0xff)	 //24CXX未初始化，总里程清0
        {
            LC_CLEAN();  //里程清0
        }
        else
        {
            TOTAL_LC = (long)(AT24CXX_READ(0XFE00)) * 256 * 256 * 256 + (long)(AT24CXX_READ(0XFE01)) * 256 * 256 + (long)(AT24CXX_READ(0XFE02)) * 256 + (long)(AT24CXX_READ(0XFE03));
        }


    }

    AB_LC = 0;	 //首次发里程清0
}









void GET_GPS_TIME(uchar *nbuffer)	   //截取GPS时间
{
    uchar i, k, n;
//$GPRMC,032149.365,A,3134.0372,N,12020.2073,E,0.00,345.47,300113,,,A*66
    i = 0;
    k = 0;

    while (nbuffer[i] != 0x00)
    {
        if (nbuffer[i] == ',')
        {
            k++;
        }

        i++;

        if (k == 1)
        {
            break;
        }
    }

    if (nbuffer[i] == ',')
    {
        return;
    }

    k = 0;
    GPS_TIME[k] = nbuffer[i];
    k++;
    i++;		//
    GPS_TIME[k] = nbuffer[i];
    k++;
    i++;		//
    GPS_TIME[k] = ':';
    k++; 		//

    GPS_TIME[k] = nbuffer[i];
    k++;
    i++;		//
    GPS_TIME[k] = nbuffer[i];
    k++;
    i++;		//
    GPS_TIME[k] = ':';
    k++; 	//

    GPS_TIME[k] = nbuffer[i];
    k++;
    i++;		//
    GPS_TIME[k] = nbuffer[i];
    k++;
    i++;		//
    GPS_TIME[k] = 0X00;		//

    // 转换成24小时


    if (EEPROM_Buffer[0x002C] < 13) //时区调整
    {
        n = ((GPS_TIME[0] - 0x30) * 10 + (GPS_TIME[1] - 0x30)) - (13 - EEPROM_Buffer[0x002C]);
    }
    else
    {
        n = ((GPS_TIME[0] - 0x30) * 10 + (GPS_TIME[1] - 0x30)) + (EEPROM_Buffer[0x002C] - 13);
    }

    if (n >= 24)
    {
        n -= 24;
    }

    AUTO_TIME = n;
    GPS_TIME[0] = n / 10 + 0x30;
    GPS_TIME[1] = n % 10 + 0x30;

    TIME_GPS_SEC =	(GPS_TIME[6] - 0x30) * 10 +	(GPS_TIME[7] - 0x30);
}

//uchar GPS_DATE[10];	//日期

void GET_GPS_DATE(uchar *nbuffer)	   //截取GPS日期
{
    uchar i, k;
    uchar temp[10];

    unsigned char Bhour = 0, Bday = 0, Bmonth = 0;
    unsigned int Byear = 0;


//		for(i=0;i<12;i++)  {	GPS_DATE[i]=0;}

//$GPRMC,032149.365,A,3134.0372,N,12020.2073,E,0.00,345.47,300113,,,A*66
    i = 0;
    k = 0;

    while (nbuffer[i] != 0x00)
    {
        if (nbuffer[i] == ',')
        {
            k++;
        }

        i++;

        if (k == 9)
        {
            break;
        }
    }

    if (nbuffer[i] == ',')
    {
        return;
    }

    temp[0] = nbuffer[i];
    temp[1] = nbuffer[i + 1];	//日
    temp[2] = nbuffer[i + 2];
    temp[3] = nbuffer[i + 3];	//月
    temp[4] = nbuffer[i + 4];
    temp[5] = nbuffer[i + 5];	//年

    Bday = (temp[0] - 0x30) * 10 + (temp[1] - 0x30);
    Bmonth = (temp[2] - 0x30) * 10 + (temp[3] - 0x30);
    Byear = (temp[4] - 0x30) * 10 + (temp[5] - 0x30) + 2000;

    if (AUTO_TIME < 8) 		 //调日期
    {
        Bday++;	    //日期数加1

        switch(Bday)		//判断日期
        {
            case 29:	//普通年的2月份
                if((!((Byear % 400 == 0) || ((Byear % 4 == 0) && (Byear % 100 != 0))) && (Bmonth == 2)))
                {
                    Bday = 1;
                    Bmonth++;
                }

                break;

            case 30:      //如果是闰年的2月
                if(((Byear % 400 == 0) || ((Byear % 4 == 0) && (Byear % 100 != 0))) && (Bmonth == 2))
                {
                    Bday = 1;
                    Bmonth++;
                }

                break;

            case 31:
                if((Bmonth == 4) || (Bmonth == 6) || (Bmonth == 9) || (Bmonth == 11))
                {
                    Bday = 1;
                    Bmonth++;
                }

                break;

            case 32:
                Bday = 1;
                Bmonth++;

                if(Bmonth >= 13)
                {
                    Byear++;
                    Bmonth = 1;
                }

                break;
        }
    }


//	Byear=Byear-2000;

    k = 0;
    tostring(Byear);
    GPS_DATE[k++] = qian;
    GPS_DATE[k++] = bai;
    GPS_DATE[k++] = shi;
    GPS_DATE[k++] = ge;
    GPS_DATE[k++] = '-';
    tostring(Bmonth);
    GPS_DATE[k++] = shi;
    GPS_DATE[k++] = ge;
    GPS_DATE[k++] = '-';
    tostring(Bday);
    GPS_DATE[k++] = shi;
    GPS_DATE[k++] = ge;
    GPS_DATE[k++] = 0X00;
}




void GPS_INIT()
{
    if (EEPROM_Buffer[0x002B] == 1)
    {
        GPS_EN = 1;   //GPS打开或关闭
    }
    else
    {
        GPS_EN = 0;
    }

    GPS_OLD_DIR = 0;	//初始GPS前次角度=0度。
    GPS_NOW_DIR = 0;		 //初始航向=0
    GPS_LOCKED = 0;

    INIT_LC();	   //里程初始化
    TIME_GPS_SEC = 0;


    QUICK_SET_EN = 0; //禁止快速设置固定站
}


uchar GET_GPS_LOCK_STATUS(uchar *nbuffer) //检查GPS数据GPRMC，是否定位，0=未定位，1=定位
{
    //数据帧中提取一行
    //$GPRMC,032149.365,A,3134.0372,N,12020.2073,E,0.00,345.47,300113,,,A*66
    uchar i, k;

    i = 0;
    k = 0;

    while (nbuffer[i] != 0x00)
    {
        if (nbuffer[i] == ',')
        {
            k++;
        }

        i++;

        if (k == 2)
        {
            break;   //跳过2个,号
        }
    }

    //1=GPS定位		0=GPS未定位  	GPS_LOCKED=1; 	return 1;	 }
    if  (nbuffer[i] == 'A')
    {
        GPS_LOCKED = 1;
        return 1;
    }

    GPS_LOCKED = 0;	 // 	LED_STU=!LED_STU;
    return 0;
}

uchar GET_GPS_LONG_LAT(uchar *nbuffer)	   //截取经纬度，保存到GPS_DATA
{
    uchar i, k, n; //uchar tempA,tempB;
    //数据帧中提取一行
    //$GPRMC,032149.365,A,3134.0372,N,12020.2073,E,0.00,345.47,300113,,,A*66
    //数据解析转换处理


    i = 0;
    k = 0;

    while (nbuffer[i] != 0x00)
    {
        if (nbuffer[i] == ',')
        {
            k++;
        }

        i++;

        if (k == 3)
        {
            break;   //跳过3个,号
        }
    }

//A,3134.0432,N,12020.2056,E,0.00,353.93,010213,,,A*64
//	if  (nbuffer[i]!='A')  	{ 	return 0;}	  	//05=GPS未定位,不再继续解析
//	i++;  	  i++;			   //第3个','
//	while (nbuffer[i]!=',') { i++;  }

    //3134.0377,N,12020.2048,E,0.00,198.43,010213,,,A*6B

//	k=0;
//	while (nbuffer[i]!='.')	{ GPS_WD[k]=nbuffer[i];	 k++;	i++;  } 	//插入纬度，到小数点后2位

//	GPS_WD[k]=nbuffer[i];	 k++;	i++;	   //插入小数点
//	GPS_WD[k]=nbuffer[i];	 k++;	i++;	   //插入小数点后2位
//	GPS_WD[k]=nbuffer[i];	 k++;	i++;	   //
//
//	while (nbuffer[i]!=',') { i++;  } i++;//第4个','
//
//	//N,12020.2048,E,0.00,198.43,010213,,,A*6B
//	GPS_WD[k]=nbuffer[i];	 k++;	i++;		//插入 N=南半球，S=北半球

//220430 支持小数点5位
//$GNRMC,082737.00,A,2457.09420,N,11833.39780,E,0.467,,300422,,,A,V*19
//$GNGGA,082737.00,2457.09420,N,11833.39780,E,1,04,2.54,18.9,M,9.6,M,,*45

    k = 0;
    GPS_WD[k++] = '0';

    for(n = 0; n < 9; n++)
    {
        GPS_WD[k++] = nbuffer[i];    //插入纬度，到小数点后4位
        i++;
    }

    if (nbuffer[i] == ',')
    {
        i++;   //4位
    }
    else
    {
        i++;    //5位
        i++;
    }

    GPS_WD[k++] = nbuffer[i];	 		 //插入 N=南半球，S=北半球
    GPS_WD[k++] = 0x00;	//插入结束符号

    i++;
    i++;

    k = 0;

    for(n = 0; n < 10; n++)
    {
        GPS_JD[k++] = nbuffer[i];    //插入经度，到小数点后4位
        i++;
    }

    if (nbuffer[i] == ',')
    {
        i++;   //4位
    }
    else
    {
        i++;    //5位
        i++;
    }

    GPS_JD[k++] = nbuffer[i];	  	 //插入E=东经，W=西经
    GPS_JD[k++] = 0x00;	//插入结束符号


//	GPS_WD_NEW=(tempA-0x30)*10+(tempB-0X30);	 //纬度小数部分
//-------------------------------------------------------
//	while (nbuffer[i]!=',') { i++;  } i++;//第5个','
//	//12020.2048,E,0.00,198.43,010213,,,A*6B
//	k=0;
//	while (nbuffer[i]!='.') 	{ GPS_JD[k]=nbuffer[i];	k++;	i++;  }	//插入经度，到小数点后2位
//	GPS_JD[k]=nbuffer[i];	 k++;	i++;	   //插入小数点
////		tempA=nbuffer[i];
////	GPS_JD[k]=tempA;	 k++;	i++;	   //插入小数点后2位
////		tempB=nbuffer[i];
////	GPS_JD[k]=tempB;	 k++;	i++;	   //
//	GPS_JD[k]=nbuffer[i];	 k++;	i++;	   //插入小数点后2位
//	GPS_JD[k]=nbuffer[i];	 k++;	i++;	   //
//	while (nbuffer[i]!=',') { i++;  } i++;//第6个','
//E,0.00,198.43,010213,,,A*6B
//	GPS_JD[k]=nbuffer[i];	 k++;	i++;   //插入E=东经，W=西经
//	GPS_JD[k]=0x00;	//插入结束符号
// 	GPS_JD_NEW=(tempA-0x30)*10+(tempB-0X30);	 //经度小数部分
    return 1;
}

void GET_GPS_SIG(uchar *nbuffer)	   //截取定位的卫星数量	00-12
{
    uchar i, k;
    // $GPGGA,154549.00,3134.02922,N,12020.20288,E,1,05,1.69,29.6,M,7.1,M,,*5B
    i = 0;
    k = 0;

    while (nbuffer[i] != 0x00)
    {
        if (nbuffer[i] == ',')
        {
            k++;
        }

        i++;

        if (k == 7)
        {
            break;
        }
    }

//	k=0;
//	GPS_SIG[k]=nbuffer[i];	 k++;	i++;		//
//	GPS_SIG[k]=nbuffer[i];	 k++;	i++;		//
//	GPS_SIG[k]=0X00;		//

    GPS_SIG[0] = nbuffer[i];
    i++;
    GPS_SIG[1] = nbuffer[i];
    GPS_SIG[2] = 0X00;

    //GPS 信号个位插0
    if (GPS_SIG[1] == ',')
    {
        GPS_SIG[1] = GPS_SIG[0];
        GPS_SIG[0] = '0';
    }
}




uchar GET_GPS_ALT(uchar *nbuffer)	   //截取海拔高度度，保存到		  海拔-9999.9  ~ 99999.9
{
    uchar i, k, n;
    unsigned char TEMP[7];
    unsigned long GPS_ALT;
    uchar  GPS_ALT_ZERO;	//小于0标志


    //数据帧中提取一行
    // $GPGGA,154549.00,3134.02922,N,12020.20288,E,1,05,1.69,29.6,M,7.1,M,,*5B
    //数据解析转换处理


    GPS_ALT = 0;	 //海拔清0
    GPS_HEIGHT[0] = '-';
    GPS_HEIGHT[1] = '-';
    GPS_HEIGHT[2] = '-';
    GPS_HEIGHT[3] = '-';
    GPS_HEIGHT[4] = '-';
    GPS_HEIGHT[5] = '-';
    GPS_HEIGHT[6] = 0x00; //结束符号


    i = 0;
    k = 0;

    while (nbuffer[i] != 0x00)
    {
        if (nbuffer[i] == ',')
        {
            k++;
        }

        i++;

        if (k == 9)
        {
            break;
        }
    }

    if (nbuffer[i] == ',')
    {
        GPS_ALT_ERR = 1;    //GPS没有效定位，无海拔数据，数据无效
        GPS_ALT = 0;
        GPS_ALT_MI = 0;
        return 0;
    }

    GPS_ALT_ERR = 0;

    if (nbuffer[i] == '-') 	//海拔负数处理	//海拔小于0
    {
        GPS_ALT_ZERO = 0;	   	   //跳过符号‘-’
        i++;
    }
    else
    {
        GPS_ALT_ZERO = 1;	   //海拔大于0
    }


    n = 0;

    while (nbuffer[i] != '.')	 	//到'.'为止，忽略小数部分
    {
        TEMP[n] = nbuffer[i] - 0X30;
        n++;
        i++;

        if (n > 5)
        {
            GPS_ALT_ERR = 1; 	   //最多5个字符，超长报错
            GPS_ALT_MI = 0;
            return 0;
        }
    }

//	n=4;	   //测试
//	TEMP[0]=9; 	TEMP[1]=9;	TEMP[2]=9;	TEMP[3]=9;	TEMP[4]=9; //测试

    switch (n)
    {
        case 1:		 //海拔0-9米
            GPS_ALT = TEMP[0];
            break;

        case 2:		 //海拔10-99米
            GPS_ALT = TEMP[0] * 10 + TEMP[1];
            break;

        case 3:		 //海拔100-999米
            GPS_ALT = (long)TEMP[0] * 100 + TEMP[1] * 10 + TEMP[2];
            break;

        case 4:		 //海拔1000-9999米
            GPS_ALT = (long)TEMP[0] * 1000 + (long)TEMP[1] * 100 + TEMP[2] * 10 + TEMP[3];
            break;

        default:	 //海拔10000-99999米
            GPS_ALT = (long)TEMP[0] * 10000 + (long)TEMP[1] * 1000 + (long)TEMP[2] * 100 + TEMP[3] * 10 + TEMP[4];
            break;
    }

    if (GPS_ALT_ZERO == 0)
    {
        GPS_ALT_MI = -GPS_ALT;
    }
    else
    {
        GPS_ALT_MI = GPS_ALT;
    }

    GPS_ALT = GPS_ALT * 3.2808;	 //转换成英制

    GPS_HEIGHT[0] = GPS_ALT / 100000 % 10 + 0x30;
    GPS_HEIGHT[1] = GPS_ALT / 10000 % 10 + 0x30;
    GPS_HEIGHT[2] = GPS_ALT / 1000 % 10 + 0x30;
    GPS_HEIGHT[3] = GPS_ALT / 100 % 10 + 0x30;
    GPS_HEIGHT[4] = GPS_ALT / 10 % 10 + 0x30;
    GPS_HEIGHT[5] = GPS_ALT % 10 + 0x30;
    GPS_HEIGHT[6] = 0x00; //结束符号

    if (GPS_ALT_ZERO == 0)
    {
        GPS_HEIGHT[0] = '-';
    }

    return 1;
}


unsigned char GPS_SMART()	//是否满足智能GPS信标规则，1=OK
{
    uint DIR, TEMP;
    uchar set_time;

    if (GPS_WAIT_TIME < 10)
    {
        return 0;   //每次信标发射间隔10秒
    }

//	if (GPS_WAIT_TIME>179)	{  GPS_WAIT_TIME=0;	return 2;	}  	 	//待机状态，每隔3分钟发一条数据
    if (GPS_WAIT_TIME > ((uint)EEPROM_Buffer[0XBE] * 256 + EEPROM_Buffer[0XBF]))
    {
        GPS_WAIT_TIME = 0;
        return 2;
    }

    //待机状态，每隔3分钟发一条数据
    //--------------------------------------------------------------------------

    if (GPS_LOCKED == 0)
    {
        return 0;   //GPS没定位，不发射
    }

    if (GPS_NOW_SPEED_KNOT == 0)
    {
        return 0;   //速度=0，不发射
    }

//	if (GPS_NOW_SPEED_KNOT==1){	return 0;}  //速度=1，不发射
    if (GPS_DIR_NOTHING == 0)
    {
        return 0;   //无航向数据，不发射
    }


    //--------------------------------------------------------------------------
    //GPS模式=智能信标模式
    //---智能GPS信标条件之一 ，方向超过左右25度，发射信标		//提取的GPS方向0-359度

    //求当前角度与前次角度最小角度值（绝对值）
    //1、比较当前角度与前次角度哪个值大,2值相等则忽略以下计算
    //2、计算 （大值角度-小值角度	）
    //3、如果 （大值角度-小值角度	）>180度，则最小角度插值=360- （大值角度-小值角度	）
    //4、如果 （大值角度-小值角度	）<=180度，则最小角度插值=（大值角度-小值角度	）
    if (GPS_NOW_DIR > GPS_OLD_DIR) 	 //当前角度值>前次角度值
    {
        TEMP = (GPS_NOW_DIR - GPS_OLD_DIR);
    }
    else							  //当前角度值<前次角度值 ,即前次角度值>当前角度值
    {
        TEMP = (GPS_OLD_DIR - GPS_NOW_DIR);
    }

    if (TEMP > 180)
    {
        DIR = (360 - TEMP);
    }
    else
    {
        DIR = TEMP;
    }

    if (DIR > 25)	//绝对最小角度偏差大于25度，允许发射
    {
        GPS_WAIT_TIME = 0;
        GPS_OLD_DIR = GPS_NOW_DIR; //更新前次角度值
        return 1;	//08=GPS连接并成功解析,绝对最小角度偏差大于25度，允许发射
    }

    //--------------------------------------------------------------------------
    //--------------------------------------------------------------------------
    switch ( EEPROM_Buffer[0x04])   	//设置 0=FAST  1=MID1  2=MID2 3=SLOW
    {
        case 1:
            set_time = 20;
            break;

        case 2:
            set_time = 40;
            break;

        case 3:
            set_time = 60;
            break;

        case 4:
            set_time = 90;
            break;

        case 5:
            set_time = 120;
            break;


        default:	//0
            set_time = 20;
            break;
    }

    if 	(GPS_WAIT_TIME > set_time)  //平均间隔20秒发射一次
//	if 	(GPS_WAIT_TIME>20)    //平均间隔20秒发射一次
    {
        GPS_WAIT_TIME = 0;
        GPS_OLD_DIR = GPS_NOW_DIR; //更新前次角度值
        return 1;	//允许发射
    }

    return 0;	 //不做任何处理
}

//void speed_haili_txt()
//{
//	tostring(GPS_NOW_SPEED_KNOT);	  //海里速度转成文本
//	GPS_NOW_SPEED_KNOT_TXT[0]=bai;
//	GPS_NOW_SPEED_KNOT_TXT[1]=shi;
//	GPS_NOW_SPEED_KNOT_TXT[2]=ge;
//	GPS_NOW_SPEED_KNOT_TXT[3]=0;
//
//}

uchar GET_GPS_SPEED(uchar *nbuffer)	//求当前速度GPS_NOW_SPEED_KNOT,返回0=无速度 1=有速度
{
    unsigned char TEMP[3];
    uchar i, n;
//--------------------------------------------------------------------------
//求当前速度、航向
//---------------------------------------------------------
//$GPRMC,032149.365,A,3134.0372,N,12020.2073,E,0.00,345.47,300113,,,A*66
//$GPRMC,000000.000,V,0000.0000,S,00000.0000,W,0.00,0.00,220899,,,A*7E
//--------------------------------------------------------------------------
//$GPRMC,145240.00,A,3134.00679,N,12020.21635,E,1.768,159.36,180813,,,A*64
//$GPRMC,145241.00,A,3134.00716,N,12020.21622,E,1.467,,180813,,,A*71
//$GPRMC,145006.00,A,3134.01359,N,12020.21880,E,1.142,,180813,,,A*7A
//注意这条航向数据，可能造成计算溢出，影响后续数据
//--------------------------------------------------------------------------
//$GPRMC,143350.00,A,3134.03522,N,12020.20752,E,1.195,143.22,101013,,,A*66

    i = 0;
    n = 0;

    while (nbuffer[i] != 0x00)
    {
        if (nbuffer[i] == ',')
        {
            n++;
        }

        i++;

        if (n == 7)
        {
            break;   //跳过7个,号
        }
    }

    if (nbuffer[i] == ',') 	 //没有速度数据，不做任何处理
    {
        GPS_NOW_SPEED_KNOT = 0;
        GPS_SPEED_KM = 0;

        return 0;
    }

    //--------------------------------------------------------------------------

    n = 0;

    while (nbuffer[i] != '.')
    {
        TEMP[n] = nbuffer[i];		   //到'.'为止，忽略小数部分
        i++;
        n++;
    }

    switch (n)
    {
        case 2:		 //速度10-99海里
            GPS_NOW_SPEED_KNOT = ((TEMP[0] - 0x30) * 10 + (TEMP[1] - 0x30)) ;	 //GPS当前速度，0-255海里
            break;

        case 3:		 //速度100-255海里
            GPS_NOW_SPEED_KNOT = ((TEMP[0] - 0x30) * 100 + (TEMP[1] - 0x30) * 10 + (TEMP[2] - 0x30));	 //GPS当前速度，0-255海里
            break;

        default:	 //速度0-9海里
            GPS_NOW_SPEED_KNOT = (TEMP[0] - 0x30);	 //GPS当前速度，0-255海里
            break;
    }


    GPS_SPEED_KM = GPS_NOW_SPEED_KNOT * 1.852;	//转换成米,0.1KM

//	speed_haili_txt();


//	if (n==1){TEMP[2]=TEMP[0];  TEMP[1]='0';	  TEMP[0]='0'; }  //速度0-9海里
//	if (n==2){TEMP[2]=TEMP[1];	TEMP[1]=TEMP[0];  TEMP[0]='0'; }  //速度10-99海里
//	GPS_NOW_SPEED_KNOT=((TEMP[0]-0x30)*100+ (TEMP[1]-0x30)*10+(TEMP[2]-0x30));	 //GPS当前速度，0-255海里
    return 0;
}


uchar GET_GPS_DIR(uchar *nbuffer)	//求当前航向GPS_NOW_DIR,返回0=无航向 1=有航向
{
    unsigned char TEMP[3];
    uchar i, n;
    //--------------------------------------------------------------------------
    //求当前速度、航向
    //静止时，无航向数据，可能造成计算溢出，影响后续数据
    //$GPRMC,145006.00,A,3134.01359,N,12020.21880,E,1.142,,180813,,,A*7A
    //--------------------------------------------------------------------------
    //有航向数据
    //$GPRMC,143350.00,A,3134.03522,N,12020.20752,E,1.195,143.22,101013,,,A*66

    i = 0;
    n = 0;

    while (nbuffer[i] != 0x00)
    {
        if (nbuffer[i] == ',')
        {
            n++;
        }

        i++;

        if (n == 8)
        {
            break;   //跳过8个,号
        }
    }


    if (nbuffer[i] == ',')
    {
        //GPS_NOW_DIR=0; 		 	//当没有航向数据时，保留最后的航向
        GPS_DIR_NOTHING = 0;
        return 0; 	//没有航向数据，不做任何处理
    }

    //--------------------------------------------------------------------------

    n = 0;

    while (nbuffer[i] != '.')
    {
        TEMP[n] = nbuffer[i];			   //到'.'为止，忽略小数部分
        i++;
        n++;
    }

//	n=3;
//	TEMP[0]='9'; TEMP[1]='8';TEMP[2]='7';

    switch (n)
    {
        case 2:		 //GPS当前方向10-99度
            GPS_NOW_DIR = ((TEMP[0] - 0x30) * 10 + (TEMP[1] - 0x30)) ;
            break;

        case 3:		 //GPS当前方向100-359度
            GPS_NOW_DIR = ((TEMP[0] - 0x30) * 100 + (TEMP[1] - 0x30) * 10 + (TEMP[2] - 0x30));
            break;

        default:	 //GPS当前方向0-9度
            GPS_NOW_DIR = (TEMP[0] - 0x30);	 //GPS当前速度，0-255海里
            break;
    }

    GPS_DIR_NOTHING = 1;

    tostring(GPS_NOW_DIR);	  //航向
    GPS_NOW_DIR_TXT[0] = bai;
    GPS_NOW_DIR_TXT[1] = shi;
    GPS_NOW_DIR_TXT[2] = ge;
    GPS_NOW_DIR_TXT[3] = 0;

    return 1;
}




void QUICK_SET_FIXED()	  //是否快速修改固定站经纬度
{
    uint i;

    if (QUICK_SET_EN == 1)
    {
        QUICK_SET_EN = 0;

        //把当前GPS经纬度设成固定站纬度和经度
        for(i = 0; i < 8; i++)
        {
            EEPROM_Buffer[0x20 + i] = GPS_WD[i];
        }

        for(i = 0; i < 9; i++)
        {
            EEPROM_Buffer[0x30 + i] = GPS_JD[i];
        }

        EEPROM_UPDATA();

//	 UART2_SendString(GPS_WD);	UART2_SendString(" ");	 	 UART2_SendString(GPS_JD);	UART2_SendString("\r\n");
//		UART2_SendString("QUICK SETUP\r\n");


        UART4_TX_EEROM();  //刷新菜单设置

        UART2_SendString("HELLO");	 //刷新蓝牙设置

        for(i = 0; i < 512; i++)
        {
            UART2_SendData(EEPROM_Buffer[i]);	 	   //复制参数到缓冲区
        }

    }
}




void UART3_RX_GPS()	  //调试GPS数据
{
    if (sum_gps() != 1)
    {
        return;   //校验GPS数据，正确则继续处理GPS数据
    }

    GET_GPS_LOCK_STATUS(UART3_GPRMC_DATA);		//检查定位状态、截取经纬度，航向、速度、保存到GPS_DATA

    if (GPS_LOCKED == 0)
    {
        QUICK_SET_EN = 0;	   //GPS未锁定界面
        return;
    }


    //如果GPS锁定，解析数据，发送信标
    GET_GPS_TIME(UART3_GPRMC_DATA);//求时间
    GET_GPS_DATE(UART3_GPRMC_DATA);//截取GPS日期

    GET_GPS_LONG_LAT(UART3_GPRMC_DATA);	  //GPRMC截取经纬度，保存到GPS_DATA,
    GET_GPS_SPEED (UART3_GPRMC_DATA);	  //GPRMC求速度
    GPS_DIR_NOTHING = GET_GPS_DIR (UART3_GPRMC_DATA); //GPRMC求航向
//	if (GPS_DIR_NOTHING==0){return;}  //没有航向数据,不发射

    GET_GPS_ALT(UART3_GPGGA_DATA); //GPGGA中截取海拔高度
    GET_GPS_SIG(UART3_GPGGA_DATA);	//GPGGA中截取定位的卫星数量	00-12


    GPS_FORMAT_A();   //度分分    03134.3795N 12019.8827E  四舍五入 转成3134.38N 12019.88E
    //==============================================
    QUICK_SET_FIXED();	  //先四舍五入处理，是否快速修改固定站经纬度
    //==============================================


//	disp_gps_lock();	   //GPS锁定界面
    //==============================================
    if (EEPROM_Buffer[0X2A] == 1)	 // 移动站模式
    {
        if (EEPROM_Buffer[0x05] != 0)		//队列模式
        {
            if (TIME_GPS_SEC == EEPROM_Buffer[0x06])
            {
                BEACON_GPS_TXD();
            }
        }

        SMART_MODE = 0;

        if (EEPROM_Buffer[0x04] != 0)
        {
            SMART_MODE = GPS_SMART() ;

            if (SMART_MODE != 0)
            {
                BEACON_GPS_TXD();     //智能信标
            }
        }
    }
}















