//求2点经纬度之间的距离
#include <math.h>


#include "STC8A8K64D4.H"
#include "STC_EEPROM.H"
#include "PUBLIC_BUF.H"

#include "GPS_JULI.H"
#include "GPS2.H"
#include "KISS_Analysis.H"
#include "tostring.H"

#include "UART2.H"


#define   ulong unsigned long


#define pi 3.1415926
#define EARTH_RADIUS 6378.137


double GetDistance(double lat1, double lng1, double lat2, double lng2) ;
float GetAngle(float x1, float y1, float x2, float y2) ;
//void GPS_DMM_D(uchar mode); //文本度分分转度

double my_lat, my_lng, you_lat, you_lng;


double Distance_AB;


double rad(double d)
{
    return (d * pi / 180.0);
}



double GET_WD(uchar *lat)	  	//转换纬度和经度,文本,度分分  转  度
{
    uchar i;
    double lat_degree, lat_min; //度、分
    unsigned char temp[10];

    for (i = 0; i < 8; i++)
    {
        temp[i] = lat[i];
    }

    lat_min = (double)(temp[2] - 0x30) * 10 + (double)(temp[3] - 0x30) + (double)(temp[5] - 0x30) * 0.1 + (double)(temp[6] - 0x30) * 0.01;
    lat_degree =	(double)(temp[0] - 0x30) * 10 + (double)(temp[1] - 0x30) +	(lat_min / 60) ;

    //北纬N 为正，南纬 S 为负，东经E 为正，西经 W 为负，
    if (temp[7] == 'S')
    {
        lat_degree = -lat_degree;
    }

    return  lat_degree;
}

double GET_JD(uchar *lng)	 	//转换纬度和经度,文本,度分分  转  度
{
    uchar i;
    double lng_degree, lng_min; //度、分
    unsigned char temp[10];

    for (i = 0; i < 9; i++)
    {
        temp[i] = lng[i];
    }

    lng_min = (double)(temp[3] - 0x30) * 10 + (double)(temp[4] - 0x30) + (double)(temp[6] - 0x30) * 0.1 + (double)(temp[7] - 0x30) * 0.01;
    lng_degree = (double)(temp[0] - 0x30) * 100 + (double)(temp[1] - 0x30) * 10 + (double)(temp[2] - 0x30) +	(lng_min / 60) ;

    //北纬N 为正，南纬 S 为负，东经E 为正，西经 W 为负，
    if (temp[8] == 'W')
    {
        lng_degree = -lng_degree;
    }

    return 	lng_degree;
}

//度分抓换为度
//A=31.3401N  120.2021E   即31.566833   120.336833
//B=31.3400N  120.2020E	  即31.566666   120.336666

//AB=24.3米

//从地球赤道平面向北量度的纬度。符号N，北纬0°～90°。S,南纬 ，0-90度
//东经用“E”表示，西经用“W”表示。范围0-180度
//经纬度转换为度
//1度等于60分，1分等于60秒
//纬度 temp	   //经度  longitude
//北纬N 为正，南纬 S 为负，东经E 为正，西经 W 为负，
//GetDistance(A点纬度,A点经度，B点纬度,B点经度)

double GetDistance(double lat1, double lng1, double lat2, double lng2)
{
    double radLat1 = rad(lat1);
    double radLat2 = rad(lat2);
    double a = radLat1 - radLat2;
    double b = rad(lng1) - rad(lng2);

    double s = 2 * asin(sqrt(pow(sin(a / 2), 2) + cos(radLat1) * cos(radLat2) * pow(sin(b / 2), 2)));

    s = s * EARTH_RADIUS;
    //	round方法，表示“四舍五入”，算法为Math.floor(x+0.5),即将原来的数字加上0.5后再向下取整
    //  s = Math.Round(s * 10000) / 10000;	 //
//	s = floor((s * 10000)+0.5) / 10000;		 //	  单位KM
//	s = floor((s * 10000)+0.5) / 10000;	     //	  单位1KM	  1000米
//	s = floor((s * 10000)+0.5) / 1000 ;		 //	  单位0.1KM	  100米
//	s = floor((s * 10000)+0.5) / 100 ;		 //	  单位0.01KM  10米
    s = floor((s * 10000) + 0.5) / 10 ;		 //	  单位0.001KM  1米

    return s;	//0.02426 KM 四舍五入约等于	24.3米
}


double GET_JULI(uchar *a_lat, uchar *a_lng, uchar *b_lat, uchar *b_lng )
{
    // double JL,DF;
    //--------------------------------------------

//  	my_lat = GET_WD("3134.00N");	//转换A点纬度,文本,度分分  转  度
//
//	//--------------------------------------------
//	my_lng = GET_JD("12020.20W"); //转换A点经度,文本,度分分  转  度
//	//--------------------------------------------
//   	you_lat= GET_WD("3134.00N"); //转换B点纬度,文本,度分分  转  度
//	//--------------------------------------------
//	you_lng= GET_JD("12020.20W");	//转换B点经度,文本,度分分  转  度
//
//
//  JL=  GetDistance(my_lat, my_lng, you_lat, you_lng);
//   DF=JL;

//	UART2_SendString(a_lat); 	UART2_SendString(a_lng);
//	UART2_SendString(b_lat);	UART2_SendString(b_lng);


    my_lat = GET_WD(a_lat);	//转换A点纬度,文本,度分分  转  度
    //--------------------------------------------
    my_lng = GET_JD(a_lng); //转换A点经度,文本,度分分  转  度
    //--------------------------------------------
    you_lat = GET_WD(b_lat); //转换B点纬度,文本,度分分  转  度
    //--------------------------------------------
    you_lng = GET_JD(b_lng);	//转换B点经度,文本,度分分  转  度
    //--------------------------------------------
//	JL= GetDistance(my_lat, my_lng,you_lat, you_lng)*10;	  //精度100米，0.0243 KM 约等于	0.0KM
//	JL= GetDistance(my_lat, my_lng,you_lat, you_lng)*100;	  //精度10米，0.0243 KM 约等于	0.02KM
//	JL= GetDistance(31.566833, 120.336833,31.566666, 120.336666)*1000;	  //精度1米，0.0243 KM 约等于0.024KM
//  JL= GetDistance(31.566833, 120.336833,31.566666, 120.336666)*10000;	  //精度0.1米，0.0243 KM 约等于	24.3米

//   JL= GetDistance(-31.114921, -120.336833,-31.114921, -120.336666);
//
//   DF=JL;


    return GetDistance(my_lat, my_lng, you_lat, you_lng);	  //精度1米，0.0243 KM 约等于
}







//void GPS_DMM_D(uchar mode) //文本度分分转度
//{
//	//MODE=0 求固定站和对方2地距离
//	//MODE=1 求移动站和对方的2地距离
//	//MODE=2 求GPS和固定站之间的距离
//
//	//转换A点纬度和经度,文本,度分分  转  度
//  	if (mode==1)   //0=固定站，设定的经纬度  1=移动站,GPS经纬度
//	{ my_lat = GET_WD(GPS_WD);  //自己的GPS纬度
//	  my_lng = GET_JD(GPS_JD);  //自己的GPS经度
//	}
//	else
//	{ my_lat = GET_WD(EEPROM_Buffer+0x20);//固定站纬度
//	  my_lng = GET_JD(EEPROM_Buffer+0x30);//固定站经度
//	}
//	//--------------------------------------------
//	 //转换B点纬度和经度,文本,度分分  转  度
//	if (mode==2)
//	{ you_lat= GET_WD(GPS_WD);//自己的GPS纬度
//	  you_lng= GET_JD(GPS_JD);//自己的GPS经度
//	}
//	else
//	{ you_lat= GET_WD(UI_WD);	//求对方移动纬度
//	  you_lng= GET_JD(UI_JD);	  //求对方移动经度
//	}
//	//--------------------------------------------
//}
//
//





void GET_AB_JULI(uchar mode)
{
    unsigned int JL;
    unsigned char i;	//	unsigned char  n;

    //--------------------------------------------
    //MODE=0 求固定站和对方2地距离,精度1米
    if (mode == 0)
    {
        Distance_AB = GET_JULI(EEPROM_Buffer + 0x20, EEPROM_Buffer + 0x30, UI_WD, UI_JD);
    }

    //MODE=1 求移动站和对方的2地距离,精度1米
    if (mode == 1)
    {
        Distance_AB = GET_JULI(GPS_WD, GPS_JD, UI_WD, UI_JD);
    }

    //MODE=2 求GPS和固定站之间的距离,精度1米
    if (mode == 2)
    {
        Distance_AB = GET_JULI(EEPROM_Buffer + 0x20, EEPROM_Buffer + 0x30, GPS_WD, GPS_JD);
    }


//	Distance_AB= GET_JULI("3134.32N", "12020.01E","3134.32N", "12020.01E");		  //wd 0.01 =18.2米		JD 15.5米

    for(i = 0; i < 8; i++)
    {
        UI_JULI[i] = 0x00;
    }

    if (Distance_AB < 10000)	 //小于10,000米，显示米
    {
        JL = (uint)(Distance_AB);
        tostring(JL) ;
        UI_JULI[0] = '*';
        UI_JULI[1] = qian;
        UI_JULI[2] = '.';
        UI_JULI[3] = bai;
        UI_JULI[4] = shi;
        UI_JULI[5] = ge;
        UI_JULI[6] = 0x00;


//		for(i=0;i<4;i++) 	  //0替换成空格
//		{
//		  if (UI_JULI[i+1]=='0'){UI_JULI[i+1]=' ';}else{break;}
//		}

    }
    else
    {
        JL = (uint)(Distance_AB / 100); //精度调整10米，0.0243 KM 约等于	24米

        tostring(JL) ;

        UI_JULI[0] = wan;
        UI_JULI[1] = qian;
        UI_JULI[2] = bai;
        UI_JULI[3] = shi;
        UI_JULI[4] = '.';
        UI_JULI[5] = ge;
        UI_JULI[6] = 0x00;

        //	n=0;
        //	while(UI_JULI[0]=='0')	 //首位0消隐
        //	{
        //	for(i=0;i<7;i++) 	{ UI_JULI[i]=	UI_JULI[i+1]; }
        //	n++; if (n>2){break;}	//最多消隐3位，
        //	}

        for(i = 0; i < 3; i++) 	 //0替换成空格
        {
            if (UI_JULI[i] == '0')
            {
                UI_JULI[i] = ' ';
            }
            else
            {
                break;
            }
        }

    }

// 	n=0;
//	while(UI_JULI[0]==' ')	 //首位0消隐,填充空格
//	{
//	for(i=0;i<7;i++) 	{ UI_JULI[i]=	UI_JULI[i+1]; }
//	n++; if (n>2){break;}	//最多消隐3位，
//	}
}







float GetAngle(float x1, float y1, float x2, float y2)
{
    /*float angle ;
    float deltaX = x2 - x1 ;
    float deltaY = y2 - y1 ;
    float distance = sqrt(deltaX * deltaX + deltaY * deltaY) ;
    angle = acosf(deltaX/distance) ;
    if(deltaY>0)
    {
    	angle = 3.14 + (3.14 - angle) ;
    }*/


    float angle ;
    angle = atan2(x2 - x1, y2 - y1) ;
    angle = -(angle * 180.0f) / 3.141592f ;
    angle += 90.0f ;

    if(angle < 0.0f)	angle += 360.0f ;

    return angle ;
}


float GET_Angle(uchar *a_lat, uchar *a_lng, uchar *b_lat, uchar *b_lng )
{
    //--------------------------------------------
    my_lat = GET_WD(a_lat);	//转换A点纬度,文本,度分分  转  度
    //--------------------------------------------
    my_lng = GET_JD(a_lng); //转换A点经度,文本,度分分  转  度
    //--------------------------------------------
    you_lat = GET_WD(b_lat); //转换B点纬度,文本,度分分  转  度
    //--------------------------------------------
    you_lng = GET_JD(b_lng);	//转换B点经度,文本,度分分  转  度
    //--------------------------------------------
    return GetAngle(my_lat, my_lng, you_lat, you_lng);	  //返回0-359度角度
}



uint GET_AB_Angle(uchar mode)	  //求2点经纬度的相对正北的水平角度
{
    uint Angle;

    //--------------------------------------------
    //MODE=0 求固定站和对方2地，角度
    if (mode == 0)
    {
        Angle = (uint) GET_Angle(EEPROM_Buffer + 0x20, EEPROM_Buffer + 0x30, UI_WD, UI_JD);
    }

    //MODE=1 求移动站和对方的2地，角度
    if (mode == 1)
    {
        Angle = (uint)GET_Angle(GPS_WD, GPS_JD, UI_WD, UI_JD);
    }

    //MODE=2 求GPS和固定站之间的，角度
    if (mode == 2)
    {
        Angle = (uint) GET_Angle(GPS_WD, GPS_JD, EEPROM_Buffer + 0x20, EEPROM_Buffer + 0x30);
    }



    if (Angle > 359)
    {
        Angle = 0;
    }

    return Angle;
}













uint GET_EL(float Distance_AB_MI, float MY_ALT_MI,  float Target_ALT_MI   )	 //求2点 仰角  0-89度
{
//	UART0_DEBUG(	(uint)	(atan(3)*180/PI )	);
//  UART0_DEBUG(	(uint)	(atan(0)*180/PI )	);
    if (Target_ALT_MI < MY_ALT_MI)
    {
        return 0;   //如果信标高度比固定/移动站高度低，则返回仰角0
    }

    return     (uint)(atan((Target_ALT_MI - MY_ALT_MI) / Distance_AB_MI) * 180 / 3.1415926)	;
}

uint GET_AB_EL(uchar mode)	   //求2点 仰角  0-89度
{
    uint Angle;

    //--------------------------------------------
    //MODE=0 求固定站和对方2地，角度,距离转成整数
    if (mode == 0)
    {
        Angle = GET_EL((ulong)Distance_AB, EEPROM_Buffer[0x0129] * 256 + EEPROM_Buffer[0x012A], UI_ALT_MI);
    }

    //MODE=1 求移动站和对方的2地，角度
    if (mode == 1)
    {
        Angle = GET_EL((ulong)Distance_AB, GPS_ALT_MI, UI_ALT_MI);
    }

    //MODE=2 求GPS和固定站之间的，角度
    if (mode == 2)
    {
        Angle = GET_EL((ulong)Distance_AB, GPS_ALT_MI, EEPROM_Buffer[0x0129] * 256 + EEPROM_Buffer[0x012A]);
    }

    if (Angle > 90)
    {
        Angle = 90;
    }



    return Angle;
}




uint GET_AB_POINT_DIR(uint A_POINT, uint GPS_DIR)		//相对车头方向
{
    if  (A_POINT > GPS_NOW_DIR)
    {
        return A_POINT - GPS_DIR;
    }
    else
    {
        return 360 - (GPS_DIR - A_POINT);
    }
}

void angle_to_txt(uint angle, uchar *p)
{
    uint  dat;
    dat = angle;

    if (EEPROM_Buffer[0x0012B] == 0)
    {
        if (dat < 23)
        {
            *(p + 0) = 'N';  	   //NW点方位
            *(p + 1) = ' ';
            *(p + 2) = 0;
            return;
        }

        if (dat < 68)
        {
            *(p + 0) = 'N';
            *(p + 1) = 'E';
            *(p + 2) = 0;
            return;
        }

        if (dat < 113)
        {
            *(p + 0) = 'E';
            *(p + 1) = ' ';
            *(p + 2) = 0;
            return;
        }

        if (dat < 158)
        {
            *(p + 0) = 'S';
            *(p + 1) = 'E';
            *(p + 2) = 0;
            return;
        }

        if (dat < 203)
        {
            *(p + 0) = 'S';
            *(p + 1) = ' ';
            *(p + 2) = 0;
            return;
        }

        if (dat < 248)
        {
            *(p + 0) = 'S';
            *(p + 1) = 'W';
            *(p + 2) = 0;
            return;
        }

        if (dat < 293)
        {
            *(p + 0) = 'W';
            *(p + 1) = ' ';
            *(p + 2) = 0;
            return;
        }

        if (dat < 338)
        {
            *(p + 0) = 'N';
            *(p + 1) = 'W';
            *(p + 2) = 0;
            return;
        }

        if (dat < 360)
        {
            *(p + 0) = 'N';
            *(p + 1) = ' ';
            *(p + 2) = 0;
            return;
        }

        *(p + 0) = '-';
        *(p + 1) = '-';
        *(p + 2) = 0;
        return;
    }

    if (EEPROM_Buffer[0x0012B] == 1)
    {
        dat = dat / 30.0 + 0.5;			 //00-12点方位
        *(p + 0) = dat / 10 + 0x30;
        *(p + 1) = dat % 10 + 0x30;
        *(p + 2) = 0;
        return;
    }

    if (EEPROM_Buffer[0x0012B] == 2)
    {
        dat = dat / 10.0 + 0.5;			 //00-36点方位
        *(p + 0) = dat / 10 + 0x30;
        *(p + 1) = dat % 10 + 0x30;
        *(p + 2) = 0;
        return;
    }

    *(p + 0) = '-';
    *(p + 1) = '-';
    *(p + 2) = 0; 	 //设置错误，显示--，重新初始化
    EEPROM_Buffer[0x0012B] = 1;
    EEPROM_UPDATA();  	//如果没初始化则更新设置

}