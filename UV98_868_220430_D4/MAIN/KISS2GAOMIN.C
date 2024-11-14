#include "STC8A8K64D4.H"
#include "STC_EEPROM.H"
#include "KISS2GAOMIN.H"
#include "BEACON.H"

#include "KISS_Analysis.H"

#include "GPS2.H"

unsigned char GPWPL_BUF[60] = 0;		//GPWPL航点数据缓存  长度<50



//校验  $ 和 * 直接的全部字符的异或值 ，不含$ 和 * ,速率9600
void UI_TO_GPWPL()		//KISS数据转成GPWPL航点格式
{
    unsigned char  GPWPL[10] = {"$GPWPL,"};
    unsigned char i, len, crc_hex, crc_h, crc_l;

//GPWPL_BUF[0]=0;

    len = 0;

    for (i = 0; i < 7; i++)
    {
        GPWPL_BUF[len] = GPWPL[i];	    //插入字头
        len++;
    }


    //插入纬度
    for (i = 0; i < 7; i++)
    {
        GPWPL_BUF[len] = UI_WD[i];
        len++;
    }

    GPWPL_BUF[len] = ',';
    len++;
    GPWPL_BUF[len] = UI_WD_DIR;
    len++;
    GPWPL_BUF[len] = ',';
    len++;

    //插入经度
    for (i = 0; i < 8; i++)
    {
        GPWPL_BUF[len] = UI_JD[i];
        len++;
    }

    GPWPL_BUF[len] = ',';
    len++;
    GPWPL_BUF[len] = UI_JD_DIR;
    len++;
    GPWPL_BUF[len] = ',';
    len++;




    for (i = 0; i < 9; i++) 	 //插入呼号和SSID  最长9位
    {
        if (UI_CALL[i] == 0x00)
        {
            break;
        }

        GPWPL_BUF[len] = UI_CALL[i];
        len++;
    }


//	i=0;
//	while(UI_CALL[i]!=0x00) { GPWPL_BUF[len]=UI_CALL[i];	len++;	i++; 	} 	//插入呼号和SSID

//	for (i=0;i<6;i++)    {   	 SN_RX_BUFFER[len]= UI_CALL[i];  len++;	 }
//	SN_RX_BUFFER[len]= '-';	 len++;
//	for (i=0;i<2;i++)   	{   SN_RX_BUFFER[len]= UI_SSID[i];  len++;	 } 	 	//插入呼号
    GPWPL_BUF[len] = '*';
    len++;

    //计算并插入校验值
    crc_hex = 0;

    for (i = 1; i < len - 1; i++)
    {
        crc_hex = crc_hex ^ GPWPL_BUF[i];   //	//校验  $ 和 * 直接的全部字符的异或值 ，不含$ 和 *
    }

    crc_h = crc_hex / 16;

    if (crc_h < 10)
    {
        GPWPL_BUF[len] = crc_h + 0x30;
        len++;
    }
    else
    {
        GPWPL_BUF[len] = crc_h + 0x37;
        len++;
    }

    crc_l = crc_hex % 16;

    if (crc_l < 10)
    {
        GPWPL_BUF[len] = crc_l + 0x30;
        len++;
    }
    else
    {
        GPWPL_BUF[len] = crc_l + 0x37;
        len++;
    }


    //插入换行符、结束符
    GPWPL_BUF[len] = '\r';
    len++;
    GPWPL_BUF[len] = '\n';
    len++;	//补上换行符

    GPWPL_BUF[len] = 0x00;	  		//补上结束符
    //以上为数据格式转换

    return;
}


//
//uint CHECK_KISS_IDX()
//{
//  	for(i=0;i<KISS_LEN;i++)
//	{
//		if ((KISS_DATA[i]==0x03)&&(KISS_DATA[i+1]==0xF0))
//		{		return i-1;	   }
//	}
//	return 0;
//}

//校验  $ 和 * 直接的全部字符的异或值 ，不含$ 和 * ,速率9600
//void KISS_TO_GPWPL()		//KISS数据转成GPWPL航点格式
//{
//	unsigned char code GPWPL[]={"$GPWPL,"};
//	unsigned char i,len,w,k,asc,ssid,crc_hex,crc_h,crc_l;
////unsigned char code test[]={"GPWPL,3100.00,N,12100.00,E,BH4TDV-9"};
////unsigned char code test[]={"$GPWPL,3216.11,N,10532.02,E,BG8EJT-13*"};
////$GPWPL,2305.35,N,10935.62,E,BD7RS-10*5F
////$GPWPL,3216.11,N,10532.02,E,BG8EJT-13*03
////$GPWPL,3447.30,N,11342.70,E,BG6JJI-13*1B
////$GPWPL,2502.96,N,10242.02,E,BG8SXC-10*1F
////$GPWPL,3949.00,N,11812.01,E,BG3MCK-13*1A
//
//
//// 0x82, 0xA0, 0x9E, 0xA8, 0x86, 0x62, 0xE0, 				//	目标地址 +SSID
//// 0x84, 0x90, 0x68, 0xA8, 0x88, 0xAC, 0xF2, 				//	源地址	 +SSID
//// 0xAE, 0x92, 0x88, 0x8A, 0x62, 0x40, 0x63, 				//	路径	 +SSID
//// 0x03, 0xF0, 											//	03f0
//// 0x21, 													//	类型
//// 0x33, 0x31, 0x30, 0x30, 0x2E, 0x30, 0x30, 0x4E, 		//	经纬度
//// 0x2F, 													//	分隔符	"/"
//// 0x31, 0x32, 0x31, 0x30, 0x30, 0x2E, 0x30, 0x30, 0x45, 	//	经纬度
//// 0x3E, 													//	图标类型
//// 0x20, 0x30, 0x37, 0x2E, 0x38, 0x56, 0x20, 0x32, 0x31, 0x43, 0x20, 0x6F, 0x74,		 //信息
//
//
////	for (i=0;i<128;i++) {	GPWPL_BUF[i]= 0x00;}   	//初始清0
//   	 GPWPL_BUF[0]=0;
//
////	 w=CHECK_KISS_IDX();
////	 if (w==0){return;}
////27793
//
////	w=13;
////	for (k=0;k<6;k++)		 //最多6个路径
////	{
////			 if ((KISS_DATA[w] & 0x01) == 1)	 //路径结束符号
////			 {
//				len=0;
//				for (i=0;i<7;i++)   {	GPWPL_BUF[len]=GPWPL[i];	 len++;}	  //插入字头
//
//
//				//插入经纬度
//				for (i=w+4;i<w+11;i++)   {	GPWPL_BUF[len]= KISS_DATA[i];	 len++;}
//
//				GPWPL_BUF[len]= ',';  len++;
//				GPWPL_BUF[len]=  KISS_DATA[w+11];  len++;
//				GPWPL_BUF[len]= ',';  len++;
//
//				for (i=w+13;i<w+21;i++)   {	GPWPL_BUF[len]= KISS_DATA[i];	 len++;}
//
//				GPWPL_BUF[len]= ',';  len++;
//				GPWPL_BUF[len]=  KISS_DATA[w+21];  len++;
//				GPWPL_BUF[len]= ',';  len++;
//
//
//
//				//插入呼号
//
//				for (i=7;i<13;i++)
//				{  	asc= ((KISS_DATA[i])  >> 1);	if (asc !=' ')	  {	 GPWPL_BUF[len]= asc;  len++;	}  	 }
//
//				//插入SSID
//				ssid=	 ((KISS_DATA[13] & 0x1E)>>1) ;
//				if (ssid>0)
//				{	GPWPL_BUF[len]= '-';	 len++;
//
//					if (ssid<10)
//					{ GPWPL_BUF[len]=ssid+0x30;  	len++;  }	  //转换成ASCII
//					else
//					{ GPWPL_BUF[len]=ssid/10+0x30;  len++;	GPWPL_BUF[len]=ssid%10+0x30;	 len++;	}
//
//				}
//
//				   GPWPL_BUF[len]= '*';	 len++;
//
//					 //计算并插入校验值
//				crc_hex=0;
//				for (i=1;i<len-1;i++)   { crc_hex=crc_hex^GPWPL_BUF[i];}	//	//校验  $ 和 * 直接的全部字符的异或值 ，不含$ 和 *
//
//				crc_h=crc_hex/16;
//				if (crc_h<10) 			{ GPWPL_BUF[len]= crc_h+0x30;	 len++; }
//				else		 			{ GPWPL_BUF[len]= crc_h+0x37;	 len++;}
//
//				crc_l=crc_hex%16;
//				if (crc_l<10)  			{ GPWPL_BUF[len]= crc_l+0x30;	 len++; }
//				else		 			{ GPWPL_BUF[len]= crc_l+0x37;	 len++;  }
//
//				//插入换行符、结束符
//				GPWPL_BUF[len]= '\r';   len++;	GPWPL_BUF[len]= '\n';	  len++;	//补上换行符
//
//				GPWPL_BUF[len]= 0x00;	  		//补上结束符
//				//以上为数据格式转换
//
//			return;
////			 }
////	w=w+7;
////	}
//}




//校验  $ 和 * 直接的全部字符的异或值 ，不含$ 和 * ,速率9600
void MY_BEACON_TO_GPWPL()		//自己的信标直接转成GPWPL航点格式
{
    unsigned char code GPWPL[] = {"$GPWPL,"};
    unsigned char i, len, n, crc_hex, crc_h, crc_l;
//unsigned char code test[]={"GPWPL,3100.00,N,12100.00,E,BH4TDV-9"};
//unsigned char code test[]={"$GPWPL,3216.11,N,10532.02,E,BG8EJT-13*"};
//$GPWPL,2305.35,N,10935.62,E,BD7RS-10*5F
//$GPWPL,3216.11,N,10532.02,E,BG8EJT-13*03
//$GPWPL,3447.30,N,11342.70,E,BG6JJI-13*1B
//$GPWPL,2502.96,N,10242.02,E,BG8SXC-10*1F
//$GPWPL,3949.00,N,11812.01,E,BG3MCK-13*1A

    len = 0;

    for (i = 0; i < 7; i++)
    {
        GPWPL_BUF[len] = GPWPL[i];	    //插入字头
        len++;
    }

    if (EEPROM_Buffer[0X2A] == 1) //0=固定站，设定的经纬度  1=移动站,GPS经纬度
    {
        //站点纬度				   1=移动站
        for(i = 0; i < 7; i++)
        {
            GPWPL_BUF[len] = GPS_WD[i];
            len++;
        }

        GPWPL_BUF[len] = ',';
        len++;
        GPWPL_BUF[len] = GPS_WD[7];
        len++;
        GPWPL_BUF[len] = ',';
        len++;

        for(i = 0; i < 8; i++)
        {
            GPWPL_BUF[len] = GPS_JD[i];
            len++;
        }

        //站点经度
        GPWPL_BUF[len] = ',';
        len++;
        GPWPL_BUF[len] = GPS_JD[8];
        len++;
        GPWPL_BUF[len] = ',';
        len++;
    }
    else
    {
        //0=固定站
        //站点纬度
        for(i = 0; i < 7; i++)
        {
            GPWPL_BUF[len] = EEPROM_Buffer[0x20 + i];
            len++;
        }

        GPWPL_BUF[len] = ',';
        len++;
        GPWPL_BUF[len] = EEPROM_Buffer[0x20 + 7];
        len++;
        GPWPL_BUF[len] = ',';
        len++;

        for(i = 0; i < 8; i++)
        {
            GPWPL_BUF[len] = EEPROM_Buffer[0x30 + i];
            len++;
        }

        //站点经度
        GPWPL_BUF[len] = ',';
        len++;
        GPWPL_BUF[len] = EEPROM_Buffer[0x30 + 8];
        len++;
        GPWPL_BUF[len] = ',';
        len++;
    }


    for(i = 0; i < 6; i++)
    {
        n = EEPROM_Buffer[0X08 + i];

        if (n == 0x00)
        {
            break;
        }

        GPWPL_BUF[len] = n;
        len++;	// 呼号字段
    }

    GPWPL_BUF[len] = '-';
    len++;	 	//插入

    i = EEPROM_Buffer[0X0F];		//插入SSID

    if (i < 10)	//0-9
    {
        GPWPL_BUF[len] = i % 10 + 0x30;
        len++;
    }
    else	   //10-99
    {
        GPWPL_BUF[len] = i / 10 % 10 + 0x30;
        len++;
        GPWPL_BUF[len] = i % 10 + 0x30;
        len++;
    }

    GPWPL_BUF[len] = '*';
    len++;

    //计算并插入校验值
    crc_hex = 0;

    for (i = 1; i < len - 1; i++)
    {
        crc_hex = crc_hex ^ GPWPL_BUF[i];   //	//校验  $ 和 * 直接的全部字符的异或值 ，不含$ 和 *
    }

    crc_h = crc_hex / 16;

    if (crc_h < 10)
    {
        GPWPL_BUF[len] = crc_h + 0x30;
        len++;
    }
    else
    {
        GPWPL_BUF[len] = crc_h + 0x37;
        len++;
    }

    crc_l = crc_hex % 16;

    if (crc_l < 10)
    {
        GPWPL_BUF[len] = crc_l + 0x30;
        len++;
    }
    else
    {
        GPWPL_BUF[len] = crc_l + 0x37;
        len++;
    }

    //插入换行符、结束符
    GPWPL_BUF[len] = '\r';
    len++;
    GPWPL_BUF[len] = '\n';
    len++;	//补上换行符

    GPWPL_BUF[len] = 0x00;	  		//补上结束符
    //以上为数据格式转换
}




//
////校验  $ 和 * 直接的全部字符的异或值 ，不含$ 和 * ,速率9600
//void KISS_TO_GPWPL()		//未压缩KISS数据转成GPWPL航点格式
//{
//	unsigned char code GPWPL[]={"$GPWPL,"};
//	unsigned char i,len,w,k,asc,ssid,crc_hex,crc_h,crc_l;
////unsigned char code test[]={"GPWPL,3100.00,N,12100.00,E,BH4TDV-9"};
////unsigned char code test[]={"$GPWPL,3216.11,N,10532.02,E,BG8EJT-13*"};
////$GPWPL,2305.35,N,10935.62,E,BD7RS-10*5F
////$GPWPL,3216.11,N,10532.02,E,BG8EJT-13*03
////$GPWPL,3447.30,N,11342.70,E,BG6JJI-13*1B
////$GPWPL,2502.96,N,10242.02,E,BG8SXC-10*1F
////$GPWPL,3949.00,N,11812.01,E,BG3MCK-13*1A
//
//
//// 0x82, 0xA0, 0x9E, 0xA8, 0x86, 0x62, 0xE0, 				//	目标地址 +SSID
//// 0x84, 0x90, 0x68, 0xA8, 0x88, 0xAC, 0xF2, 				//	源地址	 +SSID
//// 0xAE, 0x92, 0x88, 0x8A, 0x62, 0x40, 0x63, 				//	路径	 +SSID
//// 0x03, 0xF0, 											//	03f0
//// 0x21, 													//	类型
//// 0x33, 0x31, 0x30, 0x30, 0x2E, 0x30, 0x30, 0x4E, 		//	经纬度
//// 0x2F, 													//	分隔符	"/"
//// 0x31, 0x32, 0x31, 0x30, 0x30, 0x2E, 0x30, 0x30, 0x45, 	//	经纬度
//// 0x3E, 													//	图标类型
//// 0x20, 0x30, 0x37, 0x2E, 0x38, 0x56, 0x20, 0x32, 0x31, 0x43, 0x20, 0x6F, 0x74,		 //信息
//
//
////	for (i=0;i<128;i++) {	GPWPL_BUF[i]= 0x00;}   	//初始清0
//
//
//	w=13;
//	for (k=0;k<6;k++)		 //最多6个路径
//	{
//			 if ((KISS_DATA[w] & 0x01) == 1)	 //路径结束符号
//			 {
//				len=0;
//				for (i=0;i<7;i++)   {	GPWPL_BUF[len]=GPWPL[i];	 len++;}	  //插入字头
//
//
//				//插入经纬度
//				for (i=w+4;i<w+11;i++)   {	GPWPL_BUF[len]= KISS_DATA[i];	 len++;}
//
//				GPWPL_BUF[len]= ',';  len++;
//				GPWPL_BUF[len]=  KISS_DATA[w+11];  len++;
//				GPWPL_BUF[len]= ',';  len++;
//
//				for (i=w+13;i<w+21;i++)   {	GPWPL_BUF[len]= KISS_DATA[i];	 len++;}
//
//				GPWPL_BUF[len]= ',';  len++;
//				GPWPL_BUF[len]=  KISS_DATA[w+21];  len++;
//				GPWPL_BUF[len]= ',';  len++;
//
//
//
//				//插入呼号
//
//				for (i=7;i<13;i++)
//				{  	asc= ((KISS_DATA[i])  >> 1);	if (asc !=' ')	  {	 GPWPL_BUF[len]= asc;  len++;	}  	 }
//
//				//插入SSID
//				ssid=	 ((KISS_DATA[13] & 0x1E)>>1) ;
//				if (ssid>0)
//				{	GPWPL_BUF[len]= '-';	 len++;
//
//					if (ssid<10)
//					{ GPWPL_BUF[len]=ssid+0x30;  	len++;  }	  //转换成ASCII
//					else
//					{ GPWPL_BUF[len]=ssid/10+0x30;  len++;	GPWPL_BUF[len]=ssid%10+0x30;	 len++;	}
//
//				}
//
//				   GPWPL_BUF[len]= '*';	 len++;
//
//					 //计算并插入校验值
//				crc_hex=0;
//				for (i=1;i<len-1;i++)   { crc_hex=crc_hex^GPWPL_BUF[i];}	//	//校验  $ 和 * 直接的全部字符的异或值 ，不含$ 和 *
//
//				crc_h=crc_hex/16;
//				if (crc_h<10) 			{ GPWPL_BUF[len]= crc_h+0x30;	 len++; }
//				else		 			{ GPWPL_BUF[len]= crc_h+0x37;	 len++;}
//
//				crc_l=crc_hex%16;
//				if (crc_l<10)  			{ GPWPL_BUF[len]= crc_l+0x30;	 len++; }
//				else		 			{ GPWPL_BUF[len]= crc_l+0x37;	 len++;  }
//
//				//插入换行符、结束符
//				GPWPL_BUF[len]= '\r';   len++;	GPWPL_BUF[len]= '\n';	  len++;	//补上换行符
//
//				GPWPL_BUF[len]= 0x00;	  		//补上结束符
//				//以上为数据格式转换
//
//			return;
//			 }
//	w=w+7;
//	}
//}
//
