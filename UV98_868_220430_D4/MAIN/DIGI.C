
#include "STC8A8K64D4.H"
#include "STC_EEPROM.H"

#include "DELAY.H"
#include "IO.H"

#include "DIGI.H"

#include "UART2.H"
#include "BEACON.H"
#include "CMX865A_CODE.H"
#include "UARTx.H"

#include "CHx.H"

#include "PUBLIC_BUF.H"
#include  <string.h>



/************* 数字中继 **************/

void DIGI_FUN();

uchar 	CHECK_CODE()
{
    uchar *p;
    uchar temp[10];
    unsigned char  i ;

    for(i = 0; i < 6; i++)
    {
        temp[i] = EEPROM_Buffer[REMOTE_SN + i];
        temp[i + 1] = 0;
    }

    p = strstr(SN_RX_BUFFER, temp);

    if (p != NULL)
    {
        for(i = 0; i < 8; i++)
        {
            temp[i] = (*(p + i));
            temp[i + 1] = 0;
        }


        if ((temp[6] == 'A') && (temp[7] == '1'))
        {
            EEPROM_Buffer[DIGI1_EN] = 1;
            EEPROM_UPDATA();
            UART2_SendString("DIGI 1 ON\r\n");
            return 1;
        }

        if ((temp[6] == 'A') && (temp[7] == '0'))
        {
            EEPROM_Buffer[DIGI1_EN] = 0;
            EEPROM_UPDATA();
            UART2_SendString("DIGI 1 OFF\r\n");
            return 1;
        }

        if ((temp[6] == 'B') && (temp[7] == '1'))
        {
            EEPROM_Buffer[DIGI2_EN] = 1;
            EEPROM_UPDATA();
            UART2_SendString("DIGI 2 ON\r\n");
            return 1;
        }

        if ((temp[6] == 'B') && (temp[7] == '0'))
        {
            EEPROM_Buffer[DIGI2_EN] = 0;
            EEPROM_UPDATA();
            UART2_SendString("DIGI 2 OFF\r\n");
            return 1;
        }

        if ((temp[6] == 'R') && (temp[7] == '0'))
        {
            UART2_SendString("NOW RST\r\n");
            IAP_CONTR = 0x20;
            return 1;
        }

        return 1;
    }  //

    return 0;
}








//中继转发处理 ,返回  0=名称不对，1=计数已经为0 ，2=允许转发
unsigned char DIGI_NAME_CHECK(uchar *dig_name, uchar kiss_idx)
{
    unsigned char  SSID, i;
    uchar temp;
    uchar path_name[8];  	 //路径地址6位，含空格

    //数字中继NAME开,并且路径包含NAME-N，N>0，则转发
//----------------------------------------
    SSID =	 ((KISS_DATA[kiss_idx + 6] & 0x1E) >> 1) ;		//求SSID

    if (SSID == 0)
    {
        return 1;    //转发次数为0，结束退出
    }

//----------------------------------------
    for (i = 0; i < 6; i++)
    {
        path_name[i] = ' ';   //填充空格
    }

    for(i = 0; i < 6; i++) 			//读出中继名称
    {
        temp = *(dig_name + i);	 //

        if (temp == 0x00)
        {
            break;
        }
        else
        {
            path_name[i] = temp;
        }
    }

    for(i = 0; i < 6; i++)
    {
        if (path_name[i] != KISS_DATA[kiss_idx + i] >> 1)
        {
            return 0;   //对比名称
        }
    }

//----------------------------------------

    //SSID>0
    SSID --; 	   //SSID计数-1
    SSID <<= 1;	   //左移1位
    KISS_DATA[kiss_idx + 6] &= 0xE1;	//清除SSID,SSID=0	//末尾路径结束标志不动
    KISS_DATA[kiss_idx + 6]  |= SSID;	//写入新的SSID

    if (SSID == 0)
    {
        KISS_DATA[kiss_idx + 6]  |= 0x80;   //SSID=0时，设SSID.7=1 ,即该路径转发完毕标志
    }

    return 2;
}



//KISS数据，在指定的DIGI前，插入呼号处理 ，转成新的KISS数据
void DIGI_CALL_COV(uchar kiss_idx)
{
    unsigned char i, k, new_idx;

    uchar MY_CALL[7];

    //如果数据总长小于（128-9），则KISS第14位右移7个字节，空出7个字节插入中继名称
    if  (KISS_LEN > 150)
    {
        return;   //如果KISS数据总长度不够插入7个字节中继名称，则退出
    }


    if (EEPROM_Buffer[0x08] == 0x00)
    {
        return;   //如果没有呼号则退出
    }

    //--------------------------------

    //在指定索引前空出7个字节
    new_idx = KISS_LEN;
    k = KISS_LEN - kiss_idx + 1;

    for (i = 0; i < k; i++)
    {
        KISS_DATA[new_idx + 7] = KISS_DATA[new_idx];
        new_idx--;
    }

    //--------------------------------

    for (i = 0; i < 6; i++)
    {
        MY_CALL[i]	= 0x40;    //Source呼号填充空格
    }

    for (i = 0; i < 6; i++)  		//Source呼号填充空格
    {
        if (EEPROM_Buffer[0x08 + i] == 0x00)
        {
            break;
        }

        MY_CALL[i] = EEPROM_Buffer[0x08 + i] << 1;
    }

    MY_CALL[6] = (EEPROM_Buffer[0x0F] << 1) | 0xE0 ;
    k++;	  //插入高位=1 ,Source SSID +路径结束符=0

    //--------------------------------
    //空出7个字节插入本机名称
    for (i = 0; i < 7; i++)
    {
        KISS_DATA[kiss_idx + i] = MY_CALL[i];
    }

    //--------------------------------
    KISS_LEN = (KISS_LEN + 7) ;	 //KISS数据长度+7
    //--------------------------------
//		GetCrc16_LEN(KISS_DATA,KISS_LEN);	  // 计算给定长度数据的16位CRC。
//		APRS_TX(KISS_DATA,KISS_LEN);			//发送RF

//		UART2_SendData(0XC0); UART2_SendData(0X00);
//		for (i=0;i<KISS_LEN;i++)    {  UART2_SendData(KISS_DATA[i]);	   }  //串口调试
//		UART2_SendData(0XC0);
//		UART2_SendData(FCS_LO)	;  	  UART2_SendData(FCS_HI)	;
}





//中继发送处理后的KISS数据
void DIGI_RF_TXD()
{
    uchar i;

    if (EEPROM_Buffer[0x01D5] > 0)
    {
        for (i = 0; i < EEPROM_Buffer[0x01D5]; i++)
        {
            Delay_time_25ms(40);
        }
    }

//	CMX865A_HDLC_TX(KISS_DATA,KISS_LEN);
    Delay_time_25ms(6);

    BEACON_TX_CHX(1);

}


void DIGI_FUN()
{
    uchar i, idx;

    CHECK_CODE();

//中继字段位置，每7个字节是1段 ,KISS数据从第3段开始检查
    if ((EEPROM_Buffer[DIGI1_EN] == 0) && (EEPROM_Buffer[DIGI2_EN] == 0))
    {
        return;
    }

    if ((KISS_DATA[13] & 0x01) == 1)
    {
        return;	   //如果没有中继转发路径，则退出
    }


    idx = 13;

    for (i = 0; i < 6; i++)	 //   最多查询6个路径
    {

        if (EEPROM_Buffer[DIGI1_EN] != 0) //数字中继WIDE1开,并且路径包含WIDE1-N，N>0，则转发
        {
            if (DIGI_NAME_CHECK(EEPROM_Buffer + DIGI1_NAME, idx + 1) == 2)	//对比中继名称1、转发次数
            {
                DIGI_CALL_COV(idx + 1); 	   //插入本机名,插在最前面
                DIGI_RF_TXD();
                return;
            }
        }

        if (EEPROM_Buffer[DIGI2_EN] != 0) //数字中继WIDE2开,并且路径包含WIDE2-N，N>0，则转发
        {
            if (DIGI_NAME_CHECK(EEPROM_Buffer + DIGI2_NAME, idx + 1) == 2) //对比中继名称2、转发次数
            {
                DIGI_CALL_COV(idx + 1); 	   //插入本机名,插在最前面
                DIGI_RF_TXD();
                return;
            }
        }

        if ((KISS_DATA[idx + 7] & 0x01) == 1)
        {
            return;	   //如果没有中继转发路径，则退出
        }

        idx = idx + 7;

    }


    return;

}


void dig_Initial()
{
    unsigned char code  TX_PATH1[8] = {"WIDE1 "};			//发射路径1
    unsigned char code  TX_PATH2[8] = {"WIDE2 "};			//发射路径2
    unsigned char code  DIG_PATH1[8] = {"WIDE1 "};			//中继名称1，优先检测
    unsigned char code  DIG_PATH2[8] = {"WIDE2 "};			//中继名称2，次优先检测


    unsigned char code  digi_on_code[6] = {"123456"};
//	unsigned char code  digi_off_code[6]={"654321"};

    uchar i;

    //TX  WIDE1-1
    for(i = 0; i < 6; i++)
    {
        EEPROM_write_one(PATH1_NAME + i, TX_PATH1[i]);   //
    }

    EEPROM_write_one(PATH1_NAME + 6, 0x00); //结束符号
    EEPROM_write_one(PATH1_COUNT, 1 ); 	//0=关闭，1-9转发次数

    //TX  WIDE2-1
    for(i = 0; i < 6; i++)
    {
        EEPROM_write_one(PATH2_NAME + i, TX_PATH2[i]);   //
    }

    EEPROM_write_one(PATH2_NAME + 6, 0x00); //结束符号
    EEPROM_write_one(PATH2_COUNT, 0 ); 	//0=关闭，1-9转发次数

    //DIG  WIDE1-1
    for(i = 0; i < 6; i++)
    {
        EEPROM_write_one(DIGI1_NAME + i, DIG_PATH1[i]);   //
    }

    EEPROM_write_one(DIGI1_NAME + 6, 0x00); //结束符号
    EEPROM_write_one(DIGI1_EN, 1 ); 	//0=关闭，1转发

    //DIG  WIDE2-1
    for(i = 0; i < 6; i++)
    {
        EEPROM_write_one(DIGI2_NAME + i, DIG_PATH2[i]);   //
    }

    EEPROM_write_one(DIGI2_NAME + 6, 0x00); //结束符号
    EEPROM_write_one(DIGI2_EN, 1 ); 	//0=关闭，1转发




    //*****************************************************DIGI 开关密码
    for(i = 0; i < 6; i++)
    {
        EEPROM_write_one(REMOTE_SN + i, digi_on_code[i]);
    }

    EEPROM_write_one(REMOTE_SN + 6, 0x00); //结束符号
//	for(i=0;i<6;i++){EEPROM_write_one(0x0148+i, digi_off_code[i]);}
//	EEPROM_write_one(0x0148+6, 0x00);  //结束符号
}

//void READ_DIGI_STU()
//{	  uchar i;
////--------------------------------------------------------
//UART2_SendString("AT+PATH1_NAME=");	//0-9
//for(i=0;i<6;i++)
//{
//if (EEPROM_Buffer[PATH1_NAME+i]==0){break;}	UART2_SendData(EEPROM_Buffer[PATH1_NAME+i]);
//}
//UART2_SendString("\r\n");
//
//UART2_SendString("AT+PATH1_COUNT=");		UART2_SendData(EEPROM_Buffer[PATH1_COUNT]%10+0x30);		UART2_SendString("\r\n");
////--------------------------------------------------------
//UART2_SendString("AT+PATH2_NAME=");	//0-9
//for(i=0;i<6;i++)
//{
//if (EEPROM_Buffer[PATH2_NAME+i]==0){break;} 	UART2_SendData(EEPROM_Buffer[PATH2_NAME+i]);
//}
//UART2_SendString("\r\n");
//
//UART2_SendString("AT+PATH2_COUNT=");	UART2_SendData(EEPROM_Buffer[PATH2_COUNT]%10+0x30);		UART2_SendString("\r\n");
////--------------------------------------------------------
//UART2_SendString("AT+DIGI1_NAME=");
//for(i=0;i<6;i++)
//{
//if (EEPROM_Buffer[DIGI1_NAME+i]==0){break;}   UART2_SendData(EEPROM_Buffer[DIGI1_NAME+i]);
//}
//UART2_SendString("\r\n");
//
//if (EEPROM_Buffer[DIGI1_EN]==0)	{	UART2_SendString("AT+DIGI1=OFF");}	else {UART2_SendString("AT+DIGI1=ON");}	 	UART2_SendString("\r\n");	 //
////--------------------------------------------------------
//UART2_SendString("AT+DIGI2_NAME=");
//for(i=0;i<6;i++)
//{
//if (EEPROM_Buffer[DIGI2_NAME+i]==0){break;} 	UART2_SendData(EEPROM_Buffer[DIGI2_NAME+i]);
//}
//	UART2_SendString("\r\n");
//
//if (EEPROM_Buffer[DIGI2_EN]==0)	{	UART2_SendString("AT+DIGI2=OFF");}	else {UART2_SendString("AT+DIGI2=ON");}	 	UART2_SendString("\r\n");	 //
////WIDE1启用
//
//
////--------------------------------------------------------
//
//	//中继远程密码
//UART2_SendString("AT+CODE=");		 	for(i=0;i<6;i++){UART2_SendData(EEPROM_Buffer[REMOTE_SN+i]);} 	 UART2_SendString("\r\n");	 //
////	UART2_SendString("AT+OFFSN=");		 	for(i=0;i<6;i++){UART2_SendData(EEPROM_Buffer[0X0148+i]);} 	 UART2_SendString("\r\n");	 //
//
//}
//
//
//
//uchar  SET_PATH_DIGI()
//{
//  uchar i,len;
//
// 	//============================================== //设置路径1的名称
//if (CHECK_AT_CMD("AT+PATH1_NAME=")==1)
//{
//	len=strlen(UARTx_BUF)-14-2;
//	if (len>6)  {	return 0;}	 //中继名称超长，错误
//	if (len==0) {	return 0;}	 //中继名称没设置，错误
//	for(i=0;i<len;i++) 	{ EEPROM_Buffer[PATH1_NAME+i]= UARTx_BUF[14+i]; EEPROM_Buffer[PATH1_NAME+i+1]=0;  }
//	EEPROM_UPDATA(); return 1;
//}
//	//============================================== //设置TXpath1的次数 0-9  0=关闭
//if (CHECK_AT_CMD("AT+PATH1_COUNT=")==1)
//{	len=strlen(UARTx_BUF)-15-2;
//	if (len==1)	{  	EEPROM_Buffer[PATH1_COUNT]=(UARTx_BUF[15]-0x30);EEPROM_UPDATA();  	return 1;	  }
// 	return 0;
//}
//
// 	//============================================== //设置路径2的名称
//if (CHECK_AT_CMD("AT+PATH2_NAME=")==1)
//{
//	len=strlen(UARTx_BUF)-14-2;
//
//	if (len>6)  {	return 0;}	 //中继名称超长，错误
//	if (len==0) {	return 0;}	 //中继名称没设置，错误
//	for(i=0;i<len;i++) 	{ EEPROM_Buffer[PATH2_NAME+i]= UARTx_BUF[14+i]; EEPROM_Buffer[PATH2_NAME+i+1]=0;  }
//	EEPROM_UPDATA(); return 1;
//}
//
////============================================== //设置TXpath1的次数 0-9 0=关闭
//if (CHECK_AT_CMD("AT+PATH2_COUNT=")==1)
//{	len=strlen(UARTx_BUF)-15-2;
//	if (len==1)	{  	EEPROM_Buffer[PATH2_COUNT]=(UARTx_BUF[15]-0x30);EEPROM_UPDATA();  	return 1;	  }
//	return 0;
//}
////============================================== //设置中继1名称
//if (CHECK_AT_CMD("AT+DIGI1_NAME=")==1)
//{
//	len=strlen(UARTx_BUF)-14-2;
//	if (len>6)  {	return 0;}	 //中继名称超长，错误
//	if (len==0) {	return 0;}	 //中继名称没设置，错误
//	for(i=0;i<len;i++) 	{ EEPROM_Buffer[DIGI1_NAME+i]= UARTx_BUF[14+i]; EEPROM_Buffer[DIGI1_NAME+i+1]=0;  }
//	EEPROM_UPDATA(); return 1;
//}
////============================================== //设置中继2名称
//if (CHECK_AT_CMD("AT+DIGI2_NAME=")==1)
//{
//	len=strlen(UARTx_BUF)-14-2;
//	if (len>6)  {	return 0;}	 //中继名称超长，错误
//	if (len==0) {	return 0;}	 //中继名称没设置，错误
//	for(i=0;i<len;i++) 	{ EEPROM_Buffer[DIGI2_NAME+i]= UARTx_BUF[14+i]; EEPROM_Buffer[DIGI2_NAME+i+1]=0;  }
//	EEPROM_UPDATA(); return 1;
//}
//	//==============================================
//if (CHECK_AT_CMD("AT+DIGI1=ON")==1){ EEPROM_Buffer[DIGI1_EN]=1;	EEPROM_UPDATA(); return 1;}
//if (CHECK_AT_CMD("AT+DIGI1=OFF")==1){ EEPROM_Buffer[DIGI1_EN]=0;	EEPROM_UPDATA(); return 1;}
//if (CHECK_AT_CMD("AT+DIGI2=ON")==1){ EEPROM_Buffer[DIGI2_EN]=1;	EEPROM_UPDATA(); return 1;}
//if (CHECK_AT_CMD("AT+DIGI2=OFF")==1){ EEPROM_Buffer[DIGI2_EN]=0;	EEPROM_UPDATA(); return 1;}
//	//==============================================
//
//if (CHECK_AT_CMD("AT+TEST=ON")==1){  BEACON_GPS_TXD(); return 2;}
////============================================== //中继远程密码
//if (CHECK_AT_CMD("AT+CODE=")==1)
//{
//	len=strlen(UARTx_BUF)-8-2;
//	if (len!=6) {	return 0;}	 //中继ON密码必须是6位
//	for(i=0;i<len;i++) 	{ EEPROM_Buffer[0X01D0+i]= UARTx_BUF[8+i]; EEPROM_Buffer[0X01D0+i+1]=0;  }
//	EEPROM_UPDATA(); return 1;
//
//}
//
////	if (CHECK_AT_CMD("AT+OFFSN=")==1)
////	{
////		len=strlen(UARTx_BUF)-9-2;
////		if (len!=6) {	return 0;}	 //中继OFF密码必须是6位
////		for(i=0;i<len;i++) 	{ EEPROM_Buffer[0X01D8+i]= UARTx_BUF[9+i]; EEPROM_Buffer[0X01D8+i+1]=0;  }
////		EEPROM_UPDATA(); return 1;
//// 	}
////
// return 0;
//}
//
//



/*
//网关电台RF信标，KISS格式，不含C0 00 ..C0，不含校验值，末尾补结束符0x00 ,结束符0X00不发送
unsigned char code Gate_beacen[]=
{	 0x82, 0xA0, 0x9E, 0xA8, 0x86, 0x62, 0xE0, 				//	目标地址 +SSID
	 0x84, 0x90, 0x68, 0xA8, 0x88, 0xAC, 0xF2, 				//	源地址	 +SSID
	 0xAE, 0x92, 0x88, 0x8A, 0x62, 0x40, 0x63, 				//	路径	 +SSID
	 0x03, 0xF0, 											//	03f0
	 0x21, 													//	类型
	 0x33, 0x31, 0x30, 0x30, 0x2E, 0x30, 0x30, 0x4E, 		//	经纬度
	 0x2F, 													//	分隔符	"/"
	 0x31, 0x32, 0x31, 0x30, 0x30, 0x2E, 0x30, 0x30, 0x45, 	//	经纬度
	 0x3E, 													//	图标类型
	 0x20, 0x30, 0x37, 0x2E, 0x38, 0x56, 0x20, 0x32, 0x31, 0x43, 0x20, 0x6F, 0x74,		 //信息
	 0x00, };	//非文本字符串尾部编译器不会补上00H，需手动补上，结束符0X00不发送



	82 A0 88 A4 62 64 E0
	84 90 68 A8 88 AC 6A
	AE 92 88 8A 62 40 63
	03 F0
	3D 33 31 32 30 2E 34 30 4E 2F 31 32 30 31 32 2E 30 30 45 24 20 68 74 74 70 3A 2F 2F 61 70 72 73 64 72 6F 69 64 2E 6F 72 67 2F C0

	WIDE1-2  WIDE2-1
	C0 00
	82 A0 88 A4 62 64 E0
	84 90 68 A8 88 AC 6A
	AE 92 88 8A 62 40 64
	AE 92 88 8A 64 40 63
	03 F0
	3D 33 31 32 30 2E 34 30 4E 2F 31 32 30 31 32 2E 30 30 45 24 20 68 74 74 70 3A 2F 2F 61 70 72 73 64 72 6F 69 64 2E

	6F 72 67 2F C0


	BH4TDV-9  WIDE1

C0 00 82 A0 9E A8 86 62 E0 84 90 68 A8 88 AC F2 AE 92 88 8A 62 40 63 03 F0 21 33 31 30 30 2E 30 30 4E 2F

31 32 31 30 30 2E 30 30 45 3E 20 30 37 2E 38 56 20 32 31 43 20 6F 74 C0


BH4TDV-5  WIDE1

C0 00 82 A0 88 A4 62 64 E0 84 90 68 A8 88 AC 6A AE 92 88 8A 62 40 63 03 F0 3D 33 31 32 30 2E 34 30 4E 2F

31 32 30 31 32 2E 30 30 45 24 20 68 74 74 70 3A 2F 2F 61 70 72 73 64 72 6F 69 64 2E 6F 72 67 2F C0


WIDE1-1
C0 00 82 A0 9E A8 86 62 E0 84 90 68 A8 88 AC F2 AE 92 88 8A 62 40 63 03 F0 21 33 31 30 30 2E 30 30 4E 2F

31 32 31 30 30 2E 30 30 45 3E 20 30 37 2E 38 56 20 32 31 43 20 6F 74 C0


WIDE1-2
C0 00 82 A0 88 A4 62 64 E0 84 90 68 A8 88 AC 6A AE 92 88 8A 62 40 65 03 F0 3D 33 31 32 30 2E 34 30 4E 2F

31 32 30 31 32 2E 30 30 45 24 20 68 74 74 70 3A 2F 2F 61 70 72 73 64 72 6F 69 64 2E 6F 72 67 2F C0



WIDE1-2  WIDE2-1
C0 00 82 A0 88 A4 62 64 E0 84 90 68 A8 88 AC 6A AE 92 88 8A 62 40 64 AE 92 88 8A 64 40 63 03 F0 3D 33 31

32 30 2E 34 30 4E 2F 31 32 30 31 32 2E 30 30 45 24 20 68 74 74 70 3A 2F 2F 61 70 72 73 64 72 6F 69 64 2E

6F 72 67 2F C0



WIDE1-2  WIDE2-2
C0 00 82 A0 88 A4 62 64 E0 84 90 68 A8 88 AC 6A AE 92 88 8A 62 40 64 AE 92 88 8A 64 40 65
03 F0 3D 33 31 32 30 2E 34 30 4E 2F 31 32 30 31 32 2E 30 30 45 24 20 68 74 74 70 3A 2F 2F 61 70 72 73 64

72 6F 69 64 2E 6F 72 67 2F C0
BH4TDV-9  WIDE1

C0 00 82 A0 9E A8 86 62 E0 84 90 68 A8 88 AC F2 AE 92 88 8A 62 40 63 03 F0 21 33 31 30 30 2E 30 30 4E 2F

31 32 31 30 30 2E 30 30 45 3E 20 30 37 2E 38 56 20 32 31 43 20 6F 74 C0


BH4TDV-5  WIDE1

C0 00 82 A0 88 A4 62 64 E0 84 90 68 A8 88 AC 6A AE 92 88 8A 62 40 63 03 F0 3D 33 31 32 30 2E 34 30 4E 2F

31 32 30 31 32 2E 30 30 45 24 20 68 74 74 70 3A 2F 2F 61 70 72 73 64 72 6F 69 64 2E 6F 72 67 2F C0


WIDE1-1
C0 00 82 A0 9E A8 86 62 E0 84 90 68 A8 88 AC F2 AE 92 88 8A 62 40 63 03 F0 21 33 31 30 30 2E 30 30 4E 2F

31 32 31 30 30 2E 30 30 45 3E 20 30 37 2E 38 56 20 32 31 43 20 6F 74 C0


WIDE1-2
C0 00 82 A0 88 A4 62 64 E0 84 90 68 A8 88 AC 6A AE 92 88 8A 62 40 65 03 F0 3D 33 31 32 30 2E 34 30 4E 2F

31 32 30 31 32 2E 30 30 45 24 20 68 74 74 70 3A 2F 2F 61 70 72 73 64 72 6F 69 64 2E 6F 72 67 2F C0



WIDE1-2  WIDE2-1
C0 00 82 A0 88 A4 62 64 E0 84 90 68 A8 88 AC 6A AE 92 88 8A 62 40 64 AE 92 88 8A 64 40 63 03 F0 3D 33 31

32 30 2E 34 30 4E 2F 31 32 30 31 32 2E 30 30 45 24 20 68 74 74 70 3A 2F 2F 61 70 72 73 64 72 6F 69 64 2E

6F 72 67 2F C0



WIDE1-2  WIDE2-2
C0 00 82 A0 88 A4 62 64 E0 84 90 68 A8 88 AC 6A AE 92 88 8A 62 40 64 AE 92 88 8A 64 40 65
03 F0 3D 33 31 32 30 2E 34 30 4E 2F 31 32 30 31 32 2E 30 30 45 24 20 68 74 74 70 3A 2F 2F 61 70 72 73 64

72 6F 69 64 2E 6F 72 67 2F C0

*/




