#include "STC8A8K64D4.H"
#include "STC_EEPROM.H"
#include "UART1.H"
#include "UART2.H"
#include "ADC.H"
#include "DIGI.H"
#include "tostring.H"

#include  <INTRINS.H> //Keil library 




#define IAP_EN			0x80

#define 	IAP_TRIG()	IAP_TRIG = 0x5A,	IAP_TRIG = 0xA5		//IAP触发命令

//8A8K64D4 专用
//sfr IAP_TPS  =   0xF5;   // 8A8K64D4
//#define     MAIN_Fosc       22118400L   //定义主时钟（精确计算115200波特率）
#define     IAP_ENABLE()    IAP_CONTR = IAP_EN; IAP_TPS = FOSC / 1000000
#define     IAP_DISABLE()   IAP_CONTR = 0; IAP_CMD = 0; IAP_TRIG = 0; IAP_ADDRH = 0xff; IAP_ADDRL = 0xff





/*************  外部函数和变量声明 *****************/
unsigned char code	VER[] = {"UV98_868_220430_D4"};	//固件版本日期信息


unsigned char  EEPROM_Buffer[512];



void   READ_VER(uchar UARTx)  	//读取版本号
{
    READ_CPU_ID();	 //读取CPU序列号
    READ_ADC();		 //读取电压值
 
    if (UARTx == 1)
    {
        UART1_SendString(" Ver: ");
        UART1_SendString(VER);
        UART1_SendString(" | ");	  //报告版本
        UART1_SendString("CPU ID: ");
        UART1_SendString(CPU_ID);
        UART1_SendString(" | ");	 //报告CPU序列号
        UART1_SendString("Voltage: ");
        UART1_SendString(DY);
        UART1_SendString(" V| ");	 //
 
    }
    else
    {
        UART2_SendString(" Ver: ");
        UART2_SendString(VER);
        UART2_SendString(" | ");	  //报告版本
        UART2_SendString("CPU ID: ");
        UART2_SendString(CPU_ID);
        UART2_SendString(" | ");	 //报告CPU序列号
        UART2_SendString("Voltage: ");
        UART2_SendString(DY);
        UART2_SendString(" V| ");	 //
 
    }
}
 

/******************** 读N个字节函数 最多255字节一次 *****************/
/******************** 写N个字节函数 最多255字节一次 *****************/
/******************** 扇区擦除函数 512字节/扇区*****************/


/*********************************************************************/

void DisableEEPROM(void)		//以下语句可以不用，只是出于安全考虑而已
{
    IAP_CONTR = 0;				//禁止IAP/IAP操作
    IAP_CMD   = 0;				//去除IAP/IAP命令
    IAP_TRIG  = 0;				//防止IAP/IAP命令误触发
    IAP_ADDRH = 0xff;			//指向非EEPROM区，防止误操作
    IAP_ADDRL = 0xff;			//指向非EEPROM区，防止误操作
}


/******************** 读N个字节函数 最多255字节一次 *****************/
//void EEPROM_read_n(uint EE_address,uchar *DataAddress,uchar lenth)
//{
////	EA = 0;		//禁止中断
//	IAP_ENABLE();					//宏调用, 设置等待时间，允许IAP/IAP操作，送一次就够
//	IAP_CMD =IAP_READ();						//宏调用, 送字节读命令，命令不需改变时，不需重新送命令
//	do
//	{
//		IAP_ADDRH = EE_address / 256;		//送地址高字节（地址需要改变时才需重新送地址）
//		IAP_ADDRL = EE_address % 256;		//送地址低字节
//		IAP_TRIG();							//宏调用, 先送5AH，再送A5H到IAP/IAP触发寄存器，每次都需要如此
//		_nop_();
//		*DataAddress = IAP_DATA;			//读出的数据送往
//		EE_address++;
//		DataAddress++;
//	}while(--lenth);
//
//	DisableEEPROM();
////	EA = 1;		//重新允许中断
//}


/******************** 读1个字节函数*****************/
unsigned char EEPROM_read_one(uint EE_address)
{
    unsigned char j;
//	EA = 0;		//禁止中断
    IAP_ENABLE();					//宏调用, 设置等待时间，允许IAP/IAP操作，送一次就够
    IAP_CMD = IAP_READ;						//宏调用, 送字节读命令，命令不需改变时，不需重新送命令

    IAP_ADDRH = EE_address / 256;		//送地址高字节（地址需要改变时才需重新送地址）
    IAP_ADDRL = EE_address % 256;		//送地址低字节
    IAP_TRIG();							//宏调用, 先送5AH，再送A5H到IAP/IAP触发寄存器，每次都需要如此
    _nop_();
    j = IAP_DATA;			//读出的数据送往

    DisableEEPROM();
//	EA = 1;		//重新允许中断

    return 	j;
}






/******************** 扇区擦除函数 *****************/
void EEPROM_SectorErase(uint EE_address)
{
//	EA = 0;		//禁止中断
    //只有扇区擦除，没有字节擦除，512字节/扇区。
    //扇区中任意一个字节地址都是扇区地址。
    IAP_ADDRH = EE_address / 256;			//送扇区地址高字节（地址需要改变时才需重新送地址）
    IAP_ADDRL = EE_address % 256;			//送扇区地址低字节
    IAP_ENABLE();							//设置等待时间，允许IAP/IAP操作，送一次就够
    IAP_CMD = IAP_ERASE;						//宏调用, 送扇区擦除命令，命令不需改变时，不需重新送命令
    IAP_TRIG();								//宏调用, 先送5AH，再送A5H到IAP/IAP触发寄存器，每次都需要如此
    DisableEEPROM();
//	EA = 1;		//重新允许中断
}

/******************** 写N个字节函数 最多255字节一次 *****************/
void EEPROM_write_n(uint EE_address, uchar *DataAddress, uchar lenth)
{
//	EA = 0;		//禁止中断
    IAP_ENABLE();							//设置等待时间，允许IAP/IAP操作，送一次就够
    IAP_CMD =	IAP_WRITE;						//宏调用, 送字节写命令，命令不需改变时，不需重新送命令

    do
    {
        IAP_ADDRH = EE_address / 256;		//送地址高字节（地址需要改变时才需重新送地址）
        IAP_ADDRL = EE_address % 256;		//送地址低字节
        IAP_DATA  = *DataAddress;			//送数据到IAP_DATA，只有数据改变时才需重新送
        IAP_TRIG();							//宏调用, 先送5AH，再送A5H到IAP/IAP触发寄存器，每次都需要如此
        _nop_();
        EE_address++;						//下一个地址
        DataAddress++;						//下一个数据
    }
    while(--lenth);						//直到结束

    DisableEEPROM();
//	EA = 1;		//重新允许中断
}

/******************** 写1个字节函数  *****************/
void EEPROM_write_one(uint EE_address, uchar ndata)
{
//	EA = 0;		//禁止中断
    IAP_ENABLE();							//设置等待时间，允许IAP/IAP操作，送一次就够
    IAP_CMD =	IAP_WRITE;						//宏调用, 送字节写命令，命令不需改变时，不需重新送命令

    IAP_ADDRH = EE_address / 256;		//送地址高字节（地址需要改变时才需重新送地址）
    IAP_ADDRL = EE_address % 256;		//送地址低字节
    IAP_DATA  = ndata;			//送数据到IAP_DATA，只有数据改变时才需重新送
    IAP_TRIG();							//宏调用, 先送5AH，再送A5H到IAP/IAP触发寄存器，每次都需要如此
    _nop_();
    //直到结束

    DisableEEPROM();
//	EA = 1;		//重新允许中断
}
void EEPROM_write_String(uint EE_address, uchar *s)
{
    IAP_ENABLE();							//设置等待时间，允许IAP/IAP操作，送一次就够
    IAP_CMD =	IAP_WRITE;						//宏调用, 送字节写命令，命令不需改变时，不需重新送命令

    while (*s)                  //检测字符串结束标志
    {
        IAP_ADDRH = EE_address / 256;		//送地址高字节（地址需要改变时才需重新送地址）
        IAP_ADDRL = EE_address % 256;		//送地址低字节
        IAP_DATA  = *s;			//送数据到IAP_DATA，只有数据改变时才需重新送
        IAP_TRIG();
        _nop_();			//宏调用, 先送5AH，再送A5H到IAP/IAP触发寄存器，每次都需要如此
        EE_address++;			//下一个地址
        s++;					//下一个数据
    } 						//直到结束

    //补写写入0x00结束符号
    IAP_ADDRH = EE_address / 256;		//送地址高字节（地址需要改变时才需重新送地址）
    IAP_ADDRL = EE_address % 256;		//送地址低字节
    IAP_DATA  = 0x00;			//送数据到IAP_DATA，只有数据改变时才需重新送
    IAP_TRIG();
    _nop_();			//宏调用, 先送5AH，再送A5H到IAP/IAP触发寄存器，每次都需要如此

    DisableEEPROM();
}

void   EEPROM_UPDATA()  	//更新设置
{
    uint i;
    EEPROM_SectorErase(0x0000);	   		//写入前，擦除扇区1（0-512字节）

    for(i = 0; i < 512; i++)
    {
        EEPROM_write_one(0x0000 + i, EEPROM_Buffer[i]);
    }
}



void   POWER_READ_SETUP()  	//开机读取全部参数设置到缓冲
{
    uint i;

    for(i = 0; i < 512; i++)
    {
        EEPROM_Buffer[i] = EEPROM_read_one(0x0000 + i);    //复制参数到缓冲区
    }
}


//0=U段  1=V段	  首次初始化默认V段
void DEMO_SETUP(uchar UV)	//通电检查EEROM，如有错误，恢复默认参数
{
    unsigned char i;
    unsigned char code CALL[7] = {"NOCALL"};
    unsigned char code  WD[10] = {"3133.90N"};			//默认
    unsigned char code  JD[10] = {"12022.80E"};			//默认

    unsigned char code  MSG[30] = {"UV98"};			//默认
//	unsigned char code  IP_NAME[50]={"AT+CIPSTART=\"TCP\",\"202.141.176.2\",14580\r\n"};			//默认

    unsigned char code  IP_NAME[50] = {"202.141.176.2"};			//默认

//	unsigned char code  WIFI_NAME[]={"AT+CWJAP=\"HiWiFi_3452AE_2G\",\"13013684000\"\r\n"};			//默认

    unsigned char code  WIFI_NAME[] = {"HiWiFi_3452AE_2G"};			//默认
    unsigned char code  WIFI_CODE[] = {"13013684000"};			//默认


    unsigned char code  FREQ_U[35] = {"1,431.0400,431.0400,0,3,0 "};			//默认  27 字节长度
    unsigned char code  FREQ_V[35] = {"1,144.6400,144.6400,0,3,0,0 "};			//默认	 29 字节长度


    unsigned char code  SOS[30] = {"SOS"};			//默认

//	//******************************************************

    EEPROM_SectorErase(0x0000);	   // 擦除扇区1（0-512字节）

    //擦除  写  重启

    //******************************************************
    EEPROM_write_one(0x0000, 20 / 256);	//RF信标时间设30
    EEPROM_write_one(0x0001, 20 % 256);

    EEPROM_write_one(0x0002, 0);		//定时信标ON/OFF
    EEPROM_write_one(0x0003, 0);		//PTT联动信标ON/OFF
    EEPROM_write_one(0x0004, 0);		//智能信标0-3
    EEPROM_write_one(0x0005, 0);		//队列信标ON/OFF
    EEPROM_write_one(0x0006, 0);		//队列信标时序
    //******************************************************
    EEPROM_write_one(0x0007, 3);		//PTT延时50MS步进，0-9 ，最小200MS 默认50x6=300ms
    //*****************************************************
    EEPROM_write_n(0x0008, CALL, 7);	//清空呼号寄存器
    EEPROM_write_one(0x000F, 7);	  	//SSID=7

    //*****************************************************

    EEPROM_write_one(0x0010, 1 ); //MICE ON/OFF
    EEPROM_write_one(0x0011, 0 ); //MICE类型
    //*****************************************************
    EEPROM_write_one(0x0012, '!' ); //类型
    EEPROM_write_one(0x0013, '/' ); //图标集
    EEPROM_write_one(0x0014, '[' ); //小车图标>  	中继图标#	 网关图标r	 气象图标_	  [ 小人
    //*****************************************************

    EEPROM_write_one(0x0015, 0 ); 	//蓝牙输出GPS 0=OFF、1=ON
    EEPROM_write_one(0x0016, 2 ); 	//蓝牙输出0=OFF   1=KISS hex、2=UI数据 3=航点GPS  4=KISS ASC

    //*****************************************************
    EEPROM_write_one(0x0017, 1 ); 	//坐标格式0=度     1=度分  2=度分秒
    EEPROM_write_one(0x0018, 0 ); 	//速度单位 0=公里  1=海里  2=英里
    EEPROM_write_one(0x0019, 0 ); 	//距离单位 0=公里  1=海里  2=英里
    EEPROM_write_one(0x001a, 0 ); 	//海拔单位 0=米    1=英尺

    EEPROM_write_one(0x001b, 0 ); 	//温度单位 0=摄氏  1=华氏
    EEPROM_write_one(0x001c, 0 ); 	//雨量单位 0=mm    1=inch
    EEPROM_write_one(0x001D, 0 ); 	//风速单位0=m/s    1=mph
    //*****************************************************
    EEPROM_write_one(0x001E, 1 ); 	//
    //*****************************************************
    //*****************************************************


    //*****************************************************
    //2A-2F 3A-3F A0-AF E0-FF 空闲
    EEPROM_write_one(0x002a, 0 ); 	//坐标选择，0=固定坐标/固定站 1=GPS坐标/移动站
    EEPROM_write_one(0x002b, 1 ); 	//GPS 开关0=OFF 1=ON
    EEPROM_write_one(0x002c, 21 ); 	//GPS时区 0-26 ，对应正负0-13-26     21-13=8
    EEPROM_write_one(0x002D, 0 ); 	//GPS省电功能  0=OFF 1=ON
    EEPROM_write_one(0x002E, 1 ); 	//蜂鸣器接收提示 0=OFF 1=ON
    EEPROM_write_one(0x002F, 1 ); 	//蜂鸣器发送提示 0=OFF 1=ON

    EEPROM_write_one(0x003A, 1 ); 	//蓝牙电源开关
    EEPROM_write_one(0x003B, 0 ); 	//报告气压0=OFF 1=ON
    EEPROM_write_one(0x003C, 0 ); 	//报告电压0=OFF 1=ON
    EEPROM_write_one(0x003D, 0 ); 	//报告温度0=OFF 1=ON
    EEPROM_write_one(0x003E, 0 ); 	//报告里程0=OFF 1=ON
    EEPROM_write_one(0x003F, 0 ); 	//报告卫星数0=OFF 1=ON


    //*****************************************************
    EEPROM_write_String(0x0020, WD);	 //纬度
    EEPROM_write_String(0x0030, JD);	 //经度
    EEPROM_write_String(0x0040, MSG); //自定信息


    EEPROM_write_one(0x00A0, 0 ); //内置模块开关	0=OFF 1=ON 2=only TX
    EEPROM_write_one(0x00A1, 1 ); //UV模块音量1-9级
    EEPROM_write_one(0x00A2, 1 ); //MIC灵敏度1-8
    EEPROM_write_one(0x00A3, 0 ); //MIC语音加密 0 1-8

    EEPROM_write_one(0x00A4, 0 ); 	//自动关机时间 0=OFF  1=0.5H
    EEPROM_write_one(0x00A5, 0 ); 	//蓝牙/ISP口显示分析数据
    EEPROM_write_one(0x00A6, 1 ); 	//
    EEPROM_write_one(0x00A7, 0 ); 	//GPS信标 0=RF+GPRS  1=GPRS 2=RF  3=AUTO 4=off
    EEPROM_write_one(0x00A8, 1 ); 	//0=禁止上传接收的信标  1=允许
    EEPROM_write_one(0x00A9, 1 ); 	//上传信标时，是否插入自己的呼号，0=不插入 1=插入



    EEPROM_write_one(0x00AA, 1 ); 	//1=CAR 显示对方车头航向，0=N 显示相对正北方位
    EEPROM_write_one(0x00AB, 1 ); 	//格式0=CSV    1=TXT 2=KISS 3=GPWPL

    EEPROM_write_one(0x00AC, 1 ); 	//背光亮度 0=30  1=50 2=100 3=AUTO(GPS时间控制)
    EEPROM_write_one(0x00AD, 1 );
    //初始LCD界面 0=启动界面   1=主菜单   2=APRS详细信标    3=ARS列表界面
    //4=GPS界面	  5=设置界面   6=雷达界面 7=信息界面

    EEPROM_write_one(0x00AE, 1 ); 	//温度传感器类型 0=DS18B20  1=AM2320
    EEPROM_write_one(0x00AF, 0 ); 	//WIFI开关	 //0=WIFI OFF  1= WIFI ON

    //*****************************************************

    if (UV == 0)
    {
        EEPROM_write_String(0x0080, FREQ_U);
    }
    else
    {
        EEPROM_write_String(0x0080, FREQ_V);
    }

    //0 =U 1=V    U段对讲机模块频率    // V段对讲机模块频率	  27/29字节
    //*****************************************************

    EEPROM_write_one(0x00B0, 1 ); 	//0=GPRS OFF  1= GPRS ON
    EEPROM_write_one(0x00B1, 1 ); 	//登录发送序列号
    EEPROM_write_one(0x00B2, 1 ); 	//APN        99=台湾
    EEPROM_write_one(0x00B3, 0 ); 	//GPRS 连接模式0=TCP 1=UDP
    EEPROM_write_one(0x00B4, 14580 / 256 ); 	//GPRS端口号端口号
    EEPROM_write_one(0x00B5, 14580 % 256 ); 	//端口号


    EEPROM_write_one(0x00B6, 0); 	//0=被动导航  1=动态导航


    EEPROM_write_one(0x00B7, 0); 	//里程 	 	unsigned  long 四字节 //0～4294967295
    EEPROM_write_one(0x00B8, 0);
    EEPROM_write_one(0x00B9, 0);
    EEPROM_write_one(0x00BA, 0);
    EEPROM_write_one(0x00BB, 1 ); 	//累计里程记忆	  0=OFF 1=ON



    EEPROM_write_one(0x0BC, '\\' ); //图标集2
    EEPROM_write_one(0x0BD, 'P' ); //图标2   小车>  	中继图标#	 网关图标r	 气象图标_

    EEPROM_write_one(0x00BE, 180 / 256 ); 	//图标2转换时间
    EEPROM_write_one(0x00BF, 180 % 256 ); 	//


//	EEPROM_write_String(0x00B0,IP_NAME); //服务器IP信息	 42长度	AT+DSCADDR=0,"TCP","202.141.176.2",14580
    for(i = 0; i < 32; i++)		 //限制长度32字节
    {
        EEPROM_write_one(0x00C0 + i, IP_NAME[i]); //域名或IP地址

        if (IP_NAME[i] == 0x00)
        {
            break;
        }
    }

//	EEPROM_write_String(0x0100,WIFI_NAME);	 //WIFI名称

    for(i = 0; i < 15; i++)		 //限制长度15字节
    {
        EEPROM_write_one(0x0100 + i, WIFI_NAME[i]); //WIFI名称

        if (WIFI_NAME[i] == 0x00)
        {
            break;
        }
    }

    for(i = 0; i < 15; i++)		 //限制长度15字节
    {
        EEPROM_write_one(0x0110 + i, WIFI_CODE[i]); //WIFI密码

        if (WIFI_CODE[i] == 0x00)
        {
            break;
        }
    }

    //*****************************************************
    for(i = 0; i < 30; i++)		 //限制长度30字节
    {
        EEPROM_write_one(0x01B0 + i, SOS[i]);	//自定信息

        if (SOS[i] == 0x00)
        {
            break;
        }
    }

    EEPROM_write_one(0x01D0, 0 ); 	// 启用紧急信标0=OFF 1=ON
    EEPROM_write_one(0x01D1, 1 );   //
    EEPROM_write_one(0x01D2, 1 );   //
    EEPROM_write_one(0x01D3, 1 ); 	// CMX865AE TX电平 0-7	 0=-10.5DB  7=0dB
    EEPROM_write_one(0x01D4, 6 ); 	// CMX865AE RX电平 0-7
    EEPROM_write_one(0x01D5, 0 ); 	// 中继转发等待时间
    EEPROM_write_one(0x01D6, 0 ); 	// 中继转发信道，0=A ，1=B ，2=A+B
    EEPROM_write_one(0x01D7, 0 ); 	// 发射信道，0=A ，1=B ，2=A+B


    EEPROM_write_n(0x0120, "START1", 7);	//G5500目标呼号
    EEPROM_write_one(0x0127, 1);	  		//G5500目标呼号SSID
    EEPROM_write_one(0x0128, 0);	  		//G5500启用  0=OFF 1=ON
    EEPROM_write_one(0x0129, 100 / 256);	//固定站高度
    EEPROM_write_one(0x012a, 100 % 256);

    EEPROM_write_one(0x012b, 1); //列表方位显示方式  0=英文  1=0-12  2-0-36

    EEPROM_write_one(0x012c, 0); //  0=关闭LCD  1=启动LCD
    EEPROM_write_one(0x012D, 0); //  LCD显示界面
    EEPROM_write_one(0x012E, 0); //	 最后位置报告
    EEPROM_write_one(0x012F, 0); //	 允许6海里低速发送，高速不发功能



    dig_Initial();

    EEPROM_write_one(0x0130, 0 ); 	//  未移动30分钟报警
    EEPROM_write_one(0x0131, 0 ); 	//  未移动60分钟EMERGENCY

//   	EEPROM_write_one(0x0132, 1 ); 	//  启用微信
//   	EEPROM_write_one(0x0133, 1 ); 	//  启用APRS

    EEPROM_write_one(0x01D1, 0); //LCD亮度 0=自动1  //1=高亮	//2=常规

//LCD亮度
//0=自动1
//1=自动2
//1=高亮
//2=常规
//3=关闭
    EEPROM_write_one(0x01D2, 0); //信标弹出 0=关闭 1=启用
    EEPROM_write_one(0x01DF, 0); //  LCD熄灭时间  0=不熄灭  1=1分钟


    EEPROM_write_one(0x001F, 0x03);	//初始化标记


}


//系统复位 	  //0=U段  1=V段	  首次初始化默认V段
unsigned char Initial_check_erom()	  //检查MAC地址	是否已存储，没有则TNC没初始化
{
    if ( EEPROM_read_one(0X001F) != 0x03)
    {
        DEMO_SETUP(1);
        IAP_CONTR = 0x20;
        return 0;
    }

    return 1;
}


