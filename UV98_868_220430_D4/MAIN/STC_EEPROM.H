
#ifndef _STC_EEPROM_H_
#define _STC_EEPROM_H_

/*************  外部函数和变量声明 *****************/
 
/******************** 读1个字节函数*****************/
extern void EEPROM_SectorErase(uint EE_address) ;

extern unsigned char EEPROM_read_one(uint EE_address);
/******************** 写1个字节函数  *****************/
extern void EEPROM_write_one(uint EE_address,uchar ndata);

extern void EEPROM_UPDATA();   //更新设置	
extern void DEMO_SETUP(uchar UV);		  //通电检查EEROM，如有错误，恢复默认参数  0=U段 1=V段

extern void   READ_SETUP();  	//读取设置到缓冲	

extern unsigned char Initial_check_erom();	  //检查MAC地址	是否已存储，没有则TNC没初始化
extern unsigned char  EEPROM_Buffer[512];

extern void   POWER_READ_SETUP();  	//开机读取全部参数设置到缓冲


extern unsigned char code	VER[];	//固件版本日期信息


extern void   READ_VER(uchar UARTx);  	//读取版本号


/*****************************************/
#endif