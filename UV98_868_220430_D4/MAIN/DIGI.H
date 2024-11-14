

#ifndef _DIGI_H_
#define _DIGI_H_

 
/*************  外部函数和变量声明 *****************/



extern   void DIGI_FUN();



#define PATH1_NAME 0X01E0
#define PATH1_COUNT 0X01E7

#define PATH2_NAME 0X01E8
#define PATH2_COUNT 0X01EF

#define DIGI1_NAME 0X01F0
#define DIGI1_EN 0X01F7

#define DIGI2_NAME 0X01F8
#define DIGI2_EN 0X01FF


#define REMOTE_SN 0X01D8

extern void DIGI_FUN();


extern void READ_DIGI_STU();

extern void dig_Initial();

extern uchar  SET_PATH_DIGI();

/*****************************************/


#endif