

#ifndef _ADC_H_
#define _ADC_H_


/*************  外部函数和变量声明 *****************/
extern unsigned int	adc10_start(unsigned char channel);
extern void	adc_Initial();
extern void READ_ADC();			 //读取电压值
extern unsigned char DY[8];

/*****************************************/
extern void CHECK_BATT();	 //检查电池	 //工作最低电压3.36V
extern unsigned char  CHECK_BATT_POWER_OFF();	 //检查电池	 //工作最低电压3.05V,低于3.05V关机，保护电池

#endif