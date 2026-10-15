

#ifndef _ADC_H_
#define _ADC_H_


/************* External function and variable declaration*****************/
extern unsigned int	adc10_start(unsigned char channel);
extern void	adc_Initial();
extern void READ_ADC();			 // Read voltage value
extern unsigned char DY[8];

/*****************************************/
extern void CHECK_BATT();	 // Check the battery //Minimum working voltage 3.36V
extern unsigned char  CHECK_BATT_POWER_OFF();	 // Check the battery //Minimum working voltage is 3.05V, if it is lower than 3.05V, the device will shut down to protect the battery

#endif