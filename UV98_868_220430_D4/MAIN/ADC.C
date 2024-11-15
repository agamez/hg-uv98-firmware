#include "STC8A8K64D4.H"
#include "ADC.H"
#include "io.H"
#include "DELAY.H"
// #include "UART1.H"
#include "tostring.H"



#include  <INTRINS.H> // Keil library


/********* AD ***********/
#define ADC_ON		0x80
#define ADC_START	0x40	// Automatically clear to 0
#define ADC_FLAG	0x20	// Software clear

unsigned char DY[8];
void LED_SHOW(uchar n);


void adc_Initial()	   // ADC Initialization
{
    P1M1 |= 0x80;		// P 1.7 High impedance for ADC

    ADCCFG = 0x0f; // Set ADC clock to system clock/2/16/16
    ADC_CONTR = ADC_ON | 7;	 // Enable the ADC module, pay attention to the problem that ADC initialization affects P1.0 RXD2
}




/********************* Do an ADC conversion*******************/
uint	adc10_start(uchar channel)	// channel = 0~7
{
    ADC_RES = 0;
    ADC_RESL = 0;
    ADC_CONTR = (ADC_CONTR & 0x80) | ADC_START | channel;   // Start AD conversion

    _nop_();
    _nop_();
    _nop_();
    _nop_();

    while(!(ADC_CONTR & ADC_FLAG));  // Query ADC completion flag

    ADC_CONTR &= ~ADC_FLAG;	 // Clear the conversion completion flag
    return	(ADC_RES * 16) + (ADC_RESL >> 4);
}

void READ_ADC()			 // Read P1.7 //Read voltage value
{
    float temp, ad;
    uint vdd;
    uchar i;
    ad =	adc10_start(7);		 // Read P1.7
// vdd= ad*3.30*100*11/1024;
    temp = ad * 3.3 * 2.96 / 40.96;

// temp= ad*3.3*100*/40.96;

    vdd = (uint)temp;
    tostring(vdd);

    if (vdd < 1000) // Insertion voltage
    {
        i = 0;
        DY[i++] = bai;
        DY[i++] = '.';
        DY[i++] = shi;
    }
    else
    {
        i = 0;
        DY[i++] = qian;
        DY[i++] = bai;
        DY[i++] = '.';
        DY[i++] = shi;
    }

    DY[i++] = 0X00; // End
}

