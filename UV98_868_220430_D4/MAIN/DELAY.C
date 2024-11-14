
//#include "STC12C5A32S2.H"
#include	"STC8A8K64D4.H"

#include "DELAY.H"






void Delay1ms()		//@22.1184MHz
{
    unsigned char i, j;

    i = 29;
    j = 183;

    do
    {
        while (--j);
    }
    while (--i);
}



void Delay_time_1ms(uint n)	  //使用TIME1做定时器,单位25ms, 系数1-65535
{
    if (n == 0)
    {
        return;
    }

    while (n--)
    {
        Delay1ms();		//@22.1184MHz
    }
}

void Delay25ms()		//@22.1184MHz
{
    unsigned char i, j, k;

    i = 3;
    j = 207;
    k = 28;

    do
    {
        do
        {
            while (--k);
        }
        while (--j);
    }
    while (--i);
}




void Delay_time_25ms(uint n)	  //使用PCA定时器,单位20ms, 系数0-65535
{
    if (n == 0)
    {
        return;
    }

    while (n--)
    {
        Delay25ms();
    }
}



