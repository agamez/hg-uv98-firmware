
#include "STC8A8K64D4.H"
#include "GPS_CHECKSUM.H"
// #include "UART4.H"
#include "UART3.H"

#include <string.h>

uchar checksum_A,	 checksum_B ;


uchar	Hex2Ascii_A(uchar dat)
{
    dat &= 0x0f;

    if(dat <= 9)	return (dat + '0');

    return (dat - 10 + 'A');
}

uchar checksum_gps(char *s)
{
    uchar i;
    uchar  checksum;
    uchar checksum_A,	 checksum_B ;

    s++;	 // Does not contain $
    checksum = *s;

    for (i = 0; i < 126; i++)
    {
        s++;

        if (*s == '*')
        {
            break;   // Not included *
        }

        checksum = checksum ^ (*s);
    }

    s++;
    checksum_A =	*s;
    s++;
    checksum_B =	*s;

// UART1_SendData(checksum_A);	UART1_SendData(checksum_B);
// UART1_SendData(Hex2Ascii_A(checksum>>4));
// UART1_SendData(Hex2Ascii_A(checksum));
// UART1_SendData(0x0d);
// UART1_SendData(0x0a);

    if (checksum_A != Hex2Ascii_A(checksum >> 4))
    {
        return 0;
    }

    if (checksum_B != Hex2Ascii_A(checksum ))
    {
        return 0;
    }

    return 1;
}


uchar sum_gps()
{
    if (checksum_gps(UART3_GPRMC_DATA) == 0)
    {
        return 0;
    }

    if (checksum_gps(UART3_GPGGA_DATA) == 0)
    {
        return 0;
    }

    return 1;
}


