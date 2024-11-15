
#include "STC8A8K64D4.H"
#include "STC_EEPROM.H"

#include "tostring.H"
#include "UART2.H"
#include "UART1.H"

#include<string.h>


uchar wan, qian, bai, shi, ge;
void tostring(uint val); // Convert to 3 bytes

uchar str_txt[10];


uchar CPU_ID[16];	 // Read CPU serial number

uchar TEMP_Call[10];	// Temporary call sign





uchar	Ascii2Hex(uchar dat)
{
  if (dat>0x39)	{dat=dat-'A'+10;}else{dat=dat-0X30;}
	return dat;
}



uchar	Hex2Ascii(uchar dat)
{
    dat &= 0x0f;

    if(dat <= 9)	return (dat + '0');

    return (dat - 10 + 'A');
}

 








void tostring(uint val) // Convert to 3 bytes
{
	wan=val/10000%10+0x30; 	qian=val/1000%10+0x30;	bai=val/100%10+0x30; 	shi=val/10%10+0x30;	ge=val%10+0x30;

	str_txt[0]=wan;	  str_txt[1]=qian;  str_txt[2]=bai;  str_txt[3]=shi;  str_txt[4]=ge;  str_txt[5]=0;

}
 
// void tostring_dot(uint val) //Convert to 3 bytes with 1 decimal point
// {
// wan=val/10000%10+0x30; 	qian=val/1000%10+0x30;	bai=val/100%10+0x30; 	shi=val/10%10+0x30;	ge=val%10+0x30;
// str_txt[0]=wan;	  str_txt[1]=qian; str_txt[2]='.'; str_txt[3]=bai;  str_txt[4]=shi;   str_txt[5]=ge;  str_txt[6]=0;
// }



void disp_Hex2Ascii2(uchar dat)
{
    UART2_SendData(Hex2Ascii(dat >> 4));
    UART2_SendData(Hex2Ascii(dat));
    UART2_SendString(" ");
}

 

void DEBUG_KISS(uchar *p, uint len)
{
    uint i;
// UART2_SendString(&quot;KISS LEN: &quot;);
// UART2_DEBUG(only);

    disp_Hex2Ascii2(0xC0);
    disp_Hex2Ascii2(0x00);

    for (i = 0; i < len; i++)
    {
        disp_Hex2Ascii2(*(p + i));
    }

    disp_Hex2Ascii2(0xC0);
    UART2_SendString(" \r\n");

 
}



void READ_CPU_ID()	 // Read CPU serial number
{
    uchar	idata *ip;		// Note the pointer, pointing to the back RAM
// flying code *cp;
    uchar i, TEMP;
    ip = 0xF1;	// CPU serial number is stored in internal RAM F1H-F7H

    for(i = 0; i < 7; i++)
    {
        TEMP = *ip;
        ip++;
        CPU_ID[i * 2] = Hex2Ascii(TEMP >> 4);
        CPU_ID[i * 2 + 1] = Hex2Ascii(TEMP);
    }

    CPU_ID[14] = 0x00;
}






void READ_TEMP_CALL(uint call_eerom_add, uint SSID_eerom_add)	 // Read out the call sign of the target to be compared
{
    uchar	i, n, w;

// eerom_add=0x0008;
    w = 0;	   	// Insert call sign, up to 6 digits

    for(i = 0; i < 6; i++)
    {
        n = EEPROM_Buffer[call_eerom_add + i];

        if (n == 0x00)
        {
            break;
        }

        TEMP_Call[w] = n;
        w++;	 // Store call sign field at the same time
    }


    TEMP_Call[w] = '-';
    w++;			// insert

// i=EEPROM_read_one(0x000F);		//插入SSID

    i = EEPROM_Buffer[SSID_eerom_add];

    if (i < 10)	// 0-9
    {
        TEMP_Call[w] = i % 10 + 0x30;    // insert
        w++;
    }
    else	   // 10-99
    {
        TEMP_Call[w] = i / 10 % 10 + 0x30;    // insert
        w++;
        TEMP_Call[w] = i % 10 + 0x30;
        w++;
    }

    TEMP_Call[w] = 0x00;	// End of call sign field
}


 
