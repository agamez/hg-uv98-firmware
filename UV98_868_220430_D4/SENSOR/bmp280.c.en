
#include  <INTRINS.H> // Keil library

#include "STC8A8K64D4.H"
#include "bmp280.h"
#include "UART2.H"
#include "DELAY.H"

#include "tostring.H"
#include "IO.H"



// unsigned char QY_SHIWAN,QY_WAN,QY_QIAN,QY_BAI,QY_SHI;
// uint BMP280_QY;
// float  BMP280_TEMP;
uchar  BMP280_TEMP[10];
uchar  BMP280_QY[10];


#define BMP280_I2C_ADDR                      (0x76)		// sdo=0 address=0X76
#define BMP280_DEFAULT_CHIP_ID               (0x58)

#define BMP280_CHIP_ID_REG                   (0xD0)  /* Chip ID Register */
#define BMP280_RST_REG                       (0xE0)  /* Softreset Register */
#define BMP280_STAT_REG                      (0xF3)  /* Status Register */
#define BMP280_CTRL_MEAS_REG                 (0xF4)  /* Ctrl Measure Register */
#define BMP280_CONFIG_REG                    (0xF5)  /* Configuration Register */


// ---------------------------------------------------------
#define BMP280_PRESSURE_MSB_REG              (0xF7)  /* Pressure MSB Register */
#define BMP280_PRESSURE_LSB_REG              (0xF8)  /* Pressure LSB Register */
#define BMP280_PRESSURE_XLSB_REG             (0xF9)  /* Pressure XLSB Register */
#define BMP280_TEMPERATURE_MSB_REG           (0xFA)  /* Temperature MSB Reg */
#define BMP280_TEMPERATURE_LSB_REG           (0xFB)  /* Temperature LSB Reg */
#define BMP280_TEMPERATURE_XLSB_REG          (0xFC)  /* Temperature XLSB Reg */


#define BMP280_FORCED_MODE                   (0x01)

#define BMP280_TEMPERATURE_CALIB_DIG_T1_LSB_REG             (0x88)
// #define BMP280_PRESSURE_TEMPERATURE_CALIB_DATA_LENGTH       (24)
// #define BMP280_DATA_FRAME_SIZE               (6)

#define BMP280_OVERSAMP_SKIPPED          (0x00)
#define BMP280_OVERSAMP_1X               (0x01)
#define BMP280_OVERSAMP_2X               (0x02)
#define BMP280_OVERSAMP_4X               (0x03)
#define BMP280_OVERSAMP_8X               (0x04)
#define BMP280_OVERSAMP_16X              (0x05)

// configure pressure and temperature oversampling, forced sampling mode
#define BMP280_PRESSURE_OSR              (BMP280_OVERSAMP_8X)
#define BMP280_TEMPERATURE_OSR           (BMP280_OVERSAMP_1X)
#define BMP280_MODE                      (BMP280_PRESSURE_OSR << 2 | BMP280_TEMPERATURE_OSR << 5 | BMP280_FORCED_MODE)

#define T_INIT_MAX                       (20)	// 20/16 = 1.25 ms
#define T_MEASURE_PER_OSRS_MAX           (37)	// 37/16 = 2.3125 ms
#define T_SETUP_PRESSURE_MAX             (10)  // 10/16 = 0.625 ms

unsigned short dig_T1, dig_P1;
short dig_T2, dig_T3, dig_P2, dig_P3, dig_P4, dig_P5, dig_P6, dig_P7, dig_P8, dig_P9;
long t_fine;

bit BMP280_LINK;

uchar bmp280_chip_id = 0;
long bmp280_up = 0;
long bmp280_ut = 0;
uchar bmp280_data[24];


// 
// Delay 5ms
void BMP280_Delay5ms()// @22.1184MHz
{
    unsigned char data i, j;

    _nop_();
    _nop_();
    i = 144;
    j = 157;

    do
    {
        while (--j);
    }
    while (--i);
}




void BMP280_Delay5us()		// @@22.1184MHz
{
    unsigned char i;

    _nop_();
    i = 25;

    while (--i);
}




/**************************************
Start signal
**************************************/
void BMP280_Start()
{
    BMP280_SDA = 1;                    // Pull up the data line
    BMP280_Delay5us();                 // Delay

    BMP280_SCL = 1;                    // Pull the clock line high
    BMP280_Delay5us();                 // Delay
    BMP280_SDA = 0;                    // Generate falling edge
    BMP280_Delay5us();                 // Delay
    BMP280_SCL = 0;                    // Pull the clock line low
    BMP280_Delay5us();                 // Delay
}

/**************************************
Stop signal
**************************************/
void BMP280_Stop()
{
    BMP280_SDA = 0;                    // Pull the data line low
    BMP280_Delay5us();                 // Delay
    BMP280_SCL = 1;                    // Pull the clock line high
    BMP280_Delay5us();                 // Delay
    BMP280_SDA = 1;                    // Generate rising edge
    BMP280_Delay5us();                 // Delay
}

/**************************************
Sending an answer signal
Ingress parameter: ack (0:ACK 1:NAK)
**************************************/
void BMP280_SendACK(bit ack)
{
    BMP280_SDA = ack;                  // Write response signal
    BMP280_Delay5us();                 // Delay
    BMP280_SCL = 1;                    // Pull the clock line high
    BMP280_Delay5us();                 // Delay
    BMP280_SCL = 0;                    // Pull the clock line low
    BMP280_Delay5us();                 // Delay
}

/**************************************
Receive response signal
**************************************/
uchar BMP280_RecvACK()
{
    BMP280_SDA = 1;
    BMP280_Delay5us();
    BMP280_SCL = 1;                    // Pull the clock line high
    BMP280_Delay5us();                 // Delay

    CY = BMP280_SDA;                   // Read response signal

    BMP280_SCL = 0;                    // Pull the clock line low
    BMP280_Delay5us();                 // Delay

    if (CY == 0)
    {
        return 1;   // If there is no response, the BMP280 is not installed.
    }
    else
    {
        return 0;
    }
}

/**************************************
Send a byte of data to the IIC bus
**************************************/
uchar BMP280_SendByte(uchar dat)
{
    uchar i;

    for (i = 0; i < 8; i++)     // 8-bit counter
    {
        dat <<= 1;              // Shift out the highest bit of data
        BMP280_SDA = CY;               // Data transmission port
        BMP280_Delay5us();             // Delay
        BMP280_SCL = 1;                // Pull the clock line high
        BMP280_Delay5us();             // Delay
        BMP280_SCL = 0;                // Pull the clock line low
        BMP280_Delay5us();             // Delay
    }

    if(BMP280_RecvACK() == 1)
    {
        return 1;   // 1 = Response 0 = No response
    }
    else
    {
        return 0;
    }
}

/**************************************
Receive one byte of data from the IIC bus
**************************************/
uchar BMP280_RecvByte()
{
    uchar i;
    uchar dat = 0;

    BMP280_SDA = 1;                    // Enable internal pull-up and prepare to read data.

    for (i = 0; i < 8; i++)     // 8-bit counter
    {
        dat <<= 1;
        BMP280_SCL = 1;                // Pull the clock line high
        BMP280_Delay5us();             // Delay
        dat |= BMP280_SDA;             // Read Data
        BMP280_Delay5us();             // Delay
        BMP280_SCL = 0;                // Pull the clock line low
        BMP280_Delay5us();             // Delay
    }

    return dat;
}

uchar i2cRead(uchar add, uchar reg, uchar len, uchar *ndata)
{
    uchar i;

    BMP280_Start();            // Start signal

    if (BMP280_SendByte(add << 1) == 0)
    {
        return 3;   // Send device address + write signal
    }

// if (BMP280_SendByte(0xec)==0){return 3;} //Send device address + write signal
// ----------------------------------------------
    if (BMP280_SendByte(reg) == 0)
    {
        return 1;   // The starting address of the register to be read
    }

// ----------------------------------------------
    BMP280_Start();       // Start signal

    if (BMP280_SendByte((add << 1) + 1) == 0)
    {
        return 2;   // Send device address + read signal
    }

// if (BMP280_SendByte(0xed)==0){return 2;} //Send device address + read signal
// ----------------------------------------------
    if (len == 1)
    {
        *ndata = BMP280_RecvByte();
        ndata++;  // BUF[0] storage
        BMP280_SendACK(1);     // The last data needs to return NOACKM
    }
    else
    {
        for (i = 0; i < (len - 1); i++)
        {
            *ndata = BMP280_RecvByte();
            ndata++;  // BUF[0] storage
            BMP280_SendACK(0);     // If it is not the last data, respond with ACKM
        }

        *ndata = BMP280_RecvByte();
        ndata++;  // BUF[0] storage
        BMP280_SendACK(1);     // The last data needs to return NOACKM
    }

    BMP280_Stop();                   // Send a stop signal
    return 5;
 
}


uchar i2cWrite(uchar add, uchar reg, uchar  ndata)
{
    BMP280_Start();            // Start signal

    if (BMP280_SendByte(add << 1) == 0)
    {
        return 0;   // Send device address + write signal
    }

    // ----------------------------------------------
    if (BMP280_SendByte(reg) == 0)
    {
        return 0;   // The starting address of the register to be read
    }

    if (BMP280_SendByte(ndata) == 0)
    {
        return 0;   // The starting address of the register to be read
    }

    BMP280_Stop();                   // Send a stop signal
    return 1;
}





// Returns temperature in DegC, resolution is 0.01 DegC. Output value of "5123" equals 51.23 DegC
// t_fine carries fine temperature as global value
long bmp280_compensate_T(long adc_T)
{
// long var1, var2, T;
// var1 = ((((adc_T >> 3) - ((long)dig_T1 << 1))) * ((long)dig_T2)) >> 11;
// var2 = (((((adc_T &gt;&gt; 4) - ((long)dig_T1)) * ((adc_T &gt;&gt; 4) - ((long)dig_T1))) &gt;&gt; 12) * ((long)dig_T3)) &gt; &gt; 14;
// t_fine = var1 + var2;
// T = (t_fine * 5 + 128) >> 8;
// return T;

    double var1, var2, temperature;

    var1 = (((double) adc_T) / 16384.0 - ((double) dig_T1) / 1024.0)  * ((double) dig_T2);
    var2 = ((((double) adc_T) / 131072.0 - ((double) dig_T1) / 8192.0)   * (((double) adc_T) / 131072.0 - ((double) dig_T1) / 8192.0))   * ((double) dig_T3);
    t_fine = (long) (var1 + var2);
    temperature = (var1 + var2) / 5120.0 * 10; // Keep one decimal place
    return temperature;
}

// Returns pressure in Pa as unsigned 32 bit integer in Q24.8 format (24 integer bits and 8 fractional bits).
// Output value of "24674867" represents 24674867/256 = 96386.2 Pa = 963.862 hPa
long bmp280_compensate_P(long adc_P)
{
// double var1, var2, p;
// var1 = ((double)t_fine) - 128000;
// var2 = var1 * var1 * (double)dig_P6;
// var2 = var2 + ((var1*((double)dig_P5))&lt;&lt;17);
// var2 = var2 + (((double)dig_P4) << 35);
// var1 = ((var1 * var1 * (double)dig_P3) &gt;&gt; 8) + ((var1 * (double)dig_P2) &lt;&lt; 12);
// var1 = (((((double)1) << 47) + var1)) * ((double)dig_P1) >> 33;
// if (var1 == 0)        return 0;
// p = 1048576 - adc_P;
// p = (((p << 31) - var2) * 3125) / var1;
// var1 = (((double)dig_P9) * (p >> 13) * (p >> 13)) >> 25;
// var2 = (((double)dig_P8) * p) >> 19;
// p = ((p + var1 + var2) >> 8) + (((double)dig_P7) << 4);
// return (long)p;

    double var1, var2, pressure;

    var1 = ((double) t_fine / 2.0) - 64000.0;
    var2 = var1 * var1 * ((double) dig_P6) / 32768.0;
    var2 = var2 + var1 * ((double) dig_P5) * 2.0;
    var2 = (var2 / 4.0) + (((double) dig_P4) * 65536.0);
    var1 = (((double) dig_P3) * var1 * var1 / 524288.0    + ((double) dig_P2) * var1) / 524288.0;

    var1 = (1.0 + var1 / 32768.0) * ((double) dig_P1);

    if (var1 == 0.0)
    {
        return 0;   // avoid exception caused by division by zero
    }

    pressure = 1048576.0 - (double) adc_P;
    pressure = (pressure - (var2 / 4096.0)) * 6250.0 / var1;
    var1 = ((double) dig_P9) * pressure * pressure / 2147483648.0;
    var2 = pressure * ((double) dig_P8) / 32768.0;
    pressure = pressure + (var1 + var2 + ((double) dig_P7)) / 16.0;

    return (long)pressure / 10; // Keep one decimal place
}

void bmp280_get_up()       // read data from sensor
{
    uchar temp[6];
    // read data from sensor
    i2cRead(BMP280_I2C_ADDR, BMP280_PRESSURE_MSB_REG, 6, temp);
    bmp280_up = (long)((((long)(temp[0])) << 12) | (((long)(temp[1])) << 4) | ((long)temp[2] >> 4));
    bmp280_ut = (long)((((long)(temp[3])) << 12) | (((long)(temp[4])) << 4) | ((long)temp[5] >> 4));
}


uchar READ_BMP280()
{
// volatile temp[6];
    float t;
    float p;
    uchar i;

    BMP280_LINK = 0;

    bmp280_data[0] = 0;
    i2cRead(BMP280_I2C_ADDR, BMP280_CHIP_ID_REG, 1, bmp280_data); /* read Chip Id */

// if (bmp280_data[0]==BMP280_DEFAULT_CHIP_ID)	{	UART2_SendString("ID: 0X58 \r\n");}else	{UART2_DEBUG2(bmp280_data[0]);	UART2_SendString("NO link BMP280\r\n"); }
    if (bmp280_data[0] != BMP280_DEFAULT_CHIP_ID)
    {
        return 0;
    }

// bmp280_data[0]=	Read_BMP280_SPI_1BYTE(BMP280_CHIP_ID_REG);
// UART0_DEBUG(bmp280_data[0]);
// return 0;

// 1.BMP280 test startup process (note the status register in its datasheet):
// (1) Read the ID of BMP280, its value is equal to 0x58.
// (2) Read the values of the compensation registers.
// (3) Reset the BMP280.
// (4) Configure the data acquisition accuracy, mode, etc. of BMP280.
// (5) Give a delay and wait for the data to be collected and put into the register.
// (6) Read the collected data in the register.

    // read calibration
    i2cRead(BMP280_I2C_ADDR, BMP280_TEMPERATURE_CALIB_DIG_T1_LSB_REG, 24, bmp280_data);

    dig_T1 = bmp280_data[1] * 256 + bmp280_data[0];
    dig_T2 = bmp280_data[3] * 256 + bmp280_data[2];
    dig_T3 = bmp280_data[5] * 256 + bmp280_data[4];
    dig_P1 = bmp280_data[7] * 256 + bmp280_data[6];
    dig_P2 = bmp280_data[9] * 256 + bmp280_data[8];
    dig_P3 = bmp280_data[11] * 256 + bmp280_data[10];
    dig_P4 = bmp280_data[13] * 256 + bmp280_data[12];
    dig_P5 = bmp280_data[15] * 256 + bmp280_data[14];
    dig_P6 = bmp280_data[17] * 256 + bmp280_data[16];
    dig_P7 = bmp280_data[19] * 256 + bmp280_data[18];
    dig_P8 = bmp280_data[21] * 256 + bmp280_data[20];
    dig_P9 = bmp280_data[23] * 256 + bmp280_data[22];

// UART2_SendString(&quot;-------\r\n&quot;);
// UART0_DEBUG(you_T1); UART0_DEBUG(you_T2); UART0_DEBUG(-dig_T3);
// UART0_DEBUG(you_P1); UART0_DEBUG(-dig_P2); UART0_DEBUG(you_P3);
// UART0_DEBUG(you_P4); UART0_DEBUG(you_P5); UART0_DEBUG(-dig_P6);
// UART0_DEBUG(dig_P7);	UART0_DEBUG(-dig_P8);	UART0_DEBUG(-dig_P9);
// UART2_SendString(&quot;-------\r\n&quot;);

    // set oversampling + power mode (forced), and start sampling
    i2cWrite(BMP280_I2C_ADDR, BMP280_CTRL_MEAS_REG, BMP280_MODE);

// Delay_time_50ms(1);
    BMP280_Delay5ms();

    bmp280_get_up();       // read data from sensor
    t = bmp280_compensate_T(bmp280_ut);	   // UART2_SendString(&quot;TEMP: &quot;); UART0_DEBUG(t); UART2_SendString(&quot; C\r\n&quot;);
    p = bmp280_compensate_P(bmp280_up);	// UART2_SendString(&quot;press: &quot;); UART0_DEBUG(p); UART2_SendString(&quot; hPa\r\n&quot;);

// BMP280_QY=(uint)p;

    for(i = 0; i < 10; i++)
    {
        BMP280_TEMP[i] = 0x00;
    }

    if (t < 0)
    {
        BMP280_TEMP[0] = '-';
        t = -t;
    }
    else
    {
        BMP280_TEMP[0] = ' ';
    }

    tostring((uint)t);
    BMP280_TEMP[1] = bai;
    BMP280_TEMP[2] = shi;
    BMP280_TEMP[3] = '.';
    BMP280_TEMP[4] = ge;
    BMP280_TEMP[5] = 0x00;

    if (BMP280_TEMP[1] == '0')
    {
        BMP280_TEMP[1] = ' ';
    }



// n=0;
// while(BMP280_TEMP[1]==&#39;0&#39;) //The first 0 is blanked, at least one bit is retained
// {
// for(i=1;i<8;i++) 	{ BMP280_TEMP[i]=BMP280_TEMP[i+1]; }
// n++; if (n>2){break;}
// }

// UART2_SendString(&quot;BMP280: &quot;); UART2_SendString(BMP280_TEMP); UART2_SendString(&quot; °C &quot; ); //UART2_SendString(&quot;\r\n&quot;);


    for(i = 0; i < 10; i++)
    {
        BMP280_QY[i] = 0x00;
    }

    tostring((uint)p);

    BMP280_QY[0] = wan;
    BMP280_QY[1] = qian;
    BMP280_QY[2] = bai;
    BMP280_QY[3] = shi;
    BMP280_QY[4] = '.';
    BMP280_QY[5] = ge;
    BMP280_QY[6] = 0;

    if (BMP280_QY[0] == '0')
    {
        BMP280_QY[0] = ' ';
    }

// UART2_SendString(BMP280_QY ); UART2_SendString(&quot; hPa\r\n&quot; );

    BMP280_LINK = 1;
    return 1;
}

