// 8K CPU
// UV98_868_220430_D4 Firmware


// Automatically identify 4-digit or 5-digit GPS/Beidou latitude and longitude data


// Corrected the grid display problem under latitude S and longitude W




// Corrected negative altitude display
// The number of GPS satellites is filled with 0
// Initial mileage automatic initialization
// Smart beacon, minimum interval 10 seconds


// Weather data is based on weather icons


// BT sends beacon, adds prompt tone
// 1. Added a shortcut key. Long press 1 on the GPS interface to quickly clear the mileage.
// 2. Added shortcut keys. Long press 2 on the GPS interface.
// Quickly set the current GPS location as a fixed station location
// 2. GPS positioning is required

// Supports transmitting beacons and relaying beacons. Only Bluetooth output is available.

// Identify meteorological data and display the meteorological interface

// Added Bluetooth menu 4, data analysis output
// Receive only beacon data analysis
// Only valid under UI\GPWPL\KISS ASC option

// Added weather unit setting interface



// APRS Upgrade Package_HG_UV98_20190212 Firmware

// Added dynamic navigation (dynamic distance, direction)

// Bluetooth support to control the spinner
// Fixed beacons include heading, speed, altitude

// The maximum length of the received beacon is increased from 120 bytes to 190 bytes
// Allows storing beacons without specific latitude and longitude

// Correct navigation angle and relative angle
// Added fixed longitude and latitude as navigation points
// Added navigation distance, true north direction, and relative direction at the top of the GPS interface

// Added global orientation display mode 0=NSWE 1=00-12 2=00-36
// After adjusting the mileage is turned off, the mileage on the GPS interface is always 0
// Fixed the distance measurement issue for users in the southern hemisphere
// List callsign + distance + direction, try it out


// Corrected the overflow of the path when it is too long
// Add mileage display to GPS interface
// Default 2nd Icon P
// Default mileage automatic accumulation
// Bluetooth synchronous output setting data



// Resistor voltage divider 100K 51K
// Side 2 button press A09
// Alarm button pressed A10
// Release the PTT button A11
// Read version adjusted to A50
// The read serial number is adjusted to A51



// Bluetooth off other debugging data
// If the distance is less than 10,000 meters, 0.001KM will be displayed.
// 


// Default reception, transmission 6DB
// P3.6 P2.7 Push-pull

// GPS status change notification tone
// The first 7 rows are arranged in order
// In line 8, if it is ---, the arrow will display ---.
// If it is a number, show an arrow

// GPS Display Grid
// GPS 3 formats display
// PTT is pulled high, E01 is sent, beacon is sent, PTT is pulled low







// Interface adjustments
// Data sent to the host is automatically delayed for processing





// Fixed Bluetooth reception issue





// 8K CPU
// APRS_8K_20180922_D test firmware
// After Bluetooth writes the settings, 512 bytes are sent to the host at the same time


 

// AT11\r\n //Read version number
// AT12\r\n //Read CPU ID

// The CPU is powered on for 500MS and initialized twice in succession
// Adjust 865 reset method
// No test tone when powering on
// AT+TONE=1200 Bass test
// AT+TONE=2200 high tone test
// AT+TONE=OFF Turn off the high and low tone test


 



// Support RF remote switch relay
// Default password 123456

// Command A0 turns off DIGI 1
// Command A1 Turn on DIGI 1
// Command B0 turns off DIGI 2
// Command B1 Turn on DIGI 2
// Instruction R0 reset restart

// Support custom TX PATH 1 name
// Support custom TX PATH 2 name

// Support custom DIGI 1 alias
// Support custom DIGI 2 aliases


// With hyperbolic interface
// Support GPS altitude and speed curves
// Support temperature and pressure curves
// After GPS positioning, click on speed and altitude

// APRS_51G3_4K_20180408 test firmware
// Brush support curve interface
// Support GPS altitude and speed curves
// After GPS positioning, click on speed and altitude


// APRS_51G3_4K_20180221 test firmware
// Fixed decompression bug


// Mobile beacons support MIC-E encoding
// AT+MICE=ON  AT+MICE=OFF

// No mileage is counted when parking in P
// Accumulated mileage while moving, range 0-5000.0KM
// If the mileage exceeds 5000km, the mileage will be reset to zero and counted again.
// Mileage unit letters (letters AZ, az)
// One letter 5000KM

// If the satellite signal is less than 4 stars, no beacon will be sent
// Prevent satellite signals from drifting due to interference

// APRS_51G3_4K_20180211A Firmware
// Bluetooth adds 2 outputs
// GPS synchronized output GPRMC GPGGA
// GPS+UI synchronous mixed output GPRMC GPGGA and decoded data
// $GPTXT,BH4TDV-7&gt;RTUTV3,WIDE1-1:`.;u &gt;/&quot;3r} 8.3V 25.6C 1016.3pa KG928*70
// May affect touch sensitivity and decoding efficiency


// Memory Arrangement UART1 128