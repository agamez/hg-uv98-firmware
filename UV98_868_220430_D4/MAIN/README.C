// 8K CPU
// UV98_868_220430_D4 firmware

// Automatic recognition of 4-digit or 5-digit GPS/Beidou longitude and latitude data

// Fixed the problem of grid display under latitude S and longitude W

// Fixed negative altitude display
// GPS satellite number zero padding
// Initial mileage auto-initialization
// Intelligent beacon, minimum interval of 10 seconds

// Meteorological data is based on weather icons

// BT sends beacon, adds prompt sound
// 1. Added shortcut key, long press 1 in GPS interface, quickly clear mileage to 0
// 2. Added shortcut key, long press 2 in GPS interface,
// Quickly set the current GPS position as the fixed station position
// The above two require valid GPS positioning

// Supports sending beacon, relay forwarding beacon via Bluetooth output only

// Recognize meteorological data and display meteorological interface

// Added Bluetooth menu 4, data analysis output
// Only receive beacon data analysis
// Only valid under UI\GPWPL\KISS ASC options

// Added meteorological unit setting interface

// APRS upgrade package_HG_UV98_20190212 firmware

// Added dynamic navigation (dynamic distance, azimuth)

// Bluetooth supports controlling rotator
// Fixed beacon includes heading, speed, altitude

// Allowed maximum length of received beacon extended from 120 bytes to 190 bytes
// Allowed storing beacons without specific longitude and latitude

// Fixed navigation angle and relative angle
// Added using fixed longitude and latitude as navigation point
// Added GPS interface top display of navigation distance, true north bearing, movement relative bearing

// Added global bearing display mode 0=NSWE 1=00-12 2=00-36
// Adjust mileage to always be 0 after turning off
// Fixed distance measurement issue for southern hemisphere users
// List callsign + distance + bearing, new trial

// Fixed length overflow display issue
// Added mileage display in GPS interface
// Default second icon P
// Default automatic accumulation of mileage
// Bluetooth sync output setting data

// Resistor voltage divider 100K 51K
// Side key 2 pressed A09
// Alarm key pressed A10
// PTT key released A11
// Read version adjusted to A50
// Read serial number adjusted to A51

// Bluetooth closes other debug data
// Distance less than 10,000 meters, display 0.001KM

// Default receive, transmit 6DB
// P3.6 P2.7 push-pull

// GPS status change prompt sound
// The first 7 lines are arranged in order
// The 8th line, if it is --- , the arrow shows ---,
// If it is a number, it shows an arrow

// GPS grid display
// GPS 3 format display
// PTT pulled high, send E01, send beacon, PTT pulled low

// Interface adjustments accordingly
// Data sent to the host, automatically delayed processing

// Fixed Bluetooth reception issue

// 8K CPU
// APRS_8K_20180922_D test firmware
// After writing Bluetooth settings, simultaneously send 512 bytes to the host

// AT11\r\n   // Read version number
// AT12\r\n   // Read CPU ID

// CPU power on for 500MS, continuous initialization twice
// Adjusted 865 reset method
// No test sound at startup
// AT+TONE=1200 Low tone test
// AT+TONE=2200 High tone test
// AT+TONE=OFF High and low tone test off

// Support RF remote switch relay
// Default password 123456

// Command A0 Close DIGI 1
// Command A1 Open DIGI 1
// Command B0 Close DIGI 2
// Command B1 Open DIGI 2
// Command R0 Reset and restart

// Support custom TX PATH 1 name
// Support custom TX PATH 2 name

// Support custom DIGI 1 alias
// Support custom DIGI 2 alias

// Matched hyperbolic interface
// Support GPS altitude and speed curve
// Support temperature and pressure curve
// After GPS positioning, click speed and altitude

// APRS_51G3_4K_20180408 test firmware
// Support curve interface
// Support GPS altitude and speed curve
// After GPS positioning, click speed and altitude

// APRS_51G3_4K_20180221 test firmware
// Fixed decompression bug

// Mobile beacon supports MIC-E encoding
// AT+MICE=ON  AT+MICE=OFF

// Do not accumulate mileage when parking P
// Accumulate mileage when moving, range 0-5000.0KM
// Over 5000KM, reset and recount mileage
// Mileage unit letter (letter A-Z,a-z)
// One letter represents 5000KM

// Do not send beacon when satellite signal is less than 4 stars
// Prevent large drift in position caused by satellite signal interference

// APRS_51G3_4K_20180211A firmware
// Bluetooth adds 2 output items
// GPS sync output GPRMC GPGGA
// GPS+UI sync mixed output GPRMC GPGGA and decoded data
// $GPTXT,BH4TDV-7>RTUTV3,WIDE1-1:`.;u >>>/3r} 8.3V 25.6C 1016.3pa KG928*70
// May affect touch sensitivity and decoding efficiency

// Memory arrangement UART1 128
