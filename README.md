 [![Wiki](https://github.com/SV-Zanshin/BME680/blob/master/Images/Documentation-wiki.svg)](https://github.com/SV-Zanshin/BME680/wiki)
 # FridgeLogger Program
*Arduino* sketch to log temperature and current consumption data to a microSD card. The statistics collected by this sketch are used to compute various parameters for a refrigerator or freezer. 

## Hardware
Rather than use a prebuilt Arduino, I used an Atmel ATMega 328PU chip with the Arduino bootloader and wired it up on a breadboard with a couple of hardware components, most of which aren't really necessary to the board's function.  I was curious about the Adafruit INA219 breakout board and wired that to the high-side power supply, then ran the power supply through an Adafruit Boost 500 to regulate the power supply, since I'm using rechargeable NiMh batteries and want to keep a nice constant 5V to the breadboard.  A Sparkfun DS1307 breakout board supplies the date and time for the log file. Rather than use a FTDI chip or JTAG programming, I wired in a (removable) Adafruit Bluefruit EZ-Link bluetooth wireless device which allows me to monitor the reading remotely as well as to program the board.
Logging is done using an Adafruit SD-Card breakout board.

<img src="https://github.com/SV-Zanshin/FridgeLogger/blob/master/Images/FridgeLogger_bb.png" width="100%"/>

## Software
The standard Arduino libraries are used as well as the specific ones for the Sparkfun/Adafruit breakout boards. In addition to these libraries the project makes use of the [DS-Family library](https://github.com/SV-Zanshin/DSFamily) to simplify managing numerous DS-Family type thermometers

## Output Files
The output is written to files on the SD-Card, with a new file started each day. The filename format is "GFYYMMDD.CSV" and the format is a DOS comma-separated variable file. The temperature readings in the file aren't in degrees Celsius, but in the 12-bit internal format used by the DS-Family of devices. Multiply the temperature readings by 0.0625°C to get degrees Celsius.

![Zanshin Logo](https://www.sv-zanshin.com/r/images/site/gif/zanshinkanjitiny.gif) <img src="https://www.sv-zanshin.com/r/images/site/gif/zanshintext.gif" width="75"/>
