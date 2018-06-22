/*******************************************************************************************************************
** Program to monitor refrigerator data using a minimal breadboarded Atmel ATMega 328 processor with the Arduino  **
** bootloader and development environment. I used some of the hardware lying around to put this project together. **
**                                                                                                                **
** Program code can be found on GitHub at https://github.com/SV-Zanshin/FridgeLogger and board schematics at      **
** https://github.com/SV-Zanshin/FridgeLogger/blob/master/Images/FridgeLogger_bb.png                              **
**                                                                                                                **
** The hardware used is as follows:                                                                               **
**     - ATMega 328-PU Processor using the Arduino Bootloader                                                     **
**     - Adafruit INA219 Breakout board attached to the input power supply                                        **
**     - Adafruit PowerBoost 500 breakout board using the input power supply and regulating it to 5.2V            **
**     - Sparkfun DS1307 RTC (Real-Time-Clock) with a coin battery backup.                                        **
**     - Adafruit Micro-SD Breakout board from Adafruit with level shifter down to 3V                             **
**     - Adafruit Bluefruit Ez-Link bluetooth device                                                              **
**     - Many DS18B20's (15) in a 1-Wire microLAN daisy-chain                                                     **
**                                                                                                                **
** The project makes use of the "DSFamily.h" library, which makes using a large number of DS-Family type          **
** thermometers a lot simpler and also takes care of the large amount of memory needed to reserve DS-Family       **
** addresses by using the Atmel's EEPROM. The library is available on GitHub and the newest version can be down-  **
** loaded at https://github.com/SV-Zanshin/DSFamily                                                               **
**                                                                                                                **
** This autonomous project measures the temperature in various locations in a refrigerator. At least one of the   **
** DS18B20s should be attached as close as possible to the evaporator plate.  The data is written to a text file  **
** on the MicroSD card in .CSV format for later analysis in Excel. This is part of a bigger project to develop a  **
** PID-based energy saving refrigeration control for electric cooling systems on boats and motor homes. The goal  **
** of these measurements is to get an algorithm for various aspects of the cooling and warming cycle:             **
** 1. Determine temperature jumps associated with a "door open" event - but not using a door or light sensor      **
** 2. Determine temperature curves associated with (a) removals (b) warm item insertions and (c) no content change**
** 3. Determine compressor cycle times using the evaporator plate temperature                                     **
**                                                                                                                **
** Although programming for the Arduino and in c/c++ is new to me, I'm a professional programmer and have learned,**
** over the years, that it is much easier to ignore superfluous comments than it is to decipher non-existent ones;**
** so both my comments and variable names tend to be verbose. The code is written to fit in the first 80 spaces   **
** and the comments start after that and go to column 117 - allowing the code to be printed in A4 landscape mode. **
**                                                                                                                **
** This program is free software: you can redistribute it and/or modify it under the terms of the GNU General     **
** Public License as published by the Free Software Foundation, either version 3 of the License, or (at your      **
** option) any later version. This program is distributed in the hope that it will be useful, but WITHOUT ANY     **
** WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the   **
** GNU General Public License for more details. You should have received a copy of the GNU General Public License **
** along with this program.  If not, see <http://www.gnu.org/licenses/>.                                          **
**                                                                                                                **
** Vers.   Date       Developer           Comments                                                                **
** ======= ========== =================== ======================================================================= **
** 1.0.1   2016-12-28 Arnd@SV-Zanshin.Com Added error checking for DS1307 returned date/time                      **
** 1.0.0   2016-12-27 Arnd@SV-Zanshin.Com Implemented                                                             **
** 1.0.0b2 2016-12-25 Arnd@SV-Zanshin.Com Added SD-Card                                                           **
** 1.0.0b1 2016-12-25 Arnd@SV-Zanshin.Com Initial coding.                                                         **
**                                                                                                                **
*******************************************************************************************************************/
#include <Adafruit_INA219.h>                                                  // INA219 Volts and Amps data       //
#include <RTClib.h>                                                           // DS1307 Real-Time-Clock commands  //
#include <SD.h>                                                               // SD-Card input and output         //
#include <avr/sleep.h>                                                        // Sleep mode Library               //
#include <DSFamily.h>                                                         // DS Thermometers calls and methods//
/*******************************************************************************************************************
** Declare program Constants                                                                                      **
*******************************************************************************************************************/
const uint8_t  GREEN_LED_PIN         =      5;                                // Green LED (nonstandard location) //
const uint8_t  CS_PIN                =     10;                                // Chip-Select pin for SD-Card      //
const uint8_t  INA219_I2C_ADDRESS    =   0x40;                                // I2C address = 1000000 (A0+A1=GND)//
const uint8_t  SPRINTF_BUFFER_SIZE   =     64;                                // Bytes in sprintf buffer          //
const uint16_t MEASUREMENT_MILLIS    =   5000;                                // Interval to perform measurements //
const uint16_t TIMER1_265DIV_10HZ    =  59286;                                // 65536 - (16Mhz/256/10)           //
const uint32_t SERIAL_SPEED          = 115200;                                // Use fast serial speed            //
const uint8_t  ONE_WIRE_PIN          =      4;                                // Digital Pin 2 for 1-Wire MicroLAN//
const char     FILE_PREFIX[3]        =   "GF";                                // Define output file prefix        //
/*******************************************************************************************************************
** Declare global variables and instantiate classes                                                               **
*******************************************************************************************************************/
Adafruit_INA219   INA219;                                                     // INA219 class instantiation       //
DSFamily_Class    DSFamily(ONE_WIRE_PIN);                                     // Instantiate the DSFamily class   //
RTC_DS1307        rtc;                                                        // Real-time-clock instantiation    //
char              sprintfBuffer[SPRINTF_BUFFER_SIZE];                         // Define buffer for sprintf        //
DateTime          now;                                                        // Define class to hold date & time //
File              dataFile;                                                   // Instantiate the SD file class    //
volatile uint16_t INA219_Readings   = 0;                                      // Number of INA219 readings done   //
volatile float    INA219_ShuntV     = 0;                                      // Sum for averaging Shunt Voltage  //
volatile float    INA219_BusV       = 0;                                      // Sum for averaging Bus Voltage    //
volatile float    INA219_mA         = 0;                                      // Sum for averaging MilliAmperes   //
uint16_t          ThermometersFound = 0;                                      // Store device detected count      //
uint8_t           Today             = UINT8_MAX;                              // Current day-of-week              //
/*******************************************************************************************************************
** Declare prototypes for all functions used                                                                      **
*******************************************************************************************************************/
void    setup();                                                              // Called once on power-up/restart  //
void    loop();                                                               // Called repeatedly after setup()  //
void    sleep(const uint16_t SleepMilliseconds);                              // Save on power consumption        //
void    openFile();                                                           // Open SD-Card file, if necessary  //
/*******************************************************************************************************************
** This is the function which collects the INA219 voltage and amperage statistics, it is triggered by an overflow **
** interrupt on TIMER1 which is set to about 10Hz (10 times a second) using the value of TIMER1_265DIV_10HZ. Once **
** triggered, it re-enables interrupts, which are turned off by default inside interrupt vectors, and calls the   **
** INA219 library commands to retrieve the voltage and current information and add them to the global variables.  **
*******************************************************************************************************************/
ISR(TIMER1_OVF_vect) {                                                        // Timer1 overflow service routine  //
  sei();                                                                      // Enable interrupts for INA219     //
  INA219_ShuntV += INA219.getShuntVoltage_mV();                               // Add reading to sum               //
  INA219_BusV   += INA219.getBusVoltage_V();                                  // Add reading to sum               //
  INA219_mA     += INA219.getCurrent_mA();                                    // Add reading to sum               //
  cli();                                                                      // Disable interrupts again         //
  INA219_Readings++;                                                          // Increment readings counter       //
  TCNT1 = TIMER1_265DIV_10HZ;                                                 // Set timer again                  //
} // of method timer1 overflow vector                                         //                                  //
/*******************************************************************************************************************
** Method sleep() will put the system into a low-energy mode for the given number of milliseconds. Any wake-ups   **
** such as interrupt that occur during this sleep period are handled and then the system goes back to sleep.      **
*******************************************************************************************************************/
void sleep(const uint16_t SleepMilliseconds) {                                //                                  //
  uint32_t SleepUntil = millis() + SleepMilliseconds;                         // Set time we want to sleep until  //
  bool     LEDState   = digitalRead(GREEN_LED_PIN);                           // Store value of pin               //
  digitalWrite(GREEN_LED_PIN,false);                                          // Turn off LED while we sleep      //
  ADCSRA = 0;                                                                 // turn off the ADC module          //
  while(millis()<=SleepUntil) {                                               // Put the system into sleep mode   //
    set_sleep_mode(SLEEP_MODE_IDLE);                                          // Set the minimum power savings    //
    cli();                                                                    // disable interrupts               //
    sleep_enable();                                                           // Enable sleep, but don't sleep yet//
    sei();                                                                    // Re-enable interrupts             //
    sleep_cpu ();                                                             // Sleep within 3 clock cycles      //
  } // of while we don't have anything to do                                  // of re-enabling interrupts        //
  digitalWrite(GREEN_LED_PIN,LEDState);                                       // Restore LED state                //
  ADCSRA = B10000000;                                                         // turn the ADC module back on      //
  digitalWrite(GREEN_LED_PIN,HIGH);                                           // Turning on LED                   //
} // of method sleep()                                                        //                                  //
/*******************************************************************************************************************
** Method openFile() will open up a file with today's date for writing on the SD-Card. the filename is named as   **
** "GF{yy}{mm}{dd}.CSV" and it is a comma-separated file for use in Excel.                                        **
*******************************************************************************************************************/
void openFile() {                                                             //                                  //
  if ( now.day() != Today || !dataFile) {                                     // If the day has changed           //
    if (dataFile) dataFile.close();                                           // close the file, if it was open   //
    Today = now.day();                                                        // store current day of the month   //
    sprintf(sprintfBuffer,"%s%02d%02d%02d.CSV", FILE_PREFIX,                  // Format an 8.3 type filename      //
            now.year()-2000,now.month(),now.day());                           //                                  //
    dataFile = SD.open(sprintfBuffer, FILE_WRITE);                            // creates file if it doesn't exist //
    if (dataFile) {                                                           // If file was successfully opened  //
      if (dataFile.position()==0) {                                           // Then if the file was empty, add  //
        dataFile.print(F("Date,Time,Input Volts,milliAmps"));                 // the header line                  //
        for(uint8_t i=0;i<DSFamily.ThermometersFound;i++) {                   // Add each thermometer found to    //
          sprintf(sprintfBuffer,",Temp%02d",i);                               // the output                       //
          dataFile.print(sprintfBuffer);                                      //                                  //
        } // of loop for each thermometer found                               //                                  //
        dataFile.println();                                                   // Terminate line                   //
      } // of if-then we have an empty file                                   //                                  //
    } else {                                                                  //                                  //
      Serial.print(F("Error opening \""));                                    //                                  //
      Serial.print(sprintfBuffer);                                            //                                  //
      Serial.print(F("\"\n"));                                                //                                  //
    } // if-then-else file could be opened                                    //                                  //
  } // we have a new day and need to open a new file                          //                                  //
} // of method openFile()                                                     //                                  //
/*******************************************************************************************************************
** Method Setup(). This is an Arduino IDE method which is called first upon initial boot or restart. It is only   **
** called one time and all of the variables and other initialization calls are done here prior to entering the    **
** main loop for data measurement and storage.                                                                    **
*******************************************************************************************************************/
void setup() {                                                                //                                  //
  pinMode(GREEN_LED_PIN, OUTPUT);                                             // Define the green LED as an output//
  pinMode(CS_PIN,OUTPUT);                                                     // Declare CS_PIN as an output pin  //
  Serial.begin(SERIAL_SPEED);                                                 // Start serial communications      //
  ThermometersFound = DSFamily.ScanForDevices();                              // Search for thermometers on 1-Wire//
  Serial.print(F("\n\nFridgeLogger V1.0.1b\n"));                              // Display program information      //
  INA219.begin(INA219_I2C_ADDRESS);                                           // Start the INA219 breakout        //
  INA219.setCalibration_16V_400mA();                                          // Lowest voltage / highest accuracy//
  rtc.begin();                                                                // Start the RTC interface          //
  if ( !rtc.isrunning()) rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));      // Set clock to compile date/time   //
  now = rtc.now();                                                            // Get the Date and Time from DS1307//
  Today = now.day();                                                          // Store the current day of month   //
  sprintf(sprintfBuffer,"Now: %04d-%02d-%02d %02d:%02d:%02d\n",now.year(),    // Put together the display string  //
          now.month(), now.day(),now.hour(),now.minute(),now.second());       //                                  //
  Serial.print(sprintfBuffer);                                                // Output the display string        //
  if (ThermometersFound==0)                                                   // If we have searched but not found//
    Serial.println(F("No thermometers detected!"));                           // show the error message           //
  else {                                                                      // or                               //
    Serial.print(F("Found "));                                                // Display devices found            //
    Serial.print(ThermometersFound);                                          //                                  //
    Serial.print(F(" devices.\n"));                                           //                                  //
  } // of if-then-else no devices found                                       //                                  //
  Sd2Card card;                                                               // Instantiate Micro-SD Card class  //
  if (!card.init(SPI_HALF_SPEED,CS_PIN))                                      // Initialize the card              //
    Serial.print(F("- SD Initialization failed. Insert card, restart.\n"));   // Show error message               //
  SdVolume volume;                                                            // Instantiate Micro-SD Volume      //
  if (!volume.init(card))                                                     // Check to find partition          //
    Serial.print(F("Could not find FAT16/FAT32 partition.\n"));               // Show error message               //
  if (!SD.begin(CS_PIN)) Serial.println(F("SD initialization failed!"));      // Initialize SD card I/O           //
  cli();                                                                      // Disable interrupts               //
  TCCR1A  = 0;                                                                // Set timer1 A compare value to 0  //
  TCCR1B  = 0;                                                                // Set timer1 B compare value to 0  //
  TCNT1   = TIMER1_265DIV_10HZ;                                               // Load timer value                 //
  TCCR1B |= (1 << CS12);                                                      // Set the pre-scaler to 256        //
  TIMSK1 |= (1 << TOIE1);                                                     // Enable timer1 overflow interrupt //
  sei();                                                                      // Enable interrupts                //
} // of method setup()                                                        //                                  //
/*******************************************************************************************************************
** This is the main program for the Arduino IDE, it is called in an infinite loop. The INA219 measurements are    **
** automatically triggered within a timed interrupt routine and the statistics gathered. The 1-Wire microLAN      **
** DS18B20 thermometers values are read and then written to the SD card                                           **
*******************************************************************************************************************/
void loop() {                                                                 // Main program loop                //
  static bool FirstLoop = true;                                               // Initialized for 1st iteration    //
  sleep(MEASUREMENT_MILLIS);                                                  // Low-Energy until wake-up time    //
  openFile();                                                                 // Open output file, if necessary   //
  if (FirstLoop) {                                                            // If first time the display header //
    Serial.print(F("## BusVolts Shunt mV Current mA milliWatts\n"));          //                                  //
    Serial.print(F("== ======== ======== ========== ==========\n"));          //                                  //
    FirstLoop = false;                                                        // Reset flag after showing header  //
  } // of if-then we need to display header                                   //                                  //
  Serial.print(INA219_Readings);                                              //                                  //
  Serial.print(F("     "));                                                   //                                  //
  Serial.print(INA219_BusV/INA219_Readings,2);                                //                                  //
  Serial.print(F("     "));                                                   //                                  //
  Serial.print(INA219_ShuntV/INA219_Readings,2);                              //                                  //
  Serial.print(F("      "));                                                  //                                  //
  Serial.print(INA219_mA/INA219_Readings,2);                                  //                                  //
  Serial.print(F("     "));                                                   //                                  //
  Serial.println((INA219_BusV/INA219_Readings)*(INA219_mA/INA219_Readings),2);//                                  //
  now = rtc.now();                                                            // Get the Date and Time from DS1307//
  while (now.year()>(uint16_t)2100) now = rtc.now;                            // RTC returns bad value sometimes  //
  cli();                                                                      // disable interrupts               //
  sprintf(sprintfBuffer,"%04d-%02d-%02d,%02d:%02d:%02d,%d.%02d,%d.%02d",      // Build up the output line using   //
        now.year(),now.month(),now.day(),                                     // formatting and avoiding floating //
        now.hour(),now.minute(),now.second(),                                 // point output, as sprintf() on the//
        (int)(INA219_BusV/INA219_Readings),                                   // Arduino doesn't support it.      //
        (int)(INA219_BusV/INA219_Readings*100)%100,                           //                                  //
        (int)(INA219_mA/INA219_Readings),                                     //                                  //
        (int)(INA219_mA/INA219_Readings*100)%100 );                           //                                  //
  INA219_ShuntV   = 0;                                                        // Reset values after processing    //
  INA219_BusV     = 0;                                                        //                                  //
  INA219_mA       = 0;                                                        //                                  //
  INA219_Readings = 0;                                                        //                                  //
  sei();                                                                      // Re-enable interrupts             //
  dataFile.print(sprintfBuffer);                                              // Write to SD-Card                 //
  Serial.print(sprintfBuffer);                                                // Display to serial port           //
  for(uint8_t i=0;i<DSFamily.ThermometersFound;i++) {                         // Loop for each thermometer  found //
    Serial.print(F(","));                                                     // display to serial port           //
    dataFile.print(F(","));                                                   // format output line               //
    int16_t temperature = DSFamily.ReadDeviceTemp(i);                         // Read the raw temperature value   //
    Serial.print(temperature);                                                // write output                     //
    dataFile.print(temperature);                                              // write output                     //
  } // for-next every thermometer loop                                        //                                  //
  DSFamily.DeviceStartConvert();                                              // Start conversion on all devices  //
  dataFile.println();                                                         // Write to SD-Card                 //
  dataFile.flush();                                                           // force write everything in memory //
  Serial.println();                                                           // Display to serial port           //
} // of method loop                                                           //----------------------------------//
