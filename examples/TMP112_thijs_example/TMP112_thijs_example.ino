/*

this is a test of the temperature sensor: TMP112NAIDRLR
it supports I2C (/SMBus) interfacing from 1kHz to 2.83MHz (note: special instruction required for >400kHz operation)
note: I2C is not the most efficient protocol, so at 1kHz, retrieving 1 measurement may well take 25 milliseconds

i made my own library for interfacing with it: TMP112_thijs.h (temporary name?)

TMP112 I2C address table (connect the A0 pin to another):
pin to:   addr(bin):  addr(hex):  addr(dec):
GND       0bx1001000  0x48        72
V+        0bx1001001  0x49        73
SDA       0bx1001010  0x4A        74
SCL       0bx1001011  0x4B        75
*/
/*
This is an example sketch for the TMP112_thijs library
This sketch showcases most of the functions and the differences between platforms.
*/


// #define TMP112_useWireLib  // force the use of Wire.h (instead of platform-optimized code (if available))

#define TMP112debugPrint(x)  Serial.println(x)    //you can undefine these printing functions no problem, they are only for printing I2C errors
//#define TMP112debugPrint(x)  log_d(x)  // ESP32 style logging

#include <TMP112_thijs.h>

TMP112_thijs sensor(TMP112_ADDR_A0_to_VCC);


#ifdef ARDUINO_ARCH_ESP32  // on the ESP32, almost any pin can become an I2C pin
  const uint8_t TMP112_SDApin = 21; // 'defualt' is 21 (but this is just pin someone on the internet decided, i think the Wire library uses it)
  const uint8_t TMP112_SCLpin = 22; // 'defualt' is 22 (but this is just pin someone on the internet decided, i think the Wire library uses it)
#endif
#ifdef ARDUINO_ARCH_STM32   // on the STM32, each I2C peripheral has several pin options
  const uint8_t TMP112_SDApin = SDA; // default pin, on the STM32WB55 (nucleo_wb55rg_p) that's pin PB9
  const uint8_t TMP112_SCLpin = SCL; // default pin, on the STM32WB55 (nucleo_wb55rg_p) that's pin PB8
  /* Here is a handy little table of I2C pins on the STM32WB55 (nucleo_wb55rg_p):
      I2C1: SDA: PA10, PB7, PB9
            SCL: PA9, PB6, PB8
      I2C3: SDA: PB4, PB11, PB14, PC1
            SCL: PA7 PB10, PB13, PC0      */
#endif


void setup() 
{
  Serial.begin(115200);  delay(50); Serial.println();
  #ifdef TMP112_useWireLib // the slow (but pretty universally compatible) way
    sensor.init(100000); // NOTE: it's up to the user to find a frequency that works well.
  #elif defined(__AVR_ATmega328P__) || defined(__AVR_ATmega328__) // TODO: test 328p processor defines! (also, this code may be functional on other AVR hw as well?)
    pinMode(SDA, INPUT_PULLUP); //A4    NOT SURE IF THIS INITIALIZATION IS BEST (external pullups are strongly recommended anyways)
    pinMode(SCL, INPUT_PULLUP); //A5
    sensor.init(100000); // your average atmega328p should do 800kHz, but the TMP112 is 1kHz~400kHz limited (unless! you use some specific I2C commands to unlock the full 2.8Mhz range)
  #elif defined(ARDUINO_ARCH_ESP32)
//    pinMode(TMP112_SDApin, INPUT_PULLUP); //not needed, as twoWireSetup() does the pullup stuff for you
//    pinMode(TMP112_SCLpin, INPUT_PULLUP);
    esp_err_t initErr = sensor.init(100000, TMP112_SDApin, TMP112_SCLpin, 0); //on the ESP32 (almost) any pins can be I2C pins
    if(initErr != ESP_OK) { Serial.print("I2C init fail. error:"); Serial.println(esp_err_to_name(initErr)); Serial.println("while(1){}..."); while(1) {} }
    //note: on the ESP32 the actual I2C frequency is lower than the set frequency (by like 20~40% depending on pullup resistors, 1.5kOhm gets you about 800kHz)
  #elif defined(__MSP430FR2355__) //TBD: determine other MSP430 compatibility: || defined(ENERGIA_ARCH_MSP430) || defined(__MSP430__)
    // not sure if MSP430 needs pinMode setting for I2C, but it seems to work without just fine.
    sensor.init(100000); // TODO: test what the limit of this poor microcontroller are ;)
    delay(50);
  #elif defined(ARDUINO_ARCH_STM32)
    // not sure if STM32 needs pinMode setting for I2C
    sensor.init(100000, SDA, SCL, false); // TODO: test what the limits of this poor microcontroller are ;)
  #else
    #error("should never happen, TMP112_useWireLib should have automatically been selected if your platform isn't one of the optimized ones")
  #endif
  
  if(!sensor.connectionCheck()) { Serial.println("TMP112 connection check failed!"); while(1);}
  
  sensor.resetConfig(); //writes all 0s to the configuration registers

  sensor.printConfig(); //shows you the contents of the configuration registers
  Serial.println();

  // set some stuff
  sensor.setCR(3); // set to highest conversion rate (8/sec)
  sensor.setPOL(true); // invert the polarity of Alert (pin & CONF bit)
  sensor.printConfig(); Serial.println(); // print the config again just to review the changes
  Serial.print("temperature:"); Serial.println(sensor.getTemp());
  Serial.print("Thigh:"); Serial.println(sensor.getThigh());
  Serial.print("Tlow:"); Serial.println(sensor.getTlow());
  Serial.println("\n");

  // enable extended mode (extends the highest measurable temperature to 150 deg C)
  sensor.setEM(true); // enable Extended Mode (13bit)
  sensor.printConfig(); Serial.println(); // print the config again just to review the changes
  delay(500); // give the sensor time to get a new conversion done, now that Extended Mode is enabled
  Serial.print("temperature:"); Serial.println(sensor.getTemp(true)); // in Extended Mode (13bit), functions must be notified of the different math
  Serial.print("Thigh:"); Serial.println(sensor.getThigh(true)); // the Thigh and Tlow register do not automatically change when activating EM
  Serial.print("Tlow:"); Serial.println(sensor.getTlow(true)); // so these will be double the last values
  Serial.print("temperature:"); Serial.println(sensor.getTemp()); // the temperature measurement includes a bit to help identify 13bit data. If TMP112debugPrint() is defined, this will spit out a warning
  Serial.print("Thigh:"); Serial.println(sensor.getThigh()); // this however, does not have that handy check bit (for some reason), so it will just report a bad value without warning

  Serial.println('\n');
  Serial.println("retrieving value (without specifying register, so it's just reading the last-addressed register again):");
  Serial.print("value:"); Serial.println(sensor.onlyReadFloat(true));

  sensor.setEM(false); // disable Extended Mode (13bit)
  delay(500); // give the sensor time to get a new conversion done, now that Extended Mode is disabled
}

void loop()
{
  Serial.println(sensor.getTemp());
  delay(500);
}