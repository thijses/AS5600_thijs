/*
This is an example sketch for the AS5600 I2C (twi) library which I (Thijs van Liempd) wrote around 2021/2022.
I needed to read the AS5600 magnetic rotary encoder as fast as the bus would allow,
 so i made my own low-level code for the atmega328p (and later also for the ESP32, MSP430 and STM32)
In 2022 i added the loopReadWithCallback() function which slightly abuses the I2C bus by using a never-ending read packet.
The limitation of the sensor is the angle sampling rate, which this library seems to exceed (depending on filter config).
for insight into how/why i do things, please refer to the library code (only 1 header file becuase i'm too lazy to write a seperate .cpp file),
 and of course the datasheet of the sensor: https://ams.com/documents/20143/36005/AS5600_DS000365_5-00.pdf .

This sketch showcases most of the functions and how long they take to execute.
*/


//#define AS5600_useWireLib   // use the Wire.h library instead of the platform optimized code. Automatically defined if platform is not recognized

//#define AS5600_unlock_burning  //enable compilation of burn functions: burnAngle() and burnSetting()
//#define AS5600_return_esp_err  //makes certain functions return esp_err_t instead of bool, to allow for on-the-fly user debugging or whatever. AS5600debugPrint (if defined) should also print the errors

#define AS5600debugPrint(x)  Serial.println(x)    //you can undefine these printing functions no problem, they are only for printing I2C errors
//#define AS5600debugPrint(x)  log_d(x)

#include "AS5600_thijs.h"

AS5600_thijs sensor;

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
            SCL: PA7, PB10, PB13, PC0      */
#endif


const uint8_t exampleDataCount = 10;
uint32_t timers[exampleDataCount+1];
uint8_t timerItt = 0;

void printTimerValues(uint8_t spiltEvery = 0) {
  Serial.println("timers:");
  for(uint8_t i=1; i<timerItt; i++) {
    Serial.println(timers[i] - timers[i-1]);
    if(spiltEvery > 0) {
      if((i % spiltEvery) == 0) {
        Serial.println();
      }
    }
  }
  Serial.println(":timers\n");
}

uint16_t angles[exampleDataCount*4]; //record some sensor data just to prove it is indeed working as intended
uint16_t callbackItt = exampleDataCount*2; //in the callback function there is no implicit itterator

// only for .loopReadWithCallback():
void callback(uint16_t angle) { //with loopReadWithCallback you can call a function whenever new angle data is received (without ending the I2C packet)
  //Serial.println(angle);
  angles[callbackItt] = angle;    callbackItt++;
  timers[timerItt] = micros();  timerItt++;
  // to exit the loop early (or at all, if it's set to infinite), you can do:
  if(callbackItt >= (exampleDataCount*4)) {
    Serial.print("stopping loopReadWithCallback manually \t"); Serial.println(callbackItt);
    sensor.keepLooping = false;
  }
} //i haven't done extensive testing, but it seems like this function is allowed to take 100us without problems. At some point the sensor may complain and cut off the I2C packet.
// in general, i think this function should be as quick as possible (consider using 'inline'), for example, i put a motor_control_update function here.


void setup() {
  Serial.begin(115200);  Serial.println();
  #ifdef AS5600_useWireLib // the slow (but pretty universally compatible) way
    sensor.init(100000); // NOTE: it's up to the user to find a frequency that works well.
  #elif defined(__AVR_ATmega328P__) || defined(__AVR_ATmega328__) // TODO: test 328p processor defines! (also, this code may be functional on other AVR hw as well?)
    pinMode(SDA, INPUT_PULLUP); //A4    NOT SURE IF THIS INITIALIZATION IS BEST (external pullups are strongly recommended anyways)
    pinMode(SCL, INPUT_PULLUP); //A5
    sensor.init(800000); //anything beyond 800kHz doesnt seem to work properly
  #elif defined(ARDUINO_ARCH_ESP32)
//    pinMode(AS5600_SDApin, INPUT_PULLUP); //not needed, as twoWireSetup() does the pullup stuff for you
//    pinMode(AS5600_SCLpin, INPUT_PULLUP);
    esp_err_t initErr = sensor.init(1000000, AS5600_SDApin, AS5600_SCLpin, 0); //on the ESP32 (almost) any pins can be I2C pins
    if(initErr != ESP_OK) { Serial.print("I2C init fail. error:"); Serial.println(esp_err_to_name(initErr)); Serial.println("while(1){}..."); while(1) {} }
    //note: on the ESP32 the actual I2C frequency is lower than the set frequency (by like 20~40% depending on pullup resistors, 1.5kOhm gets you about 800kHz)
  #elif defined(__MSP430FR2355__) //TBD: determine other MSP430 compatibility: || defined(ENERGIA_ARCH_MSP430) || defined(__MSP430__)
    // not sure if MSP430 needs pinMode setting for I2C, but it seems to work without just fine.
    sensor.init(100000); // TODO: test what the limit is of this poor microcontroller ;)
  #elif defined(ARDUINO_ARCH_STM32)
    // not sure if STM32 needs pinMode setting for I2C
    sensor.init(100000, SDA, SCL, false); // TODO: test what the limit is of this poor microcontroller ;)
    /* NOTE: for initializing multiple devices, the code should look roughly like this:
      i2c_t* sharedBus = sensor.init(100000, SDA, SCL, false); // returns sensor._i2c (which is (currently) also just a public member, btw)
      secondSensor.init(sharedBus); // pass the i2c_t object (pointer) to the second device, to avoid re-initialization of the i2c peripheral
      //// repeated initialization of the same i2c peripheral will result in unexplained errors or silent crashes (during the first read/write action)!
    */
  #else
    #error("should never happen, AS5600_useWireLib should have automatically been selected if your platform isn't one of the optimized ones")
  #endif

  Serial.print("initial connectionCheck: "); Serial.println(sensor.connectionCheck());
  
  sensor.resetConfig(); //writes all 0s to the configuration registers

  //sensor.setSF(3); //set slow-filter to the mode with the least delay
  //sensor.setFTH(7); //set fast filter threshold to something... idk yet
  
  sensor.printConfig(); //shows you the contents of the configuration registers
  Serial.println();
  sensor.printStatus(); //shows you the status of the sensor (not whether it's connected, but whether the magnet is correctly positioned and stuff)
  Serial.println();
  
  //////////////traditional reading
  timers[timerItt] = micros();  timerItt++;
  for(uint8_t i=0; i<exampleDataCount; i++) {
    angles[i] = sensor.getAngle(); //retrieve angle the traditional way (by sending a full I2C packet including request for the angle register
    //note: getAngle() is really just a macro for requestReadInt(AS5600_RAW_ANGLE_H);
    //if you want the angle that is affected by the ZPOS/MPOS/MANG registers, you want getAltAngle() which is a macro for requestReadInt(AS5600_ANGLE_H)
    timers[timerItt] = micros();  timerItt++;
  }
  Serial.println("full request+reads times:"); printTimerValues();  timerItt = 0;

  timers[timerItt] = micros();  timerItt++;
  for(uint8_t i=exampleDataCount; i<(exampleDataCount*2); i++) {
    angles[i] = sensor.onlyReadInt(); //retrieve data without requesting a specific register to read from (works because of read cursor rollover explained on page 13)
    timers[timerItt] = micros();  timerItt++;
  }
  Serial.println("only reads times:"); printTimerValues();  timerItt = 0;

  #if (defined(__AVR_ATmega328P__) || defined(__AVR_ATmega328__)) && !defined(AS5600_useWireLib)  //i haven't made the loopReadWithCallback functions for other platforms (yet???). Most I2C peripherals don't seem to like the abuse
  
    timers[timerItt] = micros();  timerItt++;
    sensor.loopReadWithCallback(callback, exampleDataCount); //returned false when an error occurs during reading
  //  if(!sensor.loopReadWithCallback(callback, exampleDataCount)) { //check for errors
  //    Serial.println("loopReadWithCallback returned FALSE!");
  //  }
    Serial.println("loopReadWithCallback finite times:"); printTimerValues();  timerItt = 0;
  
    timers[timerItt] = micros();  timerItt++;
    sensor.loopReadWithCallback(callback, -1); //will run forever, unless a communication exception occurs or it is stopped using 'sensor.keepLooping = false'
    Serial.println("loopReadWithCallback infinite times:"); printTimerValues();  timerItt = 0;
    
  #endif
  
  Serial.println("\n angles:");
  for(uint16_t i=0; i<(exampleDataCount*4); i++) {
    Serial.println(angles[i]);
  }
}


void loop() {
}
