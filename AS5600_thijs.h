
//TBD: maybe i can go a little faster if i learn what endian-ness the sensor and microcontroller are...

/*
recent changelog:
fixed name in keywords.txt
change a few variable names (in writeBytes())
added some comments to the register defines
add connectionCheck bool function
add library.json (platformIO)


still TODO:
- remove .writeSameValue (i can live with it being slightly memory-inefficient, if it means fewer platform-specific functions)
- test Wire lib support
- test MSP430 support
- test STM32 support  (also, check writeBytes repeated start thingy (commented))
- add function descriptions
- check Wire.h function return values for I2C errors
- test if 'static' vars in the ESP32 functions actually are static (connect 2 sensors?)
- generalized memory map struct (also for other libraries). Could just be an enum, i just don't love #define

*/

//https://ams.com/documents/20143/36005/AS5600_DS000365_5-00.pdf

#ifndef AS5600_thijs_h
#define AS5600_thijs_h

#include "Arduino.h"


//#define AS5600_unlock_burning   // enable the chip-burning features of the AS5600

//#define AS5600debugPrint(x)  Serial.println(x)
//#define AS5600debugPrint(x)  log_d(x)   //using the ESP32 debug printing

#ifndef AS5600debugPrint
  #define AS5600debugPrint(x)  ;
#endif


//// AS5600 constants:
//configuration registers:
#define AS5600_ZMCO   0x00    // burn count reg   (R)
#define AS5600_ZPOS_H 0x01    // start pos Hreg   (R/W/P)
#define AS5600_ZPOS_L 0x02    // start pos Lreg   (R/W/P)
#define AS5600_MPOS_H 0x03    // stop pos Hreg    (R/W/P)
#define AS5600_MPOS_L 0x04    // stop pos Hreg    (R/W/P)
#define AS5600_MANG_H 0x05    // max angle Hreg   (R/W/P)
#define AS5600_MANG_L 0x06    // max angle Hreg   (R/W/P)
#define AS5600_CONF_A 0x07    // general configuration register A   (R/W/P)
#define AS5600_CONF_B 0x08    // general configuration register B   (R/W/P)
//output registers:
#define AS5600_RAW_ANGLE_H 0x0C   // raw measurement Hreg   (R)
#define AS5600_RAW_ANGLE_L 0x0D   // raw measurement Lreg   (R)
#define AS5600_ANGLE_H 0x0E   // measurement Hreg   (R)
#define AS5600_ANGLE_L 0x0F   // measurement Lreg   (R)
//status registers:
#define AS5600_STATUS 0x0B    // status reg   (R)
#define AS5600_AGC 0x1A       // Automatic Gain Control reg  (R)
#define AS5600_MAGNITUDE_H 0x1B   // magnitude Hreg   (R)
#define AS5600_MAGNITUDE_L 0x1C   // magnitude Hreg   (R)
//burn command:
#define AS5600_BURN 0xFF    // burn reg   (W)

#define LSB_NIBBLE 0x0F    //(nibble==4bits) the AS5600 never uses values larger than 12bits, but sometimes inserts bits in the MSB of other registers

#define AS5600_ZMCO_bits 0b00000011   // burn COunt
#define AS5600_WD_bits   0b00100000   // WatchDog
#define AS5600_FTH_bits  0b00011100   // Fast filter THreshold
#define AS5600_SF_bits   0b00000011   // Slow Filter
#define AS5600_PWMF_bits 0b11000000   // PWM Frequency
#define AS5600_OUTS_bits 0b00110000   // OUTput Stage
#define AS5600_HYST_bits 0b00001100   // HYSTeresis
#define AS5600_PM_bits   0b00000011   // Power Mode
#define AS5600_MD_bits   0b00100000   // Magnet Detected
#define AS5600_ML_bits   0b00010000   // Magnet too weak
#define AS5600_MH_bits   0b00001000   // Margnet too strong
#define AS5600_BURN_ANGLE_bits  0x80    // burn ZPOS and MPOS  (max 3 times, only if Magnet Detected, check ZMCO)
#define AS5600_BURN_SETTING_bits  0x40  // burn MANG and CONF  (only once, must be before BURN_ANGLE, check ZMCO)


#include "_AS5600_thijs_base.h" // this file holds all the nitty-gritty low-level stuff (I2C implementations (platform optimizations))
/**
 * An I2C interfacing library for the AS5600 magnetic encoder from AMS
 * 
 * features the option to use the Wire.h library, or optimized code for the following platforms:
 * atmega328p (direct register manipulation) (see loopReadWithCallback() function for absolute highest speeds)
 * ESP32 (below HAL layer, but not lowest level)
 * MSP430 (through Energia(?) middle layer)
 * STM32 (through twi->HAL layers)
 */
class AS5600_thijs : public _AS5600_thijs_base
{
  public:
  using _AS5600_thijs_base::_AS5600_thijs_base;
  /*
  This class only contains the higher level functions.
   for the base functions, please refer to _NT3H1x01_thijs_base.h
  here is a brief list of all the lower-level functions:
  - init()
  - requestReadByte()
  - requestReadBytes()
  - onlyReadBytes()
  - writeBytes()
  - writeSameValue()
  - loopReadWithCallback() ONLY for 328p (where you can (ab)use the I2C peripheral much more freely)
  */
  //// the following functions are abstract enough that they'll work for either architecture
  
  /**
   * (just a macro) check whether an AS5600_ERR_RETURN_TYPE (which may be one of several different types) is fine or not 
   * @param err (bool or esp_err_t or i2c_status_e, see on defines at top)
   * @return whether the error is fine
   */
  bool _errGood(AS5600_ERR_RETURN_TYPE err) {
    #if defined(AS5600_return_esp_err_t)
      return(err == ESP_OK);
    #elif defined(AS5600_return_i2c_status_e)
      return(err == I2C_OK);
    #else
      return(err);
    #endif
  }
  
  uint16_t requestReadInt(uint8_t registerToStartRead) { //note: by int i mean 12bit unsigned
    uint8_t readBuff[2]; //im secretly hoping the compiler will understand what i'm trying to do here and make it as efficient as possible, but probably...
    requestReadBytes(registerToStartRead, readBuff, 2);
    uint16_t returnData = (readBuff[0] & LSB_NIBBLE)  << 8;
    returnData |= readBuff[1];
    return(returnData); //(i know you could turn some of the lines of code into 1 big line, but that will not make the program faster
  }

  uint16_t onlyReadInt() { //note: by int i mean 12bit unsigned
    uint8_t readBuff[2];
    onlyReadBytes(readBuff, 2);
    uint16_t returnData = (readBuff[0] & LSB_NIBBLE) << 8;
    returnData |= readBuff[1];
    return(returnData); //(i know you could turn some of the lines of code into 1 big line, but that will not make the program faster
  }

  AS5600_ERR_RETURN_TYPE writeInt(uint8_t registerToStartWrite, uint16_t intToWrite) {
    uint8_t writeBuff[sizeof(intToWrite)];
    //writeBuff[0] = (intToWrite >> 8) & LSB_NIBBLE;
    writeBuff[0] = intToWrite >> 8;
    writeBuff[1] = intToWrite & 0xFF;
    return(writeBytes(registerToStartWrite, writeBuff, sizeof(intToWrite)));
  }
  
  uint16_t getAngle() { return(requestReadInt(AS5600_RAW_ANGLE_H)); } //basically macros
  uint16_t getAltAngle() { return(requestReadInt(AS5600_ANGLE_H)); } //this angle has hysteresis (i think?)

  AS5600_ERR_RETURN_TYPE setZPOS(uint16_t newValue) { return(writeInt(AS5600_ZPOS_H, newValue)); }
  AS5600_ERR_RETURN_TYPE setMPOS(uint16_t newValue) { return(writeInt(AS5600_MPOS_H, newValue)); }
  AS5600_ERR_RETURN_TYPE setMANG(uint16_t newValue) { return(writeInt(AS5600_MANG_H, newValue)); }

  AS5600_ERR_RETURN_TYPE setWD(bool newValue) {
    uint8_t currentReg = requestReadByte(AS5600_CONF_A);
    currentReg &= ~AS5600_WD_bits; //erase old value
    currentReg |= (newValue * AS5600_WD_bits); //insert new value
    return(writeBytes(AS5600_CONF_A, &currentReg, 1));
  }

  AS5600_ERR_RETURN_TYPE setFTH(uint8_t newValue) {
    uint8_t currentReg = requestReadByte(AS5600_CONF_A);
    currentReg &= ~AS5600_FTH_bits; //erase old value
    currentReg |= ((newValue<<2) & AS5600_FTH_bits); //insert new value
    return(writeBytes(AS5600_CONF_A, &currentReg, 1));
  }

  AS5600_ERR_RETURN_TYPE setSF(uint8_t newValue) {
    uint8_t currentReg = requestReadByte(AS5600_CONF_A);
    currentReg &= ~AS5600_SF_bits; //erase old value
    currentReg |= (newValue & AS5600_SF_bits); //insert new value
    return(writeBytes(AS5600_CONF_A, &currentReg, 1));
  }

  AS5600_ERR_RETURN_TYPE setPWMF(uint8_t newValue) {
    uint8_t currentReg = requestReadByte(AS5600_CONF_B);
    currentReg &= ~AS5600_PWMF_bits; //erase old value
    currentReg |= ((newValue<<6) & AS5600_PWMF_bits); //insert new value
    return(writeBytes(AS5600_CONF_B, &currentReg, 1));
  }

  AS5600_ERR_RETURN_TYPE setOUTS(uint8_t newValue) {
    uint8_t currentReg = requestReadByte(AS5600_CONF_B);
    currentReg &= ~AS5600_OUTS_bits; //erase old value
    currentReg |= ((newValue<<4) & AS5600_OUTS_bits); //insert new value
    return(writeBytes(AS5600_CONF_B, &currentReg, 1));
  }

  AS5600_ERR_RETURN_TYPE setHYST(uint8_t newValue) {
    uint8_t currentReg = requestReadByte(AS5600_CONF_B);
    currentReg &= ~AS5600_HYST_bits; //erase old value
    currentReg |= ((newValue<<2) & AS5600_HYST_bits); //insert new value
    return(writeBytes(AS5600_CONF_B, &currentReg, 1));
  }

  AS5600_ERR_RETURN_TYPE setPM(uint8_t newValue) {
    uint8_t currentReg = requestReadByte(AS5600_CONF_B);
    currentReg &= ~AS5600_PM_bits; //erase old value
    currentReg |= (newValue & AS5600_PM_bits); //insert new value
    return(writeBytes(AS5600_CONF_B, &currentReg, 1));
  }

  #ifdef AS5600_unlock_burning
    AS5600_ERR_RETURN_TYPE burnAngle() { return(writeSameValue(AS5600_BURN, AS5600_BURN_ANGLE_bits, 1)); }
    AS5600_ERR_RETURN_TYPE burnSetting() { return(writeSameValue(AS5600_BURN, AS5600_BURN_SETTING_bits, 1)); }
  #endif

  uint8_t getZMCO() { return(requestReadByte(AS5600_ZMCO) & AS5600_ZMCO_bits); } //burn count
  uint16_t getZPOS() { return(requestReadInt(AS5600_ZPOS_H)); }            //start pos
  uint16_t getMPOS() { return(requestReadInt(AS5600_MPOS_H)); }            //stop pos
  uint16_t getMANG() { return(requestReadInt(AS5600_MANG_H)); }            //max angle
  bool getWD() { return(requestReadByte(AS5600_CONF_A) & AS5600_WD_bits); }   //watchdog config
  uint8_t getFTH() { return((requestReadByte(AS5600_CONF_A) & AS5600_FTH_bits) >> 2); }  //fast filter threshold
  uint8_t getSF() { return(requestReadByte(AS5600_CONF_A) & AS5600_SF_bits); }          //slow filter
  uint8_t getPWMF() { return((requestReadByte(AS5600_CONF_B) & AS5600_PWMF_bits) >> 6); } //PWM frequency
  uint8_t getOUTS() { return((requestReadByte(AS5600_CONF_B) & AS5600_OUTS_bits) >> 4); } //output stage
  uint8_t getHYST() { return((requestReadByte(AS5600_CONF_B) & AS5600_HYST_bits) >> 2); } //hysteresis
  uint8_t getPM() { return(requestReadByte(AS5600_CONF_B) & AS5600_PM_bits); }          //power mode

  uint8_t getSTATUS() { return(requestReadByte(AS5600_STATUS) & 0b00111000); } //status: magnet detected, magnet too weak, magnet too strong
  uint8_t getAGC() { return(requestReadByte(AS5600_AGC)); } //should return 128 if magnet is positioned optimally (affected by temp, airgap and magnet degredation)
  uint16_t getMAGNITUDE() { return(requestReadInt(AS5600_MAGNITUDE_H)); } //magnitude of the 

  bool magnetDetected()  { return(getSTATUS() & AS5600_MD_bits); }
  bool magnetTooWeak()   { return(getSTATUS() & AS5600_ML_bits); }
  bool magnetTooStrong() { return(getSTATUS() & AS5600_MH_bits); }

  bool connectionCheck() { // there is no good value to test with the AS5600, but we can test if a read fails (by the ACKs and stuff)
    uint8_t readBuff;
    AS5600_ERR_RETURN_TYPE readSuccess = requestReadBytes(AS5600_STATUS, &readBuff, 1);
    return(_errGood(readSuccess));
  }

  void printConfig() {
    uint8_t readBuff[9];
    AS5600_ERR_RETURN_TYPE readSuccess = requestReadBytes(AS5600_ZMCO, readBuff, 9);
    if(_errGood(readSuccess))
    {
      //Serial.println("printing config:");
      Serial.print("burn count (ZMCO): "); Serial.println(readBuff[0] & AS5600_ZMCO_bits);
      uint16_t bigVal = ((readBuff[1] & LSB_NIBBLE) << 8) | readBuff[2];
      Serial.print("start pos (ZPOS): "); Serial.println(bigVal);
      bigVal = ((readBuff[3] & LSB_NIBBLE) << 8) | readBuff[4];
      Serial.print("stop pos (MPOS): "); Serial.println(bigVal);
      bigVal = ((readBuff[5] & LSB_NIBBLE) << 8) | readBuff[6];
      Serial.print("max angle (MANG): "); Serial.println(bigVal);
      Serial.print("watchdog (WD): "); Serial.println((readBuff[7] & AS5600_WD_bits)>>5);
      static const String FTHbitsToName[8] = {"slow only", "6LSBs", "7LSBs", "9LSBs", "18LSBs", "21LSBs", "24LSBs", "10LSBs"};
      uint8_t FTHbits = (readBuff[7] & AS5600_FTH_bits)>>2;
      Serial.print("fast filter threshold (FTH): "); Serial.print(FTHbits); Serial.print(" = "); Serial.println(FTHbitsToName[FTHbits]);
      uint8_t SFbits = readBuff[7] & AS5600_SF_bits;
      Serial.print("slow filter (SF): "); Serial.print(SFbits); Serial.print(" = "); Serial.print(16 >> SFbits); Serial.println('x'); // 16,8,4,2
      uint8_t PWMFbits = (readBuff[8] & AS5600_PWMF_bits)>>6;
      Serial.print("PWM frequency (PWMF): "); Serial.print(PWMFbits); Serial.print(" = "); Serial.print(115 << PWMFbits); Serial.println(" Hz"); // 115,230,460,920
      static const String OUTSbitsToName[4] = {"analog", "analog (10~90% range)", "PWM", "invalid"};
      uint8_t OUTSbits = (readBuff[8] & AS5600_OUTS_bits)>>4;
      Serial.print("output stage (OUTS): "); Serial.print(OUTSbits); Serial.print(" = "); Serial.println(OUTSbitsToName[OUTSbits]);
      Serial.print("hysteresis (HYST): "); Serial.print((readBuff[8] & AS5600_HYST_bits)>>2); Serial.println(" LSBits");
      static const String PMbitsToName[4] = {"NOM", "LPM1", "LPM2", "LPM3"};
      uint8_t PMbits = readBuff[8] & AS5600_PM_bits;
      Serial.print("power mode (PM): "); Serial.print(PMbits); Serial.print(" = "); Serial.println(PMbitsToName[PMbits]);
    } else {
      Serial.println("failed to read config!");
    }
  }

  void printStatus() {
    Serial.print("magnetDetected:"); Serial.println(magnetDetected());
    Serial.print("magnetTooWeak:"); Serial.println(magnetTooWeak());
    Serial.print("magnetTooStrong:"); Serial.println(magnetTooStrong());
    Serial.print("Automatic Gain Control (ideal: 128@5v, 64@3.3v):"); Serial.println(getAGC());
    Serial.print("magnitude (CORDIC):"); Serial.println(getMAGNITUDE());
  }

  AS5600_ERR_RETURN_TYPE resetConfig() { return(writeSameValue(AS5600_ZPOS_H, 0, 8)); } //write zeroes to all configuration registers (reset to default)
};

#endif  // AS5600_thijs_h

/*
AS5600 I2C magentic encoder

after some testing, i can confirm that, although the memory address pointer thingy advances every read cycle, 
 the RAW_ANGLE (and ANGLE?) registers make an exception to this, by jumping back from the Lower byte to the Higher byte (instead of moving to the next register)
 to be clear, the pointer will go RAW_ANGLE_H->RAW_ANGLE_L->RAW_ANGLE_H, repeat indefinetly
                       instead of RAW_ANGLE_H->RAW_ANGLE_L->ANGLE_H->ANGLE_L->invalid_sector
the pointer back-jump is done immedietly, even in the middle of a read cycle, 
 so if you read 4 bytes starting from the RAW_ANGLE_H register, you will receive RAW_ANGLE twice.
in theory, one could have an infinitely long read cycle, where you just keep reading more and more bytes.
the sensor value actualy does update in the middle of a read cycle (even at 800kHz), shown in this single 10 value (20 byte) read: 59 59 59 59 60 60 60 60 60 61
from some (very limited) testing, it seems to update the sensor value every 5 values at 800kHz I2C, which means ~65us between measurements
so i'm guessing a sampling rate of 16Khz  (at a slow-filter rate of 00==16x)

nov update: i took another look at it, and the datasheet claims a 150us sampling rate, which the data seems to reflect.
 I think i measured the sampling rate with my oscilloscope last time, not sure. All i know is, when i look at 15 measurements retrieved in ~440us,
 the same values are repeated 4~5 times, which makes ~5*30 = 150us.
 It sucks that the sampling rate is so limited, but this project was never meant to turn the motors that quickly (more about accurately).
 also, polling a single sensor value (at 800kHz I2C) takes like 72us, which leaves a lovely 70us to do some calulations in (or perhaps even leaves room for sensor interpolation???)
 i guess we'll see what the ESP32 makes of it (remember, that's what this shit will eventually need to run on)

*/


//note when writing lib: right now, i'm reading the angle as Hreg, Lreg, 
// but technically i could request the Lreg first, and the pointer return should make it so the value after that is the Hreg
// that my be able to save a single clock cylce when shifting the byte to make an int, maybe not.
