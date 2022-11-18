
#ifndef _AS5600_thijs_base_h
#define _AS5600_thijs_base_h

#include "Arduino.h" // always import Arduino.h

#include "AS5600_thijs.h" // (i feel like this constitutes a cicular dependency, but the compiler doesn't seem to mind)


#ifndef AS5600_useWireLib // (note: ifNdef!) if this has been defined by the user, then don't do all this manual stuff
  #if defined(__AVR_ATmega328P__) || defined(__AVR_ATmega328__) // TODO: test 328p processor defines! (also, this code may be functional on other AVR hw as well?)
    // nothing to import, all ATMega328P registers are imported by default
  #elif defined(ARDUINO_ARCH_ESP32)
    #include "driver/i2c.h"
  #elif defined(__MSP430FR2355__) //TBD: determine other MSP430 compatibility: || defined(ENERGIA_ARCH_MSP430) || defined(__MSP430__)
    #include <msp430.h>
    extern "C" {
      #include "twi.h"
    }
  #elif defined(ARDUINO_ARCH_STM32)
    extern "C" {
      #include "utility/twi.h"
    }
  #else // if the platform does not have optimized code
    #warning("using Wire library for AS5600 (no platform optimized code available)")
    #define AS5600_useWireLib  // use Wire as a backup
  #endif
#endif

#ifdef AS5600_useWireLib // note: this ifdef check is done after the ifndef, so the compiler gets a chance to define it anyway
  #include <Wire.h>
  // note: check the defined BUFFER_LENGTH in Wire.h for the max transmission length (on many platforms)
#endif


#ifndef AS5600_ERR_RETURN_TYPE  // unless the user already defined it manually
  #define AS5600_ERR_RETURN_TYPE_default  bool
  #ifdef AS5600_useWireLib
    #define AS5600_ERR_RETURN_TYPE  AS5600_ERR_RETURN_TYPE_default
  #elif defined(ARDUINO_ARCH_ESP32) // the ESP32 likes to spit out esp_err_t for most things
    #define AS5600_ERR_RETURN_TYPE  esp_err_t
    #define AS5600_return_esp_err_t   // to let the code below know that the return type is an esp_err_t
  #elif defined(ARDUINO_ARCH_STM32)
    #define AS5600_ERR_RETURN_TYPE  i2c_status_e
    #define AS5600_return_i2c_status_e // to let the code below know that the return type is an esp_err_t
  #else
    #define AS5600_ERR_RETURN_TYPE  AS5600_ERR_RETURN_TYPE_default
  #endif
#endif


//// some I2C constants
#define TW_WRITE 0 //https://en.wikipedia.org/wiki/I%C2%B2C  under "Addressing structure"
#define TW_READ  1

#define ACK_CHECK_EN 1  //only for ESP32
#define ACK_CHECK_DIS 0 //ack check disable does not seem to work??? (it always checks for an ack and spits out )

#define SIZEOF_I2C_CMD_DESC_T  20  //a terrible fix for a silly problem. The actual struct/typedef code is too far down the ESP32 dependancy rabbithole.
#define SIZEOF_I2C_CMD_LINK_T  20  //if the ESP32 didn't have such a fragmented filestructure this wouldn't be a problem, maybe
/* the reason why i need those 2 defines:   //this code was derived from https://github.com/espressif/esp-idf/blob/master/components/driver/i2c.c
uint8_t buffer[sizeof(i2c_cmd_desc_t) + sizeof(i2c_cmd_link_t) * numberOfOperations] = { 0 };
i2c_cmd_handle_t handle = i2c_cmd_link_create_static(buffer, sizeof(buffer));
*/



/**
 * (this is only the base class, users should use AS5600_thijs)
 * 
 */
class _AS5600_thijs_base
{
  public:
    volatile bool keepLooping = true; //used in loopReadWithCallback
  //private:
    //// I2C constants:
    static const uint8_t slaveAddress = 54; //7-bit address
    static const uint8_t SLA_W = ( slaveAddress <<1) | TW_WRITE;
    static const uint8_t SLA_R = ( slaveAddress <<1) | TW_READ;
  
  #ifdef AS5600_useWireLib // higher level generalized (arduino wire library):

    public:

    /**
     * initialize I2C peripheral through the Wire.h library
     * @param frequency SCL clock freq in Hz
     */
    void init(uint32_t frequency) {
      Wire.begin(); // init I2C as master
      Wire.setClock(frequency); // set the (approximate) desired clock frequency. Note, may be affected by pullup resistor strength (on some microcontrollers)
    }

    /**
     * request a specific register and read 1 byte
     * @param registerToRead register byte (see list of defines at top)
     * @return the byte it received (if successful)
     */
    uint8_t requestReadByte(uint8_t registerToRead) { // just abstracted with requestReadBytes()
      uint8_t readBuff;
      if(!requestReadBytes(registerToRead, &readBuff, 1)) { AS5600debugPrint("requestReadByte() requestReadBytes failed"); }
      return(readBuff);
    }
    
    /**
     * request a specific register and read bytes into a buffer
     * @param registerToStartRead register byte to start (will increment, see datasheet)
     * @param readBuff a buffer to store the read values in
     * @param bytesToRead how many bytes to read
     * @return whether it wrote/read successfully
     */
    bool requestReadBytes(uint8_t registerToStartRead, uint8_t readBuff[], uint8_t bytesToRead) {
      // ideally, i'd use the Wire function: requestFrom(address, quantity, iaddress, isize, sendStop), which lets you send the register through iaddress
      // HOWEVER, this function is not implemented on all platforms (looking at you, MSP430!), and it's not that hard to do manually anyway, so:
      Wire.beginTransmission(slaveAddress);
      Wire.write(registerToStartRead);
      Wire.endTransmission(); // the generalized Wire library is not always capable of repeated starts (on all platforms)
      return(onlyReadBytes(readBuff, bytesToRead));
    }
    
    /**
     * read bytes into a buffer (without first writing a register value!)
     * @param readBuff a buffer to store the read values in
     * @param bytesToRead how many bytes to read
     * @return whether it read successfully
     */
    bool onlyReadBytes(uint8_t readBuff[], uint8_t bytesToRead) {
      Wire.requestFrom(slaveAddress, bytesToRead);
      if(Wire.available() != bytesToRead) { AS5600debugPrint("onlyReadBytes() received insufficient data"); return(false); }
      for(uint8_t i=0; i<bytesToRead; i++) { readBuff[i] = Wire.read(); } // dumb byte-by-byte copy
      // unfortunately, TwoWire.rxBuffer is a private member, so we cant just memcpy. Then again, this implementation is not meant to be efficient
      return(true);
    }
    
    
    /**
     * request a specific register and write bytes from a buffer
     * @param registerToStartWrite register byte to start (will increment, see datasheet)
     * @param writeBuff a buffer of bytes to write to the device
     * @param bytesToWrite how many bytes to write
     * @return whether it wrote successfully
     */
    bool writeBytes(uint8_t registerToStartWrite, uint8_t writeBuff[], uint8_t bytesToWrite) {
      Wire.beginTransmission(slaveAddress);
      Wire.write(registerToStartWrite);
      Wire.write(writeBuff, bytesToWrite); // (usually) just calls a forloop that calls .write(byte) for every byte.
      Wire.endTransmission(); // this implementation does not really handle repeated starts (on all platforms)
      return(true);
    }

    /**
     * request a specific register and write the same byte several times
     * @param registerToStartWrite register byte to start (will increment, see datasheet)
     * @param valueToWrite the value to fill the registers with
     * @param bytesToWrite how many bytes to write
     * @return whether it wrote successfully
     */
    bool writeSameValue(uint8_t registerToStartWrite, uint8_t valueToWrite, uint8_t bytesToWrite) {
      Wire.beginTransmission(slaveAddress);
      Wire.write(registerToStartWrite);
      for(uint8_t i=0; i<bytesToWrite; i++) { // not very efficient, but then again, this implementation is not meant to be efficient
        Wire.write(valueToWrite); // note: in many platforms, Write.write() simply copies to a (private) buffer. Speed is clearly no concern for some people
      }
      Wire.endTransmission(); // this implementation does not really handle repeated starts (on all platforms)
      return(true);
    }

  #elif defined(__AVR_ATmega328P__) || defined(__AVR_ATmega328__) // TODO: test 328p processor defines! (also, this code may be functional on other AVR hw as well?)

    private:
    //// I2C constants:
    static const uint8_t twi_basic = (1<<TWINT) | (1<<TWEN); //any action will feature these 2 things (note TWEA is 0)
    static const uint8_t twi_START = twi_basic | (1<<TWSTA);
    static const uint8_t twi_STOP  = twi_basic | (1<<TWSTO);
    static const uint8_t twi_basic_ACK = twi_basic | (1<<TWEA); //(for master receiver mode) basic action, repond with ACK (if appropriate)
    
    static const uint8_t twi_SR_noPres = 0b11111000; //TWSR (stas register) without prescalebits
    // status register contents (master mode)
    static const uint8_t twi_SR_M_START = 0x08;      //start condition has been transmitted
    static const uint8_t twi_SR_M_RESTART = 0x10;    //repeated start condition has been transmitted
    static const uint8_t twi_SR_M_SLA_W_ACK = 0x18;  //SLA+W has been transmitted, ACK received
    static const uint8_t twi_SR_M_SLA_W_NACK = 0x20; //SLA+W has been transmitted, NOT ACK received
    static const uint8_t twi_SR_M_DAT_T_ACK = 0x28;  //data has been transmitted, ACK received
    static const uint8_t twi_SR_M_DAT_T_NACK = 0x30; //data has been transmitted, NOT ACK received
    static const uint8_t twi_SR_M_arbit = 0x38;      //arbitration
    static const uint8_t twi_SR_M_SLA_R_ACK = 0x40;  //SLA+R has been transmitted, ACK received
    static const uint8_t twi_SR_M_SLA_R_NACK = 0x48; //SLA+R has been transmitted, NOT ACK received
    static const uint8_t twi_SR_M_DAT_R_ACK = 0x50;  //data has been received, ACK returned
    static const uint8_t twi_SR_M_DAT_R_NACK = 0x58; //data has been received, NOT ACK returned
    // status register contents (slave mode)
    static const uint8_t twi_SR_S_SLA_W_ACK = 0x60;  //own address + W has been received, ACK returned
    static const uint8_t twi_SR_S_arbit_SLA_W = 0x68;//arbitration
    static const uint8_t twi_SR_S_GEN_ACK = 0x70;    //general call + W has been received, ACK returned
    static const uint8_t twi_SR_S_arbit_GEN = 0x78;  //arbitration
    static const uint8_t twi_SR_S_DAT_SR_ACK = 0x80; //data has been received after SLA+W, ACK returned
    static const uint8_t twi_SR_S_DAT_SR_NACK = 0x88;//data has been received after SLA+W, NOT ACK returned
    static const uint8_t twi_SR_S_DAT_GR_ACK = 0x90; //data has been received after GEN+W, ACK returned
    static const uint8_t twi_SR_S_DAT_GR_NACK = 0x98;//data has been received after GEN+W, NOT ACK returned
    static const uint8_t twi_SR_S_prem_STOP_RE =0xA0;//a STOP or repeated_START condition has been received prematurely (page 193)
    static const uint8_t twi_SR_S_SLA_R_ACK = 0xA8;  //own address + R has been received, ACK returned
    static const uint8_t twi_SR_S_arbit_SLA_R = 0xB0;//arbitration
    static const uint8_t twi_SR_S_DAT_ST_ACK = 0xB8; //data has been transmitted, ACK received     (master receiver wants more data)
    static const uint8_t twi_SR_S_DAT_ST_NACK = 0xC0;//data has been transmitted, NOT ACK received (master receiver doesnt want any more)
    static const uint8_t twi_SR_S_DAT_STL_ACK = 0xC8;//last (TWEA==0) data has been transmitted, ACK received (data length misconception)
    // status register contents (miscellaneous states)
    static const uint8_t twi_SR_nothing = twi_SR_noPres; //(0xF8) no relevant state info, TWINT=0
    static const uint8_t twi_SR_bus_err = 0; //bus error due to an illigal start/stop condition (if this happens, set TWCR to STOP condition)

    /*  what the ACK bit does (and what the status registers read if ACK is used wrong/unexpectedly):
    after a START, in response to an address byte, the slave uses ACK if (TWEA=1) it accepts the communication in general
    during data transferrence (either direction) the ACK/NOT-ACK is used to let the other side know whether or not they want more data
    if the recipient sends an ACK, it expects more data
    if the master transmitter ran out of data to send to the slave receiver, the slave status register will read 0xA0 (twi_SR_S_STOP_RESTART)
    if the slave transmitter ran out of data to send to the master receiver, the slave status register will read 0xC8 (twi_SR_S_DAT_STL_ACK)
        in that case, the slave transmitter will send all 1's untill STOP (or RESTART)
    in cases where there is too much data (from either side), the NOT-ACK will just be received earlier than expected
        if the slave sends NOT-ACK early, the master should STOP/RESTART the transmission (or the slave should ignore the overflowing data)
        if the master sends NOT-ACK early, the slave doesnt have to do anything (except maybe raise an error internally)
    
    in general, the TWEA (Enable Ack) bit should be synchronized in both devices (except for master transmitter, which doesnt use it).
    in master receiver, TWEA signals to the slave that the last byte is received, and the transmission will end
    in both slave modes, if TWEA==0, the slave expects for there to be a STOP/RESTART next 'tick', if not, the status register will read 0 (twi_SR_bus_err)
    */
    
    inline void twoWireTransferWait() { while(!(TWCR & (1<<TWINT))); }
    #define twoWireStatusReg      (TWSR & twi_SR_noPres)

    inline void twiWrite(uint8_t byteToWrite) {
      TWDR = byteToWrite;
      TWCR = twi_basic; //initiate transfer
      twoWireTransferWait();
    }

//    inline uint8_t twiReadAck(bool ack) {
//      TWCR = twi_basic_ACK; //request several bytes
//      twoWireTransferWait();
//      //if(twoWireStatusReg != twi_SR_M_DAT_R_ACK) { AS5600debugPrint("DAT_R Ack error"); return(0); }
//      return(TWDR);
//    }
//
//    inline uint8_t twiRead(bool ack) {
//      TWCR = twi_basic; //request several bytes
//      twoWireTransferWait();
//      //if(twoWireStatusReg != twi_SR_M_DAT_R_NACK) { AS5600debugPrint("DAT_R Nack error"); return(0); }
//      return(TWDR);
//    }
    
    inline bool startWrite() {
      TWCR = twi_START; //send start
      twoWireTransferWait();
      twiWrite(SLA_W);
      if(twoWireStatusReg != twi_SR_M_SLA_W_ACK) { AS5600debugPrint("SLA_W ack error"); TWCR = twi_STOP; return(false); }
      return(true);
    }

    inline bool startRead() {
      TWCR = twi_START; //repeated start
      twoWireTransferWait();
      twiWrite(SLA_R);
      if(twoWireStatusReg != twi_SR_M_SLA_R_ACK) { AS5600debugPrint("SLA_R ack error"); TWCR = twi_STOP; return(false); }
      return(true);
    }

    public:

    /**
     * initialize I2C peripheral
     * @param frequency SCL clock freq in Hz
     * @return frequency it was able to set
     */
    uint32_t init(uint32_t frequency) {
      // set frequency (SCL freq = F_CPU / (16 + 2*TWBR*prescaler) , where prescaler is 1,8,16 or 64x, see page 200)
      TWSR &= 0b11111000; //set prescaler to 1x
      //TWBR  = 12; //set clock reducer to 400kHz (i recommend external pullups at this point)
      #define prescaler 1
      TWBR = ((F_CPU / frequency) - 16) / (2*prescaler);
      uint32_t reconstFreq = F_CPU / (16 + (2*TWBR*prescaler));
      //Serial.print("freq: "); Serial.print(frequency); Serial.print(" TWBR:"); Serial.print(TWBR); Serial.print(" freq: "); Serial.println(reconstFreq);
      // the fastest i could get I2C to work is 800kHz (with another arduino as slave at least), which is TWBR=2 (with some 1K pullups)
      // any faster and i get SLA_ACK errors.
      return(reconstFreq);
    }

    
    /**
     * request a specific register and read 1 byte
     * @param registerToRead register byte (see list of defines at top)
     * @return the byte it received (if successful)
     */
    uint8_t requestReadByte(uint8_t registerToRead) { //can also be abstracted with requestReadBytes(), but whatever
      if(!startWrite()) { return(0); }
      twiWrite(registerToRead);  //if(twoWireStatusReg != twi_SR_M_DAT_T_ACK) { return(false); } //should be ACK(?)
      if(!startRead()) { return(0); }
      TWCR = twi_basic; //request 1 byte
      twoWireTransferWait();
      //if(twoWireStatusReg != twi_SR_M_DAT_R_NACK) { AS5600debugPrint("DAT_R Nack error"); return(0); }
      uint8_t returnData = TWDR;
      TWCR = twi_STOP;
      return(returnData);
    }
    
//    uint16_t requestReadInt(uint8_t registerToStartRead) { //note: by int i mean 12bit unsigned
//      if(!startWrite()) { return; }
//      twiWrite(registerToStartRead);  //if(twoWireStatusReg != twi_SR_M_DAT_T_ACK) { return(false); } //should be ACK(?)
//      if(!startRead()) { return; }
//      TWCR = twi_basic_ACK; //request several bytes
//      twoWireTransferWait();
//      //if(twoWireStatusReg != twi_SR_M_DAT_R_ACK) { AS5600debugPrint("DAT_R Ack error"); return(0); }
//      uint16_t returnData = (TWDR & LSB_NIBBLE) << 8;
//      TWCR = twi_basic; //request 1 more byte
//      twoWireTransferWait();
//      //if(twoWireStatusReg != twi_SR_M_DAT_R_NACK) { AS5600debugPrint("DAT_R Nack error"); return(0); }
//      returnData |= TWDR;
//      TWCR = twi_STOP;
//      return(returnData);
//    }
    
    /**
     * request a specific register and read bytes into a buffer
     * @param registerToStartRead register byte to start (will increment, see datasheet)
     * @param readBuff a buffer to store the read values in
     * @param bytesToRead how many bytes to read
     * @return whether it wrote/read successfully
     */
    bool requestReadBytes(uint8_t registerToStartRead, uint8_t readBuff[], uint8_t bytesToRead) {
      if(!startWrite()) { return(false); }
      twiWrite(registerToStartRead);  //if(twoWireStatusReg != twi_SR_M_DAT_T_ACK) { return(false); } //should be ACK(?)
      if(!startRead()) { return(false); }
      for(uint8_t i=0; i<(bytesToRead-1); i++) {
        TWCR = twi_basic_ACK; //request several bytes
        twoWireTransferWait();
        //if(twoWireStatusReg != twi_SR_M_DAT_R_ACK) { AS5600debugPrint("DAT_R Ack error"); return(false); }
        readBuff[i] = TWDR;
      }
      TWCR = twi_basic; //request 1 more byte
      twoWireTransferWait();
      //if(twoWireStatusReg != twi_SR_M_DAT_R_NACK) { AS5600debugPrint("DAT_R Nack error"); return(false); }
      readBuff[bytesToRead-1] = TWDR;
      TWCR = twi_STOP;
      return(true);
    }
    
    /**
     * read bytes into a buffer (without first writing a register value!)
     * @param readBuff a buffer to store the read values in
     * @param bytesToRead how many bytes to read
     * @return whether it read successfully
     */
    bool onlyReadBytes(uint8_t readBuff[], uint8_t bytesToRead) {
      if(!startRead()) { return(false); }
      for(uint8_t i=0; i<(bytesToRead-1); i++) {
        TWCR = twi_basic_ACK; //request several bytes
        twoWireTransferWait();
        //if(twoWireStatusReg != twi_SR_M_DAT_R_ACK) { AS5600debugPrint("DAT_R Ack error"); return(false); }
        readBuff[i] = TWDR;
      }
      TWCR = twi_basic; //request 1 more byte
      twoWireTransferWait();
      //if(twoWireStatusReg != twi_SR_M_DAT_R_NACK) { AS5600debugPrint("DAT_R Nack error"); return(false); }
      readBuff[bytesToRead-1] = TWDR;
      TWCR = twi_STOP;
      return(true);
    }
    
    
    /**
     * request a specific register and write bytes from a buffer
     * @param registerToStartWrite register byte to start (will increment, see datasheet)
     * @param writeBuff a buffer of bytes to write to the device
     * @param bytesToWrite how many bytes to write
     * @return whether it wrote successfully
     */
    bool writeBytes(uint8_t registerToStartWrite, uint8_t writeBuff[], uint8_t bytesToWrite) {
      if(!startWrite()) { return(false); }
      twiWrite(registerToStartWrite);  //if(twoWireStatusReg != twi_SR_M_DAT_T_ACK) { return(false); } //should be ACK(?)
      for(uint8_t i=0; i<bytesToWrite; i++) {
        twiWrite(writeBuff[i]);
        //if(twoWireStatusReg != twi_SR_M_DAT_T_ACK) { return(false); } //should be ACK(?)
      }
      TWCR = twi_STOP;
      return(true);
    }
    
    /**
     * request a specific register and write the same byte several times
     * @param registerToStartWrite register byte to start (will increment, see datasheet)
     * @param valueToWrite the value to fill the registers with
     * @param bytesToWrite how many bytes to write
     * @return whether it wrote successfully
     */
    bool writeSameValue(uint8_t registerToStartWrite, uint8_t valueToWrite, uint8_t bytesToWrite) {
      if(!startWrite()) { return(false); }
      twiWrite(registerToStartWrite);  //if(twoWireStatusReg != twi_SR_M_DAT_T_ACK) { return(false); } //should be ACK(?)
      for(uint8_t i=0; i<bytesToWrite; i++) {
        twiWrite(valueToWrite);
        //if(twoWireStatusReg != twi_SR_M_DAT_T_ACK) { return(false); } //should be ACK(?)
      }
      TWCR = twi_STOP;
      return(true);
    }

    /**
     * start a read, but don't finish it, allowing you to read (new) data without starting->addressing->reading->stopping every time
     * intended for absolute max-speed applications. Note: the AS5600 has an internal sampling rate that's slower than the max read-rate this allows,
     *  so you may see repeat data if your callback function is fast enough. Check the AS5600 datasheet (or some of my test-result-comments) for details
     * @param callbackFunc function to call once new angle data has been read
     * @param readsBeforeRestart how many times to read data before ending this function. Input -1 to loop indefinitely.
     * @return whether it ran successfully
     */
    bool loopReadWithCallback(/*uint8_t registerToStartRead,*/ void (*callbackFunc)(uint16_t angle), int16_t readsBeforeRestart = -1) {
      keepLooping = true;
      const uint8_t registerToStartRead = AS5600_RAW_ANGLE_H;
      uint8_t readBuff[2];
      uint16_t newAngle = 0;
      bool everyTwoBytes = true;
      
      if(!startWrite()) { return(false); }
      twiWrite(registerToStartRead);  //if(twoWireStatusReg != twi_SR_M_DAT_T_ACK) { return(false); } //should be ACK(?)
      if(!startRead()) { return(false); }
      //for(uint16_t readItt=0; ((readItt<((readsBeforeRestart*2)-1)) || (readsBeforeRestart<0)) && keepLooping; readItt++) { //this works as an infinite loop if readsBeforeRestart < 0
      for(uint16_t readItt=0; ((readsBeforeRestart>0) ? (readItt<((readsBeforeRestart*2)-1)) : true) && keepLooping; readItt++) { //this works as an infinite loop if readsBeforeRestart < 0
        everyTwoBytes = !everyTwoBytes; //flip-flop boolean to ensure 2 bytes are 
        TWCR = twi_basic_ACK; //request several bytes
        twoWireTransferWait();
        //if(twoWireStatusReg != twi_SR_M_DAT_R_ACK) { AS5600debugPrint("DAT_R Ack error"); return(false); }
        readBuff[everyTwoBytes] = TWDR;
        if(everyTwoBytes) {
          newAngle = (readBuff[0] & LSB_NIBBLE) << 8;
          newAngle |= readBuff[1];
          callbackFunc(newAngle);
        }
      }
      //according to the datasheet, any read interaction MUST end with a Nack read
      TWCR = twi_basic; //request 1 last byte
      twoWireTransferWait();
      //if(twoWireStatusReg != twi_SR_M_DAT_R_NACK) { AS5600debugPrint("DAT_R Nack error"); return(false); }
      readBuff[1] = TWDR;
      TWCR = twi_STOP;
      if(keepLooping) { //if the forloop was stopped manually, then you shouldn't run the callback another time
        newAngle = (readBuff[0] & LSB_NIBBLE) << 8;
        newAngle |= readBuff[1];
        callbackFunc(newAngle);
      }
      //keepLooping = false; //redundant?
      return(true);
    }

  #elif defined(ARDUINO_ARCH_ESP32)
    /*
    The ESP32 is both fantastically fast, and absolutely bogged down at the same time.
    The core itself runs wicked fast, so actual math is no problem.
    But interfacing with peripherals is often slow (because of the Real Time Operating System ?)
    Unfortunately, optimizing code by using low-level instructions is rarely easy, as there are always at least 3 'low' levels.
    In the case of I2C there is the arduino compatibility layer (top???): wire.begin(), etc.
    which is based on the HAL layer: i2cInit(), see esp32-hal-i2c.h in the core in Arduino15 in %appData%
    and below that there are some functions as well: i2c_driver_install() and i2c_master_cmd_begin(), see https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/peripherals/i2c.html
    there is a layer below that: https://github.com/espressif/esp-idf/blob/master/components/driver/i2c.c
    but it's badly documented, hard to read and very RTOS-optimized. I don't think i'm quite ready to unravel that thread (for a few microseconds of extra speed)
    
    The code does not appear to be very fast (an atmega328p can do twoWireReadTwoBytes() in ~80us and the ESP needs ~128us)
    using twoWireOnlyReadTwoBytes is ~34us faster, so if you're willing to risk (your code requiring some thought), you may get away with that
    The recommended method is to use a i2c_cmd_handle_t, but that slows things down SO DAMN MUCH.
    There are some options for timing customizations, and i spotted a ~10us solid LOW gap in the SCL signal, so there may be some time to be saved there.
    
    note: the frequency is significantly lower than you set it. Add stronger pullups to SDA and SCL to increase speed closer to desired (but never quite there).
            adding 4.7K pullups makes it so 1MHz requested == 750kHz real
            the signal curves looked like they could do with even lower resistance pullups, in the AS5600 datasheet they vaguely mention 20mA "Logic low output current"
    
    TBD:
        returning esp_err_t instead of boolean for most functions (while maintaining compatiblity with arduino.?.)
        const command links instead of making new ones?
        the ESP32 documentation states you "have to" delete them, but i tested it and you can reuse a command just fine...
          (note: after switching to static CMDs, it seems that constant/predefined commands would (at most!) save about 6us, while making the library a whole lot stupider)
    */
    
    public:
    //// I2C constants:
    i2c_port_t I2Cport = 0;
    uint32_t I2Ctimeout = 10; //in millis
    //const TickType_t I2CtimeoutTicks = 100 / portTICK_RATE_MS; //timeout (divide by portTICK_RATE_MS to convert millis to the right format)
    //uint8_t constWriteBuff[1]; //i2c_master_write_read_device() requires a const uint8_t* writeBuffer. You can make this array bigger if you want, shouldnt really matter
    
    /**
     * initialize I2C peripheral
     * @param frequency SCL clock freq in Hz
     * @param SDApin GPIO pin to use as SDA
     * @param SCLpin GPIO pin to use as SCL
     * @param I2CportToUse which of the ESP32's I2C peripherals to use
     * @return (esp_err_t) whether it was able to establish the peripheral
     */
    esp_err_t init(uint32_t frequency, int SDApin = 21, int SCLpin = 22, i2c_port_t I2CportToUse = 0) {
      if(I2CportToUse < I2C_NUM_MAX) { I2Cport = I2CportToUse; } else { AS5600debugPrint("can't init(), invalid I2Cport!"); return(ESP_ERR_INVALID_ARG); }
      i2c_config_t conf;
      conf.mode = I2C_MODE_MASTER;
      conf.sda_io_num = SDApin;
      conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
      conf.scl_io_num = SCLpin;
      conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
      conf.master.clk_speed = frequency;
      //conf.clk_flags = 0;          /*!< Optional, you can use I2C_SCLK_SRC_FLAG_* flags to choose i2c source clock here. */
      esp_err_t err = i2c_param_config(I2Cport, &conf);
      if (err != ESP_OK) { AS5600debugPrint("can't init(), i2c_param_config error!"); AS5600debugPrint(esp_err_to_name(err)); return(err); }
      return(i2c_driver_install(I2Cport, conf.mode, 0, 0, 0));
    }
    
    /**
     * request a specific register and read 1 byte
     * @param registerToRead register byte (see list of defines at top)
     * @return the byte it received (if successful)
     */
    uint8_t requestReadByte(uint8_t registerToRead) {
      uint8_t returnData;
//      const uint8_t numberOfCommands = 7; //start, write, write, start, write, read_NACK, stop
//      uint8_t CMDbuffer[SIZEOF_I2C_CMD_DESC_T + SIZEOF_I2C_CMD_LINK_T * numberOfCommands] = { 0 };
//      i2c_cmd_handle_t cmd = i2c_cmd_link_create_static(CMDbuffer, sizeof(CMDbuffer)); //create a CMD sequence
//      i2c_master_start(cmd);
//      i2c_master_write_byte(cmd, SLA_W, ACK_CHECK_EN);
//      i2c_master_write_byte(cmd, registerToRead, ACK_CHECK_DIS);
//      i2c_master_start(cmd);
//      i2c_master_write_byte(cmd, SLA_R, ACK_CHECK_EN);
//      i2c_master_read_byte(cmd, &returnData, I2C_MASTER_NACK); //I2C_MASTER_LAST_NACK
//      i2c_master_stop(cmd);
//      esp_err_t err = i2c_master_cmd_begin(I2Cport, cmd, I2Ctimeout / portTICK_RATE_MS);
//      i2c_cmd_link_delete_static(cmd);

      //constWriteBuff[0] = registerToRead;
      esp_err_t err = i2c_master_write_read_device(I2Cport, slaveAddress, &registerToRead, 1, &returnData, 1, I2Ctimeout / portTICK_RATE_MS); //faster (seems to work fine)
      if(err != ESP_OK) { AS5600debugPrint(esp_err_to_name(err)); }
      return(returnData);
    }
    
//    uint16_t requestReadInt(uint8_t registerToStartRead) {  //see alternate function based on twoWireReadBytes()
//      uint16_t returnData;
//      byte* returnDataPointer = (byte*) &returnData;
//      const uint8_t numberOfCommands = 8; //start, write, write, start, write, read_ACK, read_NACK, stop
//      uint8_t CMDbuffer[SIZEOF_I2C_CMD_DESC_T + SIZEOF_I2C_CMD_LINK_T * numberOfCommands] = { 0 };
//      i2c_cmd_handle_t cmd = i2c_cmd_link_create_static(CMDbuffer, sizeof(CMDbuffer)); //create a CMD sequence
//      i2c_master_start(cmd);
//      i2c_master_write_byte(cmd, SLA_W, ACK_CHECK_EN);
//      i2c_master_write_byte(cmd, registerToStartRead, ACK_CHECK_DIS);
//      i2c_master_start(cmd);
//      i2c_master_write_byte(cmd, SLA_R, ACK_CHECK_EN);
//      i2c_master_read_byte(cmd, returnDataPointer+1, I2C_MASTER_ACK); //HIGH side of 16bit int first
//      i2c_master_read_byte(cmd, returnDataPointer, I2C_MASTER_NACK);  //LOW side second
//      i2c_master_stop(cmd);
//      esp_err_t err = i2c_master_cmd_begin(I2Cport, cmd, I2Ctimeout / portTICK_RATE_MS);
//      i2c_cmd_link_delete_static(cmd);
//      if(err != ESP_OK) { AS5600debugPrint(esp_err_to_name(err)); }
//      return(returnData);
//    }
    
    /**
     * request a specific register and read bytes into a buffer
     * @param registerToStartRead register byte to start (will increment, see datasheet)
     * @param readBuff a buffer to store the read values in
     * @param bytesToRead how many bytes to read
     * @return (esp_err_t or bool) whether it wrote/read successfully
     */
    AS5600_ERR_RETURN_TYPE requestReadBytes(uint8_t registerToStartRead, uint8_t readBuff[], uint8_t bytesToRead) {
//      const uint8_t numberOfCommands = 8; //start, write, write, start, write, read_ACK, read_NACK, stop
//      uint8_t CMDbuffer[SIZEOF_I2C_CMD_DESC_T + SIZEOF_I2C_CMD_LINK_T * numberOfCommands] = { 0 };
//      i2c_cmd_handle_t cmd = i2c_cmd_link_create_static(CMDbuffer, sizeof(CMDbuffer)); //create a CMD sequence
//      i2c_master_start(cmd);
//      i2c_master_write_byte(cmd, SLA_W, ACK_CHECK_EN);
//      i2c_master_write_byte(cmd, registerToStartRead, ACK_CHECK_DIS);
//      i2c_master_start(cmd);
//      i2c_master_write_byte(cmd, SLA_R, ACK_CHECK_EN);
//      i2c_master_read(cmd, readBuff, bytesToRead, I2C_MASTER_LAST_NACK);
//      i2c_master_stop(cmd);
//      esp_err_t err = i2c_master_cmd_begin(I2Cport, cmd, I2Ctimeout / portTICK_RATE_MS);
//      i2c_cmd_link_delete_static(cmd);

      //constWriteBuff[0] = registerToStartRead;
      esp_err_t err = i2c_master_write_read_device(I2Cport, slaveAddress, &registerToStartRead, 1, readBuff, bytesToRead, I2Ctimeout / portTICK_RATE_MS); //faster (seems to work fine)
      if(err != ESP_OK) { AS5600debugPrint(esp_err_to_name(err)); }
      #ifdef AS5600_return_esp_err_t
        return(err);
      #else
        return(err == ESP_OK);
      #endif
    }
    
    /**
     * read bytes into a buffer (without first writing a register value!)
     * @param readBuff a buffer to store the read values in
     * @param bytesToRead how many bytes to read
     * @return (esp_err_t or bool) whether it read successfully
     */
    AS5600_ERR_RETURN_TYPE onlyReadBytes(uint8_t readBuff[], uint8_t bytesToRead) {
//      const uint8_t numberOfCommands = 5; //start, write, write, start, write, read_ACK, read_NACK, stop
//      uint8_t CMDbuffer[SIZEOF_I2C_CMD_DESC_T + SIZEOF_I2C_CMD_LINK_T * numberOfCommands] = { 0 };
//      i2c_cmd_handle_t cmd = i2c_cmd_link_create_static(CMDbuffer, sizeof(CMDbuffer)); //create a CMD sequence
//      i2c_master_start(cmd);
//      i2c_master_write_byte(cmd, SLA_R, ACK_CHECK_EN);
//      i2c_master_read(cmd, readBuff, bytesToRead, I2C_MASTER_LAST_NACK);
//      i2c_master_stop(cmd);
//      esp_err_t err = i2c_master_cmd_begin(I2Cport, cmd, I2Ctimeout / portTICK_RATE_MS);
//      i2c_cmd_link_delete_static(cmd);

      esp_err_t err = i2c_master_read_from_device(I2Cport, slaveAddress, readBuff, bytesToRead, I2Ctimeout / portTICK_RATE_MS);  //faster?
      if(err != ESP_OK) { AS5600debugPrint(esp_err_to_name(err)); }
      #ifdef AS5600_return_esp_err_t
        return(err);
      #else
        return(err == ESP_OK);
      #endif
    }
    
    
    /**
     * request a specific register and write bytes from a buffer
     * @param registerToStartWrite register byte to start (will increment, see datasheet)
     * @param writeBuff a buffer of bytes to write to the device
     * @param bytesToWrite how many bytes to write
     * @return (esp_err_t or bool) whether it wrote successfully
     */
    AS5600_ERR_RETURN_TYPE writeBytes(uint8_t registerToStartWrite, uint8_t writeBuff[], uint8_t bytesToWrite) {
      const uint8_t numberOfCommands = 5; //start, write, write, write, stop
      uint8_t CMDbuffer[SIZEOF_I2C_CMD_DESC_T + SIZEOF_I2C_CMD_LINK_T * numberOfCommands] = { 0 };
      i2c_cmd_handle_t cmd = i2c_cmd_link_create_static(CMDbuffer, sizeof(CMDbuffer)); //create a CMD sequence
      i2c_master_start(cmd);
      i2c_master_write_byte(cmd, SLA_W, ACK_CHECK_EN);
      i2c_master_write_byte(cmd, registerToStartWrite, ACK_CHECK_DIS);
      i2c_master_write(cmd, writeBuff, bytesToWrite, ACK_CHECK_DIS);
      i2c_master_stop(cmd);
      esp_err_t err = i2c_master_cmd_begin(I2Cport, cmd, I2Ctimeout / portTICK_RATE_MS);
      i2c_cmd_link_delete_static(cmd);
      // probably slightly slower:
//      uint8_t copiedArray[bytesToWrite+1]; copiedArray[0]=registerToStartWrite; for(uint8_t i=0;i<bytesToWrite;i++) { copiedArray[i+1]=writeBuff[i]; }
//      esp_err_t err = i2c_master_write_to_device(I2Cport, slaveAddress, copiedArray, bytesToWrite+1, I2Ctimeout / portTICK_RATE_MS);
      if(err != ESP_OK) { AS5600debugPrint(esp_err_to_name(err)); }
      #ifdef AS5600_return_esp_err_t
        return(err);
      #else
        return(err == ESP_OK);
      #endif
    }
    
    /**
     * request a specific register and write the same byte several times
     * @param registerToStartWrite register byte to start (will increment, see datasheet)
     * @param valueToWrite the value to fill the registers with
     * @param bytesToWrite how many bytes to write
     * @return (esp_err_t or bool) whether it wrote successfully
     */
    AS5600_ERR_RETURN_TYPE writeSameValue(uint8_t registerToStartWrite, uint8_t valueToWrite, uint8_t bytesToWrite) {
//      const uint8_t numberOfCommands = 5; //start, write, write, write, stop
//      uint8_t CMDbuffer[SIZEOF_I2C_CMD_DESC_T + SIZEOF_I2C_CMD_LINK_T * numberOfCommands] = { 0 };
//      i2c_cmd_handle_t cmd = i2c_cmd_link_create_static(CMDbuffer, sizeof(CMDbuffer)); //create a CMD sequence
//      i2c_master_start(cmd);
//      i2c_master_write_byte(cmd, SLA_W, ACK_CHECK_EN);
//      i2c_master_write_byte(cmd, registerToStartWrite, ACK_CHECK_DIS);
//      uint8_t singleValueArray[bytesToWrite];  for(uint8_t i=0;i<bytesToWrite;i++) { singleValueArray[i]=valueToWrite; }
//      i2c_master_write(cmd, singleValueArray, bytesToWrite, ACK_CHECK_DIS);
//      i2c_master_stop(cmd);
//      esp_err_t err = i2c_master_cmd_begin(I2Cport, cmd, I2Ctimeout / portTICK_RATE_MS);
//      i2c_cmd_link_delete_static(cmd);
      // another way:
//      uint8_t singleValueArray[bytesToWrite];  for(uint8_t i=0;i<bytesToWrite;i++) { singleValueArray[i]=valueToWrite; }
//      return(writeBytes(registerToStartWrite, singleValueArray, bytesToWrite));
      // yet another way:
      uint8_t singleValueArray[bytesToWrite+1]; singleValueArray[0]=registerToStartWrite; for(uint8_t i=0;i<bytesToWrite;i++) { singleValueArray[i+1]=valueToWrite; }
      esp_err_t err = i2c_master_write_to_device(I2Cport, slaveAddress, singleValueArray, bytesToWrite+1, I2Ctimeout / portTICK_RATE_MS);
      if(err != ESP_OK) { AS5600debugPrint(esp_err_to_name(err)); }
      #ifdef AS5600_return_esp_err_t
        return(err);
      #else
        return(err == ESP_OK);
      #endif
    }

    #elif defined(__MSP430FR2355__) //TBD: determine other MSP430 compatibility: || defined(ENERGIA_ARCH_MSP430) || defined(__MSP430__)

    public:

    /**
     * initialize I2C peripheral
     * @param frequency SCL clock freq in Hz
     */
    void init(uint32_t frequency) {
      //twi_setModule(module); // the MSP430 implementation of I2C is slightly janky. Instead of different classes for different I2C interfaces, they have a global variable indicating which module is targeted
      // the default module is all i'm going to need for my uses, but if you wanted to use multiple I2C peripherals, please uncomment all the twi_setModule() things and add a module constant to each AS5600_thijs obj
      twi_init();
      twi_setClock(frequency);
    }

    /**
     * request a specific register and read 1 byte
     * @param registerToRead register byte (see list of defines at top)
     * @return the byte it received (if successful)
     */
    uint8_t requestReadByte(uint8_t registerToRead) { // just abstracted with requestReadBytes()
      uint8_t readBuff;
      if(!requestReadBytes(registerToRead, &readBuff, 1)) { AS5600debugPrint("requestReadByte() requestReadBytes failed"); }
      return(readBuff);
    }
    
    /**
     * request a specific register and read bytes into a buffer
     * @param registerToStartRead register byte to start (will increment, see datasheet)
     * @param readBuff a buffer to store the read values in
     * @param bytesToRead how many bytes to read
     * @return whether it wrote/read successfully
     */
    bool requestReadBytes(uint8_t registerToRead, uint8_t readBuff[], uint8_t bytesToRead) {
      //twi_setModule(module);  // see init() for explenation
      int8_t ret = twi_writeTo(slaveAddress, &registerToRead, 1, 1, true); // transmit 1 byte, wait for the transmission to complete and send a STOP command
      if(ret != 0) { AS5600debugPrint("requestReadBytes() twi_writeTo error!"); return(false); }
      return(onlyReadBytes(readBuff, bytesToRead));
    }
    
    /**
     * read bytes into a buffer (without first writing a register value!)
     * @param readBuff a buffer to store the read values in
     * @param bytesToRead how many bytes to read
     * @return whether it read successfully
     */
    bool onlyReadBytes(uint8_t readBuff[], uint8_t bytesToRead) {
      uint8_t readQuantity = twi_readFrom(slaveAddress, readBuff, bytesToRead, true); // note: sendstop=true
      if(readQuantity != bytesToRead) { AS5600debugPrint("onlyReadBytes() received insufficient data"); return(false); }
      return(true);
    }
    
    
    /**
     * request a specific register and write bytes from a buffer
     * @param registerToStartWrite register byte to start (will increment, see datasheet)
     * @param writeBuff a buffer of bytes to write to the device
     * @param bytesToWrite how many bytes to write
     * @return whether it wrote successfully
     */
    bool writeBytes(uint8_t registerToStartWrite, uint8_t writeBuff[], uint8_t bytesToWrite) {
      uint8_t bufferCopyWithReg[bytesToWrite+1];   bufferCopyWithReg[0] = registerToStartWrite;
      for(uint8_t i=0;i<bytesToWrite; i++) { bufferCopyWithReg[i+1] = writeBuff[i]; } // manually copy all bytes
      int8_t ret = twi_writeTo(slaveAddress, bufferCopyWithReg, bytesToWrite+1, 1, true); // transmit some bytes, wait for the transmission to complete and send a STOP command
      if(ret != 0) { AS5600debugPrint("writeBytes() twi_writeTo error!"); return(false); }
      return(true);
      // NOTE; i'd love to just send one byte, then send the writeBuff, but the MSP430 twi library is not made for that.
      // calling twi_writeTo always calls a start condition, and a repeated start causes the AS5600 to look for a register again i think.
      // underwater, all the MSP430 twi library does is fill a buffer and start an operation which calls an ISR,
      //  but i can't even insert one byte in the buffer before the rest, because there is no function for that (also, twi_writeTo clears the buffer before adding to it).
    }

    /**
     * request a specific register and write the same byte several times
     * @param registerToStartWrite register byte to start (will increment, see datasheet)
     * @param valueToWrite the value to fill the registers with
     * @param bytesToWrite how many bytes to write
     * @return whether it wrote successfully
     */
    bool writeSameValue(uint8_t registerToStartWrite, uint8_t valueToWrite, uint8_t bytesToWrite) {
      // there is no great/special way to do this on the MSP430, so i'll just abstact it:
      uint8_t singleValueArray[bytesToWrite]; for(uint8_t i=0;i<bytesToWrite;i++) { singleValueArray[i]=valueToWrite; }
      return(writeBytes(registerToStartWrite, singleValueArray, bytesToWrite));
      //// alt:
      // uint8_t singleValueArray[bytesToWrite+1];   singleValueArray[0] = registerToStartWrite;
      // for(uint8_t i=0;i<bytesToWrite; i++) { singleValueArray[i+1] = valueToWrite; } // manually copy all bytes
      // int8_t ret = twi_writeTo(slaveAddress, singleValueArray, bytesToWrite+1, 1, true); // transmit some bytes, wait for the transmission to complete and send a STOP command
      // // see notes in writeBytes about why i made it so bad/inefficient
      // if(ret != 0) { AS5600debugPrint("writeBytes() twi_writeTo error!"); return(false); }
      // return(true);
    }

  #elif defined(ARDUINO_ARCH_STM32)
  
    /* Notes on the STM32 I2C perihperal (specifically that of the STM32WB55):
    Much like the ESP32, the STM32 libraries are built on several layers of abstraction.
    The twi.h library goes to some HAL library, and i have no intention of finding out where it goes from there.
    Since this particular implementation does not need to be terribly fast, i'll just stick with twi.h,
     which does have the advantage of working with the whole STM32 family (whereas a lower implementation would target specific subfamilies)
    The STM32 can map the I2C pins to a limited (but at least more than one) selection of pins,
     see PeripheralPins.c for the PinMap_I2C_SDA and PinMap_I2C_SCL (or just look at the datasheet)
     (for my purposes, that's: .platformio\packages\framework-arduinoststm32\variants\STM32WBxx\WB55R(C-E-G)V\PeripheralPins.c )
     Here is a handy little table:
      I2C1: SDA: PA10, PB7, PB9
            SCL: PA9, PB6, PB8
      I2C3: SDA: PB4, PB11, PB14, PC1
            SCL: PA7, PB10, PB13, PC0
    
    */

    public:

    i2c_t _i2c; // handler thingy (presumably)
    static const uint8_t STM32_MASTER_ADDRESS = 0x01; // a reserved address which tells the peripheral it's a master, not a slave
    
    /**
     * initialize I2C peripheral on STM32
     * @param frequency SCL clock freq in Hz
     * @param SDApin pin (arduino naming) to use as SDA (select few possible)
     * @param SCLpin pin (arduino naming) to use as SCL (select few possible)
     * @param generalCall i'm honestly not sure, the STM32 twi library is not documented very well...
     */
    void init(uint32_t frequency, uint32_t SDApin=PIN_WIRE_SDA, uint32_t SCLpin=PIN_WIRE_SCL, bool generalCall = false) {
      _i2c.sda = digitalPinToPinName(SDApin);
      _i2c.scl = digitalPinToPinName(SCLpin);
      _i2c.__this = (void *)this; // i truly do not understand the stucture of the STM32 i2c_t, but whatever, i guess the i2c_t class needs to know where this higher level class is or something
      _i2c.isMaster = true;
      _i2c.generalCall = (generalCall == true) ? 1 : 0; // 'generalCall' is just a uint8_t instead of a bool
      i2c_custom_init(&_i2c, frequency, I2C_ADDRESSINGMODE_7BIT, (STM32_MASTER_ADDRESS << 1));
      // note: use i2c_setTiming(&_i2c, frequency) if you want to change the frequency later
    }

    /**
     * request a specific register and read 1 byte
     * @param registerToRead register byte (see list of defines at top)
     * @return the byte it received (if successful)
     */
    uint8_t requestReadByte(uint8_t registerToRead) { // just uses requestReadBytes
      uint8_t returnData;
      AS5600_ERR_RETURN_TYPE readSuccess = requestReadBytes(registerToRead, &returnData, 1);
      #ifdef AS5600_return_i2c_status_e
        if(readSuccess != I2C_OK)
      #else
        if(!readSuccess)
      #endif
       { AS5600debugPrint("requestReadByte() requestReadBytes failed"); }
      return(returnData);
    }
    
    /**
     * request a specific register and read bytes into a buffer
     * @param registerToStartRead register byte to start (will increment, see datasheet)
     * @param readBuff a buffer to store the read values in
     * @param bytesToRead how many bytes to read
     * @return (i2c_status_e or bool) whether it wrote/read successfully
     */
    AS5600_ERR_RETURN_TYPE requestReadBytes(uint8_t registerToRead, uint8_t readBuff[], uint8_t bytesToRead) {
      #if defined(I2C_OTHER_FRAME) // if the STM32 subfamily is capable of writing without sending a stop
        _i2c.handle.XferOptions = I2C_OTHER_AND_LAST_FRAME; // tell the peripheral it should send a STOP at the end
      #endif
      i2c_status_e err = i2c_master_write(&_i2c, (slaveAddress << 1), &registerToRead, 1);
      if(err != I2C_OK) {
        AS5600debugPrint("requestReadBytes() i2c_master_write error!");
        #ifdef AS5600_return_i2c_status_e
          return(err);
        #else
          return(false);
        #endif
      }
      return(onlyReadBytes(readBuff, bytesToRead));
    }
    
    /**
     * read bytes into a buffer (without first writing a register value!)
     * @param readBuff a buffer to store the read values in
     * @param bytesToRead how many bytes to read
     * @return (i2c_status_e or bool) whether it read successfully
     */
    AS5600_ERR_RETURN_TYPE onlyReadBytes(uint8_t readBuff[], uint8_t bytesToRead) {
      #if defined(I2C_OTHER_FRAME) // not on all STM32 variants
        _i2c.handle.XferOptions = I2C_OTHER_AND_LAST_FRAME; // (this one i don't understand, but the Wire.h library does it, and without it i get HAL_I2C_ERROR_SIZE~~64 (-> I2C_ERROR~~4))
      #endif
      i2c_status_e err = i2c_master_read(&_i2c, (slaveAddress << 1), readBuff, bytesToRead);
      if(err != I2C_OK) { AS5600debugPrint("onlyReadBytes() i2c_master_read error!"); }
      #ifdef AS5600_return_i2c_status_e
        return(err);
      #else
        return(err == I2C_OK);
      #endif
    }
    
    /**
     * request a specific register and write bytes from a buffer
     * @param registerToStartWrite register byte to start (will increment, see datasheet)
     * @param writeBuff a buffer of bytes to write to the device
     * @param bytesToWrite how many bytes to write
     * @return (i2c_status_e or bool) whether it wrote successfully
     */
    AS5600_ERR_RETURN_TYPE writeBytes(uint8_t registerToStartWrite, uint8_t writeBuff[], uint8_t bytesToWrite) {
      //// NOTE: the code below is commented out because it concerns a potential optimization (repeated start instead of buffer copying), but it's untested! (and not really needed anyway)
      // #if defined(I2C_OTHER_FRAME) // if the STM32 subfamily is capable of writing without sending a stop
      //   _i2c.handle.XferOptions = I2C_OTHER_FRAME; // tell the peripheral it should NOT send a STOP at the end
      //   i2c_status_e err = i2c_master_write(&_i2c, (slaveAddress << 1), &registerToStartWrite, 1);
      //   if(err != I2C_OK) {
      //     AS5600debugPrint("requestReadBytes() first i2c_master_write error!");
      //     #ifdef AS5600_return_i2c_status_e
      //       return(err);
      //     #else
      //       return(false);
      //     #endif
      //   }
      //   _i2c.handle.XferOptions = I2C_OTHER_AND_LAST_FRAME; // tell the peripheral it should send a STOP at the end
      //   err = i2c_master_write(&_i2c, (slaveAddress << 1), writeBuff, bytesToWrite);
      // #else // if the STM32 subfamily can't handle repeated starts anyways, just do it the hard way:
      //// if the code above is uncommented, be sure to remove this next part:

      #if defined(I2C_OTHER_FRAME) // if the STM32 subfamily is capable of writing without sending a stop
        _i2c.handle.XferOptions = I2C_OTHER_AND_LAST_FRAME; // tell the peripheral it should send a STOP at the end
      #endif

        uint8_t bufferCopyWithReg[bytesToWrite+1];   bufferCopyWithReg[0] = registerToStartWrite;
        for(uint8_t i=0;i<bytesToWrite; i++) { bufferCopyWithReg[i+1] = writeBuff[i]; } // manually copy all bytes
        i2c_status_e err = i2c_master_write(&_i2c, (slaveAddress << 1), bufferCopyWithReg, bytesToWrite+1);

      // #endif // related to the commented optimization code above

      if(err != I2C_OK) { AS5600debugPrint("writeBytes() i2c_master_write error!"); }
      #ifdef AS5600_return_i2c_status_e
        return(err);
      #else
        return(err == I2C_OK);
      #endif
    }

    /**
     * request a specific register and write the same byte several times
     * @param registerToStartWrite register byte to start (will increment, see datasheet)
     * @param valueToWrite the value to fill the registers with
     * @param bytesToWrite how many bytes to write
     * @return (i2c_status_e or bool) whether it wrote successfully
     */
    AS5600_ERR_RETURN_TYPE writeSameValue(uint8_t registerToStartWrite, uint8_t valueToWrite, uint8_t bytesToWrite) {
      // there is no great/special way to do this on the STM32, so i'll just abstact it:
      uint8_t singleValueArray[bytesToWrite]; for(uint8_t i=0;i<bytesToWrite;i++) { singleValueArray[i]=valueToWrite; }
      return(writeBytes(registerToStartWrite, singleValueArray, bytesToWrite));
      //// alt:
      // uint8_t singleValueArray[bytesToWrite+1]; singleValueArray[0]=registerToStartWrite; for(uint8_t i=0;i<bytesToWrite;i++) { singleValueArray[i+1]=valueToWrite; }
      // #if defined(I2C_OTHER_FRAME) // if the STM32 subfamily is capable of writing without sending a stop
      //   _i2c.handle.XferOptions = I2C_OTHER_AND_LAST_FRAME; // tell the peripheral it should send a STOP at the end
      // #endif
      // i2c_status_e err = i2c_master_write(&_i2c, (slaveAddress << 1), singleValueArray, bytesToWrite+1);
      // if(err != I2C_OK) { AS5600debugPrint("writeBytes() i2c_master_write error!"); }
      // #ifdef AS5600_return_i2c_status_e
      //   return(err);
      // #else
      //   return(err == I2C_OK);
      // #endif
    }
    
  #else
    #error("should never happen, platform optimization code has issue (probably at the top there)")
  #endif // platform-optimized code end
  
  /*
  the remainder of the code can be found in the main header file: AS5600_thijs.h
  This is just a parent class, meant to hold all the low-level I2C implementations
  */
};

#endif // _AS5600_thijs_base_h