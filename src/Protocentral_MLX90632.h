#pragma once

#if (ARDUINO >= 100)
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

#include <Wire.h>
#define MLX90632_ADDRESS 0x3A 

//Registers
#define EE_VERSION 0x240B

//32 bit constants
#define EE_P_R 0x240C
#define EE_P_G 0x240E
#define EE_P_T 0x2410
#define EE_P_O 0x2412
#define EE_Aa 0x2414
#define EE_Ab 0x2416
#define EE_Ba 0x2418
#define EE_Bb 0x241A
#define EE_Ca 0x241C
#define EE_Cb 0x241E
#define EE_Da 0x2420
#define EE_Db 0x2422
#define EE_Ea 0x2424
#define EE_Eb 0x2426
#define EE_Fa 0x2428
#define EE_Fb 0x242A
#define EE_Ga 0x242C

//16 bit constants
#define EE_Ha 0x2481
#define EE_Hb 0x2482
#define EE_Gb 0x242E
#define EE_Ka 0x242F
#define EE_Kb 0x2430

//Control registers
#define EE_CONTROL 0x24D4
#define EE_I2C_ADDRESS 0x24D5
#define REG_I2C_ADDRESS 0x3000
#define REG_CONTROL 0x3001
#define REG_STATUS 0x3FFF

//User RAM
#define RAM_1 0x4000
#define RAM_2 0x4001
#define RAM_3 0x4002
#define RAM_4 0x4003
#define RAM_5 0x4004
#define RAM_6 0x4005
#define RAM_7 0x4006
#define RAM_8 0x4007
#define RAM_9 0x4008

//Three measurement modes available
#define MODE_SLEEP 0b01
#define MODE_STEP 0b10
#define MODE_CONTINUOUS 0b11

//REG_STATUS bits
#define BIT_DEVICE_BUSY 10
#define BIT_EEPROM_BUSY 9
#define BIT_BROWN_OUT 8
#define BIT_CYCLE_POS 2 //6:2 = 5 bits
#define BIT_NEW_DATA 0

//REG_CONTROL bits
#define BIT_SOC 3
#define BIT_MODE 1 //2:1 = 2 bits

#define I2C_SPEED_STANDARD        100000
#define I2C_SPEED_FAST            400000

const int MAX_WAIT = 750; 

class Protocentral_MLX90632 {
  public:

    // Return values
    typedef enum
    {
      SENSOR_SUCCESS,
      SENSOR_ID_ERROR,
      SENSOR_I2C_ERROR,
      SENSOR_INTERNAL_ERROR,
      SENSOR_GENERIC_ERROR,
      SENSOR_TIMEOUT_ERROR
      //...
    } status;

    boolean begin(); 
    boolean begin(uint8_t deviceAddress, TwoWire &wirePort, status &returnError);

    status readRegister16(uint16_t addr, uint16_t &outputPointer);
    status writeRegister16(uint16_t addr, uint16_t val);

    status readRegister32(uint16_t addr, uint32_t &outputPointer);

    float getObjectTemp();
    float getObjectTemp(status &returnError);
    float getObjectTempF();
    float getSensorTemp();
    float getSensorTemp(status &returnError);

    bool dataAvailable();
    void clearNewData();
    uint16_t getStatus();
    uint16_t getStatus(status &returnError);
    void stepMode();
    void continuousMode();
    Protocentral_MLX90632::status setMode(uint8_t mode);
    uint8_t getMode();
    uint8_t getMode(status &returnError);
    Protocentral_MLX90632::status setSOC(); 

  private:

    float gatherSensorTemp(status &returnError);


    TwoWire *_i2cPort; 
    uint8_t _deviceAddress; 

    Stream *_debugPort; 
    boolean _printDebug = false; 

};
