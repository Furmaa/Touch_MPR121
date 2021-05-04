/*!
 *  @file Adafruit_MPR121_mod.h
 *
 *  This is a library for the MPR121 12-Channel Capacitive Sensor
 *
 *  Designed specifically to work with the MPR121 board.
 *
 *  Pick one up today in the adafruit shop!
 *  ------> https://www.adafruit.com/product/1982
 *
 *  These sensors use I2C to communicate, 2+ pins are required to interface
 *
 *  Adafruit invests time and resources providing this open source code,
 *  please support Adafruit andopen-source hardware by purchasing products
 *  from Adafruit!
 *
 *  Limor Fried/Ladyada (Adafruit Industries).
 *
 *  BSD license, all text above must be included in any redistribution
 * 
 *  Modified in 2020 by Aron Furmann/Furmaa for Fraunhofer IZM
 */

#ifndef ADAFRUIT_MPR121_H
#define ADAFRUIT_MPR121_H

#ifndef _BV
#define _BV(bit) (1 << (bit)) 
#endif

#include "Arduino.h"
#include <Wire.h>

///< board addressing
#define MPR121_I2CADDR_ADAFRUIT 0x5A        ///< adafruit board I2C address
#define MPR121_I2CADDR_ROBOTSHOP 0x5B        ///< robotshop board I2C address
#define MPR121_I2CADDR_DEFAULT MPR121_I2CADDR_ADAFRUIT ///< default I2C address

///< more default values
#define MPR121_TOUCH_THRESHOLD_DEFAULT 12  ///< default touch threshold value
#define MPR121_RELEASE_THRESHOLD_DEFAULT 6 ///< default relese threshold value
#define MPR121_ECR_SETTING_DEFAULT 0x8F ///< 5 bits for baseline tracking & proximity disabled & all electrodes running

///< Setting up electrode configuration: baseline tracking
///< - adjusts baseline value to change in filtered output: to acknowledge variations in the electrode surroundings.
#define MPR121_BL_TRACKING_OFF 0b01000000 ///< baseline tracking OFF: rigid system, does not adjust itself to change in environment
#define MPR121_BL_TRACKING_ALL 0b11000000 
#define MPR121_BL_TRACKING_5MSB 0b10000000 //initialize only 5 most siginificant bits
#define MPR121_BL_TRACKING_NOINIT 0b00000000 // baseline tracking ON w/o init! Needs more time after startup

///< Setting up electrode configuration: proximity detection 
///< - summes the signal of all electrodes to one value virtually creating a bigger, proximity electrode
#define MPR121_ELEPROX_EN_01 0b00010000 ///< for explanation see MPR121 Application Note AN3893
#define MPR121_ELEPROX_EN_03 0b00100000 
#define MPR121_ELEPROX_EN_011 0b00110000 
#define MPR121_ELEPROX_EN_OFF 0b00000000 

///< Setting up electrode configuration: example
///< uint8_t ecr = MPR121_BL_TRACKING_ALL + MPR121_ELEPROX_EN_OFF + elcount; 
///< baseline tracking ON with full initialization



/*!
 *  Device register map
 */
enum {
  MPR121_TOUCHSTATUS_L = 0x00,
  MPR121_TOUCHSTATUS_H = 0x01,
  MPR121_FILTDATA_0L = 0x04,
  MPR121_FILTDATA_0H = 0x05,
  MPR121_BASELINE_0 = 0x1E,
  MPR121_MHDR = 0x2B, 
  MPR121_NHDR = 0x2C, 
  MPR121_NCLR = 0x2D, 
  MPR121_FDLR = 0x2E, 
  MPR121_MHDF = 0x2F, 
  MPR121_NHDF = 0x30, 
  MPR121_NCLF = 0x31,
  MPR121_FDLF = 0x32,
  MPR121_NHDT = 0x33, 
  MPR121_NCLT = 0x34,
  MPR121_FDLT = 0x35,

  MPR121_TOUCHTH_0 = 0x41,
  MPR121_RELEASETH_0 = 0x42,
  MPR121_DEBOUNCE = 0x5B,
  MPR121_CONFIG1 = 0x5C,
  MPR121_CONFIG2 = 0x5D,
  MPR121_CHARGECURR_0 = 0x5F,
  MPR121_CHARGETIME_1 = 0x6C,
  MPR121_ECR = 0x5E,
  MPR121_AUTOCONFIG0 = 0x7B,
  MPR121_AUTOCONFIG1 = 0x7C,
  MPR121_UPLIMIT = 0x7D,
  MPR121_LOWLIMIT = 0x7E,
  MPR121_TARGETLIMIT = 0x7F,

  MPR121_GPIODIR = 0x76,
  MPR121_GPIOEN = 0x77,
  MPR121_GPIOSET = 0x78,
  MPR121_GPIOCLR = 0x79,
  MPR121_GPIOTOGGLE = 0x7A,

  MPR121_SOFTRESET = 0x80,
};

//.. thru to 0x1C/0x1D

/*!
 *  @brief  Class that stores state and functions for interacting with MPR121
 *  proximity capacitive touch sensor controller.
 */
class Adafruit_MPR121_mod 
{
public:
  ///< Hardware I2C
  Adafruit_MPR121_mod();

  void open(uint8_t i2caddr = MPR121_I2CADDR_DEFAULT, 
                TwoWire *theWire = &Wire);
  boolean init( uint8_t touchThreshold = MPR121_TOUCH_THRESHOLD_DEFAULT, 
                uint8_t releaseThreshold = MPR121_RELEASE_THRESHOLD_DEFAULT,
                bool autoconfigEnabled = true);
  void begin(uint8_t ecr = MPR121_ECR_SETTING_DEFAULT); 

  uint16_t filteredData(uint8_t t);
  uint16_t filteredDataAveraged(uint8_t t, uint8_t n = 0);
  uint16_t baselineData(uint8_t t);

  uint8_t readRegister8(uint8_t reg);
  uint16_t readRegister16(uint8_t reg);
  void writeRegister(uint8_t reg, uint8_t value);
  uint16_t touched(void);
  // Add deprecated attribute so that the compiler shows a warning
  void setThreshholds(uint8_t touch, uint8_t release)
      __attribute__((deprecated));
  void setThresholds(uint8_t touch, uint8_t release);

  /* added by Furmaa */
  uint8_t pauseTicks; /* number of timer overflows while LEDs OFF in turning ON routine */  
  uint8_t offTicks;   /* number of timer overflows while LEDs ON in turning OFF routine */
  uint8_t onBlinks;   /* number of LED ON-OFF changes in turning ON routine             */

  void printStatus(void);
  void calcStatistic (uint8_t eleccount, uint8_t factor);
  uint8_t manners (volatile bool turnedon, 
                   volatile bool * turn, 
                   volatile uint8_t * ticks, 
                   uint16_t * in_intensity, 
                   uint16_t * out_intensity, 
                   uint8_t * blinkcount);

protected:
  int8_t _i2caddr;
  TwoWire *_wire;
};

class TurnwheelDEMO : public Adafruit_MPR121_mod
{
  public:
    TurnwheelDEMO();
    uint16_t calcSliding(uint16_t cancelval, 
                       uint16_t * intstart, 
                       uint16_t * intend, 
                       uint16_t * lastintstart, 
                       uint16_t * lastintend, 
                       bool * rightslide, 
                       bool * leftslide);
};

class SinusDEMO : public Adafruit_MPR121_mod
{
  public:
    SinusDEMO();
    void blink (uint8_t * blinkcount, 
                volatile uint8_t * ticks,  
                uint16_t * in_intensity,  
                uint16_t * out_intensity, 
                uint8_t pausetime, 
                uint8_t blinktime, 
                uint8_t maxintensity); 
};

#endif
