/*!
 * @file Adafruit_MPR121_mod.cpp
 *
 *  @mainpage modified Adafruit MPR121 arduino driver
 *
 *  @section intro_sec Introduction
 *
 *  This is a library for the MPR121 I2C 12-chan Capacitive Sensor
 *
 *  Designed specifically to work with the MPR121 sensor from Adafruit
 *  ----> https://www.adafruit.com/products/1982
 *
 *  These sensors use I2C to communicate, 2+ pins are required to
 *  interface
 *
 *  Adafruit invests time and resources providing this open source code,
 *  please support Adafruit and open-source hardware by purchasing
 *  products from Adafruit!
 *
 *  @section author Author
 *
 *  Written by Limor Fried/Ladyada for Adafruit Industries.
 *
 *  @section license License
 *
 *  BSD license, all text here must be included in any redistribution.
 * 
 *  @section modification Modification
 * 
 *  Modified in 2020 by Aron Furmann/Furmaa for Fraunhofer IZM
 */

#include "Adafruit_MPR121_mod.h"

/*!
 *  @brief      Default constructor
 */
Adafruit_MPR121_mod::Adafruit_MPR121_mod() {}

/*!
 *  @brief    Begin an MPR121 object on a given I2C bus.
 *  @param    i2caddr
 *            the i2c address the device can be found on. Defaults to 0x5A.
 *  @param    theWire
 *            Wire object
 */

void Adafruit_MPR121_mod::open(uint8_t i2caddr, TwoWire *theWire) 
{
  _i2caddr = i2caddr;
  _wire = theWire;

  _wire->begin();  
}

/*!
 *  @brief    This function resets the device and writes the default settings.
 *  @param    touchThreshold
 *            touch detection threshold value
 *  @param    releaseThreshold
 *            release detection threshold value
 *  @param    autoconfigEnabled
 *            yes or no to automatic configuration of electrodes
 *  @returns  true on success, false otherwise
 */

boolean Adafruit_MPR121_mod::init( uint8_t touchThreshold,
                               uint8_t releaseThreshold,
                               bool autoconfigEnabled) 
{
  // soft reset
  writeRegister(MPR121_SOFTRESET, 0x63);
  delay(10);
  for (uint8_t i = 0; i < 0x7F; i++) {
    //  Serial.print("$"); Serial.print(i, HEX);
    //  Serial.print(": 0x"); Serial.println(readRegister8(i));
  }

  uint8_t c = readRegister8(MPR121_CONFIG2);

  if (c != 0x24)
    return false;

  // writeRegister(MPR121_ECR, 0x0); unneeded! 

  setThresholds(touchThreshold, releaseThreshold);
  writeRegister(MPR121_MHDR, 0x01); ///< Maximum Half Delta Rising, value 1-63: largest magnitude passing through
  writeRegister(MPR121_NHDR, 0x05); ///< Noise Half Delta Rising, value 1-63: incremental for non-noise drift
  writeRegister(MPR121_NCLR, 0x01); ///< Noise Count Limit Rising, value 0-255: number of samples consecutively greater than MHDR
  writeRegister(MPR121_FDLR, 0x00); ///< Filter Delay Count Limit Rising, value 0-255: operation rate of filter, 0: fastest, 255: slowest

  writeRegister(MPR121_MHDF, 0x06); ///< Maximum Half Delta Falling, value 1-63: largest magnitude passing through
  writeRegister(MPR121_NHDF, 0x05); ///< Noise Half Delta Falling, value 1-63: .... so on
  writeRegister(MPR121_NCLF, 0x01);
  writeRegister(MPR121_FDLF, 0x00);

  writeRegister(MPR121_NHDT, 0x00); ///< Noise Half Delta Touch ... and so on
  writeRegister(MPR121_NCLT, 0x00);
  writeRegister(MPR121_FDLT, 0x00);
  
  ///< more: see Application Note AN3891

  writeRegister(MPR121_DEBOUNCE, 0);
  writeRegister(MPR121_CONFIG1, 0x10); // default, 16uA charge current
  writeRegister(MPR121_CONFIG2, 0x20); // 0.5uS encoding, 1ms period

  if (autoconfigEnabled)
  {
    writeRegister(MPR121_AUTOCONFIG0, 0x0B);

    // correct values for Vdd = 3.3V
    writeRegister(MPR121_UPLIMIT, 200);     // ((Vdd - 0.7)/Vdd) * 256
    writeRegister(MPR121_TARGETLIMIT, 180); // UPLIMIT * 0.9
    writeRegister(MPR121_LOWLIMIT, 130);    // UPLIMIT * 0.65
  }
  else
  {
    /* do not set up electrodes automatically */
  }

  return true;
}

/*!
 *  @brief      writes to MPR121_ECR. 0x00 puts device into Stop Mode.
 *              0x8F enables all electrodes but not the proximity sensing, with baseline tracking
 *              0x4F enables all electrodes without proximity sensing nor baseline tracking
 *  @param      ecr
 *              uint8_t byte data to control electrodes and baseline-tracking
 */

void Adafruit_MPR121_mod::begin(uint8_t ecr) {
    // enable X electrodes and start MPR121
  writeRegister(MPR121_ECR, 0x0);  
  writeRegister(MPR121_ECR, ecr); 
}

/*!
 *  @brief      DEPRECATED. Use Adafruit_MPR121_mod::setThresholds(uint8_t touch,
 *              uint8_t release) instead.
 *  @param      touch
 *              see Adafruit_MPR121_mod::setThresholds(uint8_t touch, uint8_t
 * *release)
 *  @param      release
 *              see Adafruit_MPR121_mod::setThresholds(uint8_t touch, *uint8_t
 * release)
 */
void Adafruit_MPR121_mod::setThreshholds(uint8_t touch, uint8_t release) {
  setThresholds(touch, release);
}

/*!
 *  @brief      Set the touch and release thresholds for all 13 channels on the
 *              device to the passed values. The threshold is defined as a
 *              deviation value from the baseline value, so it remains constant
 * even baseline value changes. Typically the touch threshold is a little bigger
 * than the release threshold to touch debounce and hysteresis. For typical
 * touch application, the value can be in range 0x05~0x30 for example. The
 * setting of the threshold is depended on the actual application. For the
 * operation details and how to set the threshold refer to application note
 * AN3892 and MPR121 design guidelines.
 *  @param      touch
 *              the touch threshold value from 0 to 255.
 *  @param      release
 *              the release threshold from 0 to 255.
 */
void Adafruit_MPR121_mod::setThresholds(uint8_t touch, uint8_t release) {
  // first stop sensor to make changes
  // writeRegister(MPR121_ECR, 0x00);
  // set all thresholds (the same)
  for (uint8_t i = 0; i < 12; i++) {
    writeRegister(MPR121_TOUCHTH_0 + 2 * i, touch);
    writeRegister(MPR121_RELEASETH_0 + 2 * i, release);
  }
  // turn the sensor on again
  // writeRegister(MPR121_ECR, 0x8F);
}

/*!
 *  @brief      Read the filtered data from channel t. The ADC raw data outputs
 *              run through 3 levels of digital filtering to filter out the high
 * frequency and low frequency noise encountered. For detailed information on
 * this filtering see page 6 of the device datasheet.
 *  @param      t
 *              the channel to read
 *  @returns    the filtered reading as a 10 bit unsigned value
 */
uint16_t Adafruit_MPR121_mod::filteredData(uint8_t t) {
  if (t > 12)
    return 0;
  return readRegister16(MPR121_FILTDATA_0L + t * 2);
}

/*!
 *  @brief      Read the filtered data from channel t, 2^n times, and return the average. The ADC raw data outputs
 *              run through 3 levels of digital filtering to filter out the high
 * frequency and low frequency noise encountered. For detailed information on
 * this filtering see page 6 of the device datasheet.
 *  @param      t
 *              the channel to read
 *  @param      n
 *              2^n samples taken
 *  @returns    the filtered average reading as a 10 bit unsigned value
 */
uint16_t Adafruit_MPR121_mod::filteredDataAveraged(uint8_t t, uint8_t n) {
  uint16_t avg = 0;
  uint16_t n2 = 1 << n;
  for (uint8_t i = 0; i < n2; i++) {
    avg += filteredData(t);
  }
  avg = avg >> n;
  return avg;
}

/*!
 *  @brief      Read the baseline value for the channel. The 3rd level filtered
 *              result is internally 10bit but only high 8 bits are readable
 * from registers 0x1E~0x2A as the baseline value output for each channel.
 *  @param      t
 *              the channel to read.
 *  @returns    the baseline data that was read
 */
uint16_t Adafruit_MPR121_mod::baselineData(uint8_t t) {
  if (t > 12)
    return 0;
  uint16_t bl = readRegister8(MPR121_BASELINE_0 + t);
  return (bl << 2);
}

/**
 *  @brief      Read the touch status of all 13 channels as bit values in a 12
 * bit integer.
 *  @returns    a 12 bit integer with each bit corresponding to the touch status
 *              of a sensor. For example, if bit 0 is set then channel 0 of the
 * device is currently deemed to be touched.
 */
uint16_t Adafruit_MPR121_mod::touched(void) {
  uint16_t t = readRegister16(MPR121_TOUCHSTATUS_L);
  return t & 0x0FFF;
}

/*!
 *  @brief      Read the contents of an 8 bit device register.
 *  @param      reg the register address to read from
 *  @returns    the 8 bit value that was read.
 */
uint8_t Adafruit_MPR121_mod::readRegister8(uint8_t reg) {
  _wire->beginTransmission(_i2caddr);
  _wire->write(reg);
  _wire->endTransmission(false);
  _wire->requestFrom(_i2caddr, 1);
  if (_wire->available() < 1)
    return 0;
  return (_wire->read());
}

/*!
 *  @brief      Read the contents of a 16 bit device register.
 *  @param      reg the register address to read from
 *  @returns    the 16 bit value that was read.
 */
uint16_t Adafruit_MPR121_mod::readRegister16(uint8_t reg) {
  _wire->beginTransmission(_i2caddr);
  _wire->write(reg);
  _wire->endTransmission(false);
  _wire->requestFrom(_i2caddr, 2);
  if (_wire->available() < 2)
    return 0;
  uint16_t v = _wire->read();
  v |= ((uint16_t)_wire->read()) << 8;
  return v;
}

/*!
    @brief  Writes 8-bits to the specified destination register
    @param  reg the register address to write to
    @param  value the value to write
*/
void Adafruit_MPR121_mod::writeRegister(uint8_t reg, uint8_t value) {
  // MPR121 must be put in Stop Mode to write to most registers
  bool stop_required = true;
  uint8_t ECR = readRegister8(
      MPR121_ECR); // first get the current set value of the MPR121_ECR register
  // if (reg == MPR121_ECR || (0x73 <= reg && reg <= 0x7A)) {
    if ((0x73 <= reg && reg <= 0x7A) || (reg == 0x5E)) { // seems that you need to write 0x0 to MPR121_ECR before changing its value
    // Serial.println("stop_required set to false!");
    stop_required = false;
  }
  if (stop_required) {
    _wire->beginTransmission(_i2caddr);
    _wire->write(MPR121_ECR);
    _wire->write((byte)0x00); // clear this register to set stop modus
    _wire->endTransmission();
  }

  _wire->beginTransmission(_i2caddr);
  _wire->write((uint8_t)reg);
  _wire->write((uint8_t)(value));
  _wire->endTransmission();

  if (stop_required) {
    _wire->beginTransmission(_i2caddr);
    _wire->write(MPR121_ECR);
    _wire->write(ECR); // write back the previous ECR settings
    _wire->endTransmission();
  }
}

/*!
 *  @brief      Prints status and electrode value registers to terminal.
 */
void Adafruit_MPR121_mod::printStatus (void) {
  
/*     Print touch status registers */
    Serial.println("TOUCH STATUS REGISTERS");

    for (uint8_t i = 0x00; i < 0x02; i++) {
        Serial.print("$"); Serial.print(i, HEX);
        Serial.print(": 0x"); Serial.println(readRegister8(i), HEX);
        }
    
    Serial.println();

    Serial.println("ERROR STATUS REGISTERS");

/*     Print error status registers! see application sheet 10-11 pages 
    tried with 1 electrode wired in will probaby raise flags! 
    See if connected electrode port raises any! */
    for (uint8_t i = 0x02; i < 0x04; i++) {
        Serial.print("$"); Serial.print(i, HEX);
        Serial.print(": 0x"); Serial.println(readRegister8(i), HEX);
        }

    Serial.println();

/*     Print output registers */
    Serial.println("FILTERED OUTPUT");
    for (uint8_t i = 0; i < 12; i++) {
      Serial.print("ELE"); Serial.print(i, DEC);
      Serial.print(": "); Serial.println(filteredData(i), DEC);
    }

    Serial.println();

/*     Print baseline register values */
    Serial.println("BASELINE VALUES");
    for (uint8_t i=0; i<12; i++) {
      Serial.print("ELE"); Serial.print(i, DEC);
      Serial.print(": "); Serial.println(baselineData(i), DEC);
    }

    Serial.println();
}

/*!
 *  @brief      Calculates statistics and prints std deviation of electrode values to terminal
 *  @param      eleccount number of active electrodes
 *  @param      factor 2^factor measurements taken consecutively 
 */
void Adafruit_MPR121_mod::calcStatistic (uint8_t eleccount, uint8_t factor) {

    if (factor > 6) {factor = 6;}

    uint16_t sum;
    uint16_t avg;
    uint8_t stddev;
    int8_t diff;  
    uint16_t varsum;
    uint16_t datapoints = 1 << factor;
    uint16_t filteredPoints [eleccount][datapoints];
    Serial.print("Number of active electrodes: "); Serial.println(eleccount);
    Serial.print("Number of datapoints: "); Serial.println(datapoints);

    for (uint8_t i = 0; i < eleccount; i++) {
      sum = 0;
      avg = 0;
      stddev = 0;
      diff = 0;
      varsum = 0;
    
      for (uint8_t j = 0; j < datapoints; j++) { 
        filteredPoints [i][j] = filteredData(i);
        sum += filteredPoints[i][j];
      }
      
      avg = sum >> factor; 
      
      for (uint8_t j = 0; j < datapoints; j++) { 
        diff = filteredPoints[i][j] - avg;
        varsum = varsum + pow(diff,2);
      }      
      stddev = sqrt(varsum >> factor);

      Serial.print(i); Serial.print(" electrode std dev value: "); Serial.println(stddev); Serial.println();
    }

}

/*!
 *  @brief      Handles flags for ON and OFF light shows to be controlled via timer interrupts
 *  @param      turnedon 
 *              DEMO application functionality flag
 *  @param      turn 
 *              pointer to turning on flag indicating turnedon flag status change
 *  @param      ticks p
 *              ointer to timer overflow counter
 *  @param      in_intensity 
 *              pointer to inwards facing LED ring intensity value
 *  @param      out_intensity
 *              pointer to outwards facing LED ring intensity value
 *  @param      blinkcount 
 *              pointer to blink counter while turning ON
 */
uint8_t Adafruit_MPR121_mod::manners (volatile bool turnedon, 
                                  volatile bool * turn, 
                                  volatile uint8_t * ticks,
                                  uint16_t * in_intensity, 
                                  uint16_t * out_intensity, 
                                  uint8_t * blinkcount) 
{
  uint8_t retval = 1u;
  if (!turnedon && !(*turn) ) { //then start turning on...
    *turn = 1;
    *ticks = pauseTicks;
    *in_intensity = 0; //and say goodday!
    *out_intensity = 0;
    *blinkcount = (onBlinks<<1);
    Serial.println("Turning on!"); 
    retval = 0u;
  }
  else if (turnedon && !(*turn) ) { //then start turning off...
    *turn = 1;
    *ticks = offTicks;
    *in_intensity = 255; //and say goodbye!
    *out_intensity = 255;    
    Serial.println("Turning off!"); 
    retval = 0u;
  }
  else 
  {
    /* Do nothing */
  }
  return retval;
}

/*!
 *  @brief      Default constructor
 */
TurnwheelDEMO::TurnwheelDEMO() {}

/*!
 *  @brief      Calculates sliding state and sets internal flags
 *  @param      cancelval 
 *              calculated cancellance
 *  @param      intstart 
 *              pointer to first member of current light intensities
 *  @param      intend
 *              pointer to last member of current light intensities
 *  @param      lastintstart 
 *              pointer to first member of previous light intensities
 *  @param      lastintend
 *              pointer to last member of previous light intensities
 *  @param      rightslide 
 *              pointer to flag indicating a clockwise sliding motion
 *  @param      leftslide 
 *              pointer to flag indicating a counterclockwise sliding motion
 *  @return     sliding value proportional to sliding touch intensity
 */
uint16_t TurnwheelDEMO::calcSliding(uint16_t cancelval, 
                                      uint16_t * intstart, 
                                      uint16_t * intend, 
                                      uint16_t * lastintstart, 
                                      uint16_t * lastintend, 
                                      bool * rightslide, 
                                      bool * leftslide)
{
  uint16_t slideval = 0;
  uint16_t slideleft = 0;
  uint16_t slideright = 0;
  uint16_t * prev = NULL;
  uint16_t * next = NULL;
  uint16_t * lastprev = NULL;
  uint16_t * lastnext = NULL;
  uint16_t * temp = intstart;
  uint16_t * lasttemp = lastintstart;

  while (temp != (intend + 1))
  {
    if (temp == intstart) 
    { 
      prev = intend; 
      lastprev = lastintend; 
      next = temp + 1; 
      lastnext = lasttemp + 1;
    }
    else if (temp == intend) 
    { 
      prev = temp - 1; 
      lastprev = lasttemp - 1; 
      next = intstart; 
      lastnext = lastintstart;
    }
    else 
    { 
      prev = temp - 1; 
      lastprev = lasttemp - 1; 
      next = temp + 1; 
      lastnext = lasttemp + 1;
    } 

    if ( (*prev > *lastprev) && (*temp < *lasttemp) )
    {
      slideleft += (*lasttemp - *temp) + (*prev - *lastprev);
    }
    else if ( (*prev < *lastprev) && (*temp > *lasttemp) )
    {
      slideright += (*temp - *lasttemp) + (*lastprev - *prev);
    }
    if ( (*next < *lastnext) && (*temp > *lasttemp) )
    {
      slideleft += (*temp - *lasttemp) + (*lastnext - *next);
    }
    else if ( (*next > *lastnext) && (*temp < *lasttemp) )
    {
      slideright += (*lasttemp - *temp) + (*next - *lastnext);
    }

    if (slideleft > cancelval) 
    {
      slideleft -= cancelval;
    } 
    else
    {
      slideleft = 0;
    }
        if (slideright > cancelval) 
    {
      slideright -= cancelval;
    } 
    else
    {
      slideright = 0;
    }

    temp++;
    lasttemp++;
  }

  if (slideright > slideleft)
    {
      *rightslide = true;
      *leftslide = false;
      slideval = slideright - slideleft;
    }
    else if (slideright < slideleft)
    {
      *rightslide = false;
      *leftslide = true;
      slideval = slideleft - slideright;
    }
    else
    {
      *rightslide = false;
      *leftslide = false;
    }

  return slideval;
}

/*!
 *  @brief      Default constructor
 */
SinusDEMO::SinusDEMO() {}

/*!
 *  @brief      Sets internal flags and light intensity values for Sinewave DEMO blinking
 *  @param      blinkcount 
 *              pointer to blink counter
 *  @param      ticks 
 *              pointer to timer overflow counter
 *  @param      in_intensity
 *              pointer to inwards light intensity
 *  @param      out_intensity 
 *              pointer to outwards light intensity
 *  @param      pausetime
 *              for how many timer overflows are LEDs OFF
 *  @param      blinktime 
 *              for how many timer overflows are LEDs ON
 *  @param      maxintensity 
 *              light intensity when LEDs are ON
 */
void SinusDEMO::blink (uint8_t * blinkcount, 
                   volatile uint8_t * ticks,  
                   uint16_t * in_intensity,  
                   uint16_t * out_intensity, 
                   uint8_t pausetime, 
                   uint8_t blinktime, 
                   uint8_t maxintensity) 
{
  if ( (*blinkcount) > 0) {
    if ( ( (*blinkcount) % 2 ) == 0 ) {
        (*in_intensity) = maxintensity; 
        (*out_intensity) = maxintensity; 
        (*ticks) = blinktime; 
        (*blinkcount)--;
        }
      else {
        (*in_intensity) = 0; 
        (*out_intensity) = 0; 
        (*ticks) = pausetime; 
        (*blinkcount)--;
        }
  }
}