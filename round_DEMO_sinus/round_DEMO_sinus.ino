/*********************************************************
This is a library for the MPR121 12-channel Capacitive touch sensor

Designed specifically to work with the MPR121 Breakout in the Adafruit shop 
  ----> https://www.adafruit.com/products/

These sensors use I2C communicate, at least 2 pins are required 
to interface

Adafruit invests time and resources providing this open source code, 
please support Adafruit and open-source hardware by purchasing 
products from Adafruit!

Written by Limor Fried/Ladyada for Adafruit Industries.  
BSD license, all text above must be included in any redistribution
**********************************************************/

#include "Adafruit_MPR121_mod.h"

/*****************************************************************************/
/** General application setup ************************************************/

#ifdef _LOGGING_
#undef _LOGGING_
#endif /** _LOGGING_ */
/** comment out next line to enable logging to terminal */
/* #define _LOGGING_ */

/** Serial communication setup */
#define BAUDRATE 115200l

/** Definition of the LED control pins, see DEMO board schematics in folder:
 * DEMO_touch_MPR121_circuit/...*/
#define INWARDS 10u  
#define OUTWARDS 6u    

/** Number of electrodes to be used, max. 12: 0,1,2....11 */
#define ELCOUNT 9u
/** Number of ref electrodes out of this: 0,1,2.....10 
  * always attach to higher PINs on board! E.g. touch PINs 0,1 and ref PINs 2,3,4... */
#define REFCOUNT 1u

/** Change touch and release thresholds from default if needed in range 0-255
  * Touch condition: Baseline - filtered output > touch threshold
  * Release condition: Baseline - filtered output < release threshold */
#define TOUCH   6u
#define RELEASE 3u
/** default values: 
  *  MPR121_TOUCH_THRESHOLD_DEFAULT
  *  MPR121_RELEASE_THRESHOLD_DEFAULT
  * (Product Specification sheet section 5.6) */
 /** Comment below two lines if you do not want separate thresholds for reference electrodes!*/
 #define REF_TOUCH 1u
 #define REF_RELEASE 0u

 /** Touch and release debounce values: how many times
  * MPR121 rejects the change of touch and release status.
  * Increases resistance to noise, slows detection. Range 0-7. */
#define DEBOUNCE_TOUCH    1u
#define DEBOUNCE_RELEASE  0u
/** (Product Specification sheet section 5.7) */

#define TICKSEC 61u /* 61 ticks means one second, 1 tick is one counter overflow */
#define OFF_TICKS TICKSEC 
#define BLINKS 3u
#define BLINK_TICKS 10u
#define PAUSE_TICKS 20u
/* total turning on time: tick_ON = BLINKS * ( BLINK_TICKS + PAUSE_TICKS ) */

// Active electrode configuration: see Adafruit_MPR121_mod.h for details
const uint8_t ECR = MPR121_BL_TRACKING_ALL + MPR121_ELEPROX_EN_OFF + ELCOUNT; 
/** default value: 
 *  MPR121_ECR_SETTING_DEFAULT
(Product Specification sheet section 5.11) */

/*****************************************************************************/
/** Application specific setup ***********************************************/

/** sinewave pulsatance change step */
#define OM_STEP (float)(PI/TICKSEC/4u)

/** light intensity change step */
#define INTENSITYSTEP    30u 

/** intermittent pulses -- "stroboscope" */ 
#define PULSE1_POS (float)( PI/3u )
#define PULSE2_POS (float)( PI/2u )
#define PULSE3_POS (float)( 2u*PI/3u)
#define HALFPULSEWIDTH (float)(PI/16u)

/** light intensity offset */
#define OFFSET 0u

/*****************************************************************************/
/** Internal definitions *****************************************************/

 /** software version */
#define SOFTWARE_VERSION 1
#define SOFTWARE_SUBVERSION 0
#define SOFTWARE_NAME "Sinewave DEMO"

/** sinewave pulsatance */
#define OMMAX (float)(PI/TICKSEC*4u)  
#define OMMIN (float)(PI/TICKSEC/2u)

/** light intensity boundaries */
#define INTENSITY_UPPER  200u
#define INTENSITY_LOWER  20u  

/** Internal Variables */

/** timing of status print */
uint16_t count = 0;

/** You can have up to 4 on one i2c bus. */
Adafruit_MPR121_mod cap = Adafruit_MPR121_mod();

/** states */
bool sinewave_out = true;
bool sinewave_in = true;
bool intermitt = true;
bool sinestart_in = false; // start of sinus wave flag
bool sinestart_out = false; // start of sinus wave flag

float om = OMMAX;
float phase = 0;
uint8_t sinetime = 0;
float sinintensity = 0;
const float pulse1_upperBound = (float)(PULSE1_POS + HALFPULSEWIDTH);
const float pulse1_lowerBound = (float)(PULSE1_POS - HALFPULSEWIDTH);
const float pulse2_upperBound = (float)(PULSE2_POS + HALFPULSEWIDTH);
const float pulse2_lowerBound = (float)(PULSE2_POS - HALFPULSEWIDTH);
const float pulse3_upperBound = (float)(PULSE3_POS + HALFPULSEWIDTH);
const float pulse3_lowerBound = (float)(PULSE3_POS - HALFPULSEWIDTH);

// Keeps track of the last pins touched
// so we know when buttons are 'released'
uint16_t lasttouched = 0;
uint16_t currtouched = 0;
uint16_t lastwake = 0;
uint16_t currwake = 0;

// Initialize intensity values of the INWARDS and outwards
// facing LED series for the square-round DEMO board.
uint16_t inwardsintensity = 0;
uint16_t outwardsintensity = 0;
uint16_t cancellance = 0;
uint16_t slideIntensity = 0;

// initialize arrays for basline and filtered output values of electrodes
uint16_t firstbaseline[ELCOUNT] = {0};
uint16_t currbaseline[ELCOUNT] = {0};
uint16_t filtered[ELCOUNT] = {0};
uint16_t intensity[ELCOUNT] = {0};
uint16_t lastintensity[ELCOUNT] = {0};

uint16_t intensitylvl = INTENSITY_UPPER;

// keep track of the timer overflow interrupts. Max value 255 for timers 0,2 on Arduino (here timer 2 is used)
volatile uint8_t ticking = 0; //because it is modified in ISR needs to be defined as volatile!
// turn ON blinks
uint8_t blink_count = 0;

// definition of ON/OFF state
volatile bool turned_on = false; //because it is modified in ISR needs to be defined as volatile!
volatile bool turning = false; //because it is modified in ISR needs to be defined as volatile! 

void setup()
{
    Serial.begin(BAUDRATE);
    Wire.begin();

    while (!Serial) { // needed to keep leonardo/micro from starting too fast!
        delay(10);
    }

    /* configure object */
    cap.offTicks = OFF_TICKS;
    cap.onBlinks = BLINKS;
    cap.pauseTicks = PAUSE_TICKS;

    // .open starts communication with board at given address
    cap.open(MPR121_I2CADDR_ADAFRUIT);
    // Addressing: see Adafruit_MPR121_mod.h and Product Specification sheet section 5.11

    /* .init makes soft reset and writes default config values, leaves board in Stop Mode */
    while(!cap.init(TOUCH, RELEASE, true)) 
    {
      Serial.println("MPR121 not found, check wiring?");
      delay(1000);
    }

    /** printing software info */
    Serial.println("MPR121 found!"); 
    Serial.print(SOFTWARE_NAME);
    Serial.print(", version running: v"); 
    Serial.print(SOFTWARE_VERSION, DEC); 
    Serial.print("."); 
    Serial.println(SOFTWARE_SUBVERSION, DEC); 

    /** if Baseline tracking is too fast: 
      * baseline value gets adjusted during the touch action, hence no touch detection!
      *try to set ESI value from Adafruit default 000 (1 ms, maximum scanning freq.) to slower... 
      *with encoding 100 (16ms, datasheet default) it works pretty well: recognizes swift and slow touches alike
      *
      * cap.writeRegister(MPR121_CONFIG2, 0x33);
      *
      *or it might be better to play around with the baseline filter values: 
      * with that the baseline tracking characteristics can be optimized.
      *Overwrite example: */
    cap.writeRegister(MPR121_MHDR, 0x06);
    cap.writeRegister(MPR121_NHDR, 0x05);    
    cap.writeRegister(MPR121_NCLR, 0x0A);
    cap.writeRegister(MPR121_FDLR, 0x50);
    
    cap.writeRegister(MPR121_MHDF, 0x06);
    cap.writeRegister(MPR121_NHDF, 0x01);
    cap.writeRegister(MPR121_NCLF, 0x20);
    cap.writeRegister(MPR121_FDLF, 0xB0);
    
    cap.writeRegister(MPR121_NHDT, 0x00); 
    cap.writeRegister(MPR121_NCLT, 0x00);
    cap.writeRegister(MPR121_FDLT, 0x00);
    /** (Application Note AN3891, Product Specification sheet section 5.5) */

    /* if baseline tracking disabled, baseline values can be hardcoded! */
    /* cap.writeRegister(MPR121_BASELINE_0 + electrodeNumber, 181); */

    /** set own touch and release thresholds for 8th ref electrode */
    #ifdef REF_TOUCH
    #ifdef REF_RELEASE
    for (uint8_t i = ELCOUNT; i > ELCOUNT - REFCOUNT; i--)
    {
      cap.writeRegister(MPR121_TOUCHTH_0 + 2 * i, REF_TOUCH);
      cap.writeRegister(MPR121_RELEASETH_0 + 2 * i, REF_RELEASE);
    }
    #endif /** ifdef REF_TOUCH */
    #endif /** ifdef REF_RELEASE */

    //applying debounce settings
    cap.writeRegister(MPR121_DEBOUNCE, DEBOUNCE_RELEASE<<4 + DEBOUNCE_TOUCH);
  
    // .begin writes ecr settings to put board in Run Mode
    cap.begin(ECR);

    delay(1000); // to make sure auto-config runs through

    // Print configuration registers
    Serial.println("CONFIG REGISTERS");
    for (uint8_t i = 0x2B; i < 0x80; i++) {
      Serial.print("$"); Serial.print(i, HEX);
      Serial.print(": 0x"); Serial.println(cap.readRegister8(i), HEX);
    }

    Serial.println();

    pinMode(INWARDS, OUTPUT);
    digitalWrite(INWARDS, LOW);
    pinMode(OUTWARDS, OUTPUT);
    digitalWrite(OUTWARDS, LOW);

    for (uint8_t i = 0; i < ELCOUNT; i++) {
      firstbaseline[i] = cap.baselineData(i);
    }

        // setup timer2 -- interferes with pwm pins 11, 3!

    noInterrupts(); 
    TCCR2A = 0;
    TCCR2B = 0;
    TCNT2  = 0;
    TIMSK2 = 0;
    TCCR2B |= (1 << WGM21);   // CTC mode
    TCCR2B |= (111 << CS20); // 1024 prescaler ---> 16000000 / 1024 / 255 = 61 overflows per second 
    TIMSK2 |= (1 << TOIE2);  // enable timer overflow, and compare interrupt
    interrupts();
}

void loop() {
//   Get the currently touched pads
  currtouched = cap.touched();
  currwake = 0;
  
  for (uint8_t i=0; i<ELCOUNT; i++) {
    
    #ifdef _LOGGING_
    /** it if *is* touched and *wasnt* touched before, alert! */
    if ((currtouched & _BV(i)) && !(lasttouched & _BV(i)) ) {
      Serial.print(i); Serial.print(" touched! Reading:"); Serial.println(cap.filteredData(i));
    }
    /** if it *was* touched and now *isnt*, alert! */
    if (!(currtouched & _BV(i)) && (lasttouched & _BV(i)) ) {
      Serial.print(i); Serial.print(" released! Reading:");Serial.println(cap.filteredData(i));
    }
    #endif /** _LOGGING_ */

    if (currtouched & _BV(i)) {
      currwake |= _BV(i);
      
      if ( (i+1) == (ELCOUNT - REFCOUNT) ) 
      { 
        currwake |= _BV(1); 
      }
      else if ( ((i+1) > 1) && ( (i+1) < (ELCOUNT - REFCOUNT) ) ) 
      { 
        currwake |= _BV(i+1); 
      }

      if ( (i-1) == 0 ) 
      { 
        currwake |= _BV(ELCOUNT - REFCOUNT - 1); 
      }
      else if ( ( (i-1) > 0 ) && ( (i-1) < (ELCOUNT - REFCOUNT - 1) ) ) 
      { 
        currwake |= _BV(i-1); 
      }
    }

  }

  for (uint8_t i=0; i<ELCOUNT; i++) 
  {
    lastintensity[i] = intensity[i]; // save state for turnwheel slide direction tracking
    if (currwake & _BV(i))
    {
      filtered[i] = cap.filteredDataAveraged(i, 2);
      #ifdef _LOGGING_
      Serial.print("filteredDataAveraged: "); Serial.println(filtered[i]);
      #endif /** _LOGGING_ */
      currbaseline[i] = cap.baselineData(i);
      if (currbaseline[i] > (filtered[i] + TOUCH)) {intensity[i] = currbaseline[i] - filtered[i] - TOUCH;}
      else {intensity[i] = 0;}     
    }
    else
    {
      intensity[i] = 0;
    }
  }

  /** ON/OFF as the 0th electrode is touched */
  if ( (currtouched & _BV(0)) && !(lasttouched & _BV(0)) && !(currtouched & ~_BV(0)) )
  {
    if (cap.manners(turned_on, &turning, &ticking, &inwardsintensity, &outwardsintensity, &blink_count)) 
    {
      #ifdef _LOGGING_
      Serial.println("ERROR: this fella has no manners...");
      #endif /** _LOGGING_ */
    }
  }

  /** calculate noise to be cancelled out */
  cancellance = 0u;
  for (uint8_t i = 0u; i < REFCOUNT; i++)
  {
    cancellance += intensity[ELCOUNT - 1 - i];
  }
  
  if (turned_on && !turning) 
  {
    /** different light shows for electrodes 1...7 */

    /** 1st electrode: increasing light intensity */
    if ( (currtouched & _BV(1)) && !(lasttouched & _BV(1)) && !(currtouched & ~_BV(1)) ) { //only the 1th electrode has been touched 
      if ( INTENSITY_UPPER >= ((uint16_t)(intensitylvl + 2*INTENSITYSTEP + OFFSET)) ) {
        intensitylvl += INTENSITYSTEP; 
        #ifdef _LOGGING_
        Serial.print("Maximal intensity: "); Serial.println(intensitylvl);
        #endif /** _LOGGING_ */
      }
      else 
      {
        intensitylvl = INTENSITY_UPPER;
        #ifdef _LOGGING_
        Serial.print("Intensity at maximum: "); Serial.println(intensitylvl);
        #endif /** _LOGGING_ */
      }
/*       turning = true;
      blink_count = 4; */
      if (!(sinewave_in) && !(sinewave_out)) {
        sinewave_in = true;
        sinewave_out = true;
      }    
    }

    /** 2nd electrode: decreasing light intensity */
    else if ( (currtouched & _BV(2)) && !(lasttouched & _BV(2)) && !(currtouched & ~_BV(2)) ) { //only the 2th electrode has been touched 
      if ( (uint16_t)(INTENSITY_LOWER + 2*INTENSITYSTEP + OFFSET) <= intensitylvl ) {
        intensitylvl -= INTENSITYSTEP;
        #ifdef _LOGGING_
        Serial.print("Maximal intensity: "); Serial.println(intensitylvl);
        #endif /** _LOGGING_ */ 
      }
      else {intensitylvl = INTENSITY_LOWER; Serial.print("Intensity at minimum: "); Serial.println(intensitylvl);}
/*       turning = true;
      blink_count = 2; */
      if (!(sinewave_in) && !(sinewave_out)) {
        sinewave_in = true;
        sinewave_out = true;
      }
    }

      /** 3rd electrode: sinus light outwards */
    else if ( (currtouched & _BV(3)) && !(lasttouched & _BV(3)) && !(currtouched & ~_BV(3)) ) 
    { //only the 3th electrode has been touched 
      (sinewave_out) = !(sinewave_out);
      if (!sinewave_out)
      {
        sinestart_out = false; 
      }
    }

      /** 4th electrode: sinus light inwards */
    else if ( (currtouched & _BV(4)) && !(lasttouched & _BV(4)) && !(currtouched & ~_BV(4)) ) 
    { //only the 4th electrode has been touched 
      (sinewave_in) = !(sinewave_in);
      if (!sinewave_in)
      {
        sinestart_in = false; 
      }
    }

      /** 5th electrode: increasing frequency of sine wave */
    else if ( (currtouched & _BV(5)) && !(lasttouched & _BV(5)) && !(currtouched & ~_BV(5)) ) { //only the 5th electrode has been touched 
      if ( OMMAX >= (om + OM_STEP)) 
      {
        om += OM_STEP; 
        #ifdef _LOGGING_
        Serial.print("Frequency increased: "); Serial.println(om);
        #endif /** _LOGGING_ */
      }
      else 
      {
        #ifdef _LOGGING_
        Serial.print("Frequency at maximum value: "); Serial.println(om);
        #endif /** _LOGGING_ */  
      }

      if (!(sinewave_in) && !(sinewave_out)) 
      {
        sinewave_in = true;
        sinewave_out = true;
        intermitt = true;
      }
    }

    /** 6th electrode: decreasing frequency of for sine wave */
    else if ( (currtouched & _BV(6)) && !(lasttouched & _BV(6)) && !(currtouched & ~_BV(6)) ) { //only the 6h electrode has been touched 
      if (om >= (OMMIN + OM_STEP)) 
      {
        om -= OM_STEP; 
        #ifdef _LOGGING_
        Serial.print("Frequency decreased: "); Serial.println(om);
        #endif /** _LOGGING_ */
      }
      else 
      {
        #ifdef _LOGGING_
        Serial.print("Frequency at minimum value: "); Serial.println(om);
        #endif /** _LOGGING_ */
      }

      if (!(sinewave_in) && !(sinewave_out)) 
      {
        sinewave_in = true;
        sinewave_out = true;
        intermitt = true;
      }
     }
      /** 7th electrode: */ 
    else if (  (currtouched & _BV(7)) && !(lasttouched & _BV(7)) && !(currtouched & ~_BV(7)) ) { //only the 7th electrode has been touched 
      (intermitt) = !(intermitt);
    }
  } 
  else if (!turned_on && !turning) /** while OFF */
  { 
    inwardsintensity = 0;
    outwardsintensity = 0;
  }
  else if (turned_on && turning) //&& (ticking == 0)) /**while turning off... */
  {
    inwardsintensity = 255;
    outwardsintensity = 255;
  }
  else if (!turned_on && turning && (ticking == 0)) /**while turning on...*/
  { 
    if ((blink_count % 2) == 0) {
      inwardsintensity = 255; 
      outwardsintensity = 255; 
      ticking = BLINK_TICKS; 
      blink_count--;
      }
    else {
      inwardsintensity = 0; 
      outwardsintensity = 0; 
      ticking = PAUSE_TICKS; 
      blink_count--;
      }
  }
  
  if ((sinewave_out || sinewave_in) && turned_on && !turning) 
  {
    sinetime = 0xFFu - ticking;
    phase = (float) om*sinetime;
    sinintensity = (float) ( (intensitylvl - OFFSET) * sin(phase) );
    sinintensity = abs(sinintensity);
    #ifdef _LOGGING_
      Serial.print("Phase: "); Serial.println(phase);
    #endif /** _LOGGING_ */

    if (phase < PI)
    {
      inwardsintensity = 0;
      if (sinewave_out && sinestart_out) 
      {
        outwardsintensity = (uint16_t) sinintensity;
      } 
      else
      {
        outwardsintensity = 0;
      }
    }
    else if (phase < 2*PI)
    {
      outwardsintensity = 0;
      if (sinewave_in && sinestart_in) 
      {
        inwardsintensity = (uint16_t) sinintensity;
      }
      else
      {
        inwardsintensity = 0;
      }
    }
    else
    {
      inwardsintensity = 0;
      outwardsintensity = 0;
      ticking = 0xFFu;
      if (sinewave_in)
      {
        sinestart_in = true;  
      }
      else
      {
        /** Do Nothing */
      } 
      if (sinewave_out)
      {
        sinestart_out = true;  
      }
      else
      {
        /** Do Nothing */
      } 
    }

    if (intermitt) 
    {
      if (phase <= pulse1_upperBound) 
      {
        if (phase >= pulse1_lowerBound)
        {
          inwardsintensity = 0;
          outwardsintensity = 0;
        }
        else        
        {
          /** Do Nothing */
        }
      }
      else if (phase <= pulse2_upperBound) 
      {
        if (phase >= pulse2_lowerBound)
        {
          inwardsintensity = 0;
          outwardsintensity = 0;
        }
        else
        {
          /** Do Nothing */
        }

      }
      else if (phase <= pulse3_upperBound)
      {
        if (phase >= (pulse3_lowerBound))
        {
          inwardsintensity = 0;
          outwardsintensity = 0;
        }
        else        
        {
          /** Do Nothing */
        }
      } 
      else        
      {
        /** Do Nothing */
      }
    }
    else        
    {
      /** Do Nothing */
    }
  }

  analogWrite(OUTWARDS, outwardsintensity);
  analogWrite(INWARDS, inwardsintensity);

  #ifdef _LOGGING_
  if (count == 0) {
    /** comment out next line to print all config + electrode registers to terminal */
    /*cap.printStatus(); */ 

    Serial.print("Inwards intensity: "); Serial.println(inwardsintensity, DEC);
    Serial.print("Outwards intensity: "); Serial.println(outwardsintensity, DEC);
    for (uint8_t i = 0; i < ELCOUNT; i++) {
      Serial.print("Electrode "); Serial.print(i); 
      if (currwake & _BV(i)) {Serial.print(" awake,");}
      Serial.print(" intensity: "); Serial.println(intensity[i], DEC);
    }
    Serial.print("Cancellance: "); Serial.println(cancellance);

    /** comment out next line to calc statistics and print to terminal */
    /* cap.calcStatistic(ELCOUNT, 6); */
    /** low std deviation w/o change in touch conditions indicates low noise and good stability */
    count = 2000;
  }
  count--;
  #endif /** _LOGGING_ */
  
  /** reset our state */
  lasttouched = currtouched;
  lastwake = currwake;
}

ISR(TIMER2_OVF_vect)          /** timer overflow interrupt service routine */
{
  if (ticking > 0) {
    ticking--; 
/*     Serial print occupies ISR for too long! Enable only for debugging.
    Serial.print("ticking: "); 
    Serial.println(ticking); */
    }
  else
  {
    /** Do nothing */
  }
  
  if ((ticking == 0) && turned_on && turning) {
    turned_on = 0;
    turning = 0;
    /* Serial.println("Turned off."); */ 
  }
  else if ((ticking == 0) && !turned_on && turning && (blink_count == 0)) {
    turned_on = 1;
    turning = 0;
    /* Serial.println("Turned on."); */ 
  }
  else
  {
    /** Do nothing */
  }
}