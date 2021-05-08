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

/** Light intensity scaling */
#define MULTIPLIER 2.0f

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
#define TOUCH   3u
#define RELEASE 2u
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
/** Internal definitions *****************************************************/

 /** software version */
#define SOFTWARE_VERSION 1
#define SOFTWARE_SUBVERSION 0
#define SOFTWARE_NAME "Turnwheel DEMO"

/* Internal Variables */

// timing of status print
uint16_t count = 0;

// You can have up to 4 on one i2c bus.
TurnwheelDEMO cap = TurnwheelDEMO();

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

// keep track of the timer overflow interrupts. Max value 255 for timers 0,2 on Arduino (here timer 2 is used)
volatile uint8_t ticking = 0; //because it is modified in ISR needs to be defined as volatile!
// turn ON blinks
uint8_t blink_count = 0;

// definition of ON/OFF state
volatile bool turned_on = false; //because it is modified in ISR needs to be defined as volatile!
volatile bool turning = false; //because it is modified in ISR needs to be defined as volatile!
// definition of turnwheel/sliding state
bool slidingLeft = false;
bool slidingRight = false;

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
  
  for (uint8_t i=0; i<ELCOUNT; i++) 
  {

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
      if (currbaseline[i] > (filtered[i] + TOUCH)) 
      {
        intensity[i] = currbaseline[i] - filtered[i] - TOUCH;
      }
      else 
      {
        intensity[i] = 0;
      }       
    }
    else
    {
      intensity[i] = 0;
    }
  }

  /* ON/OFF as the 0th electrode is touched */
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
    slideIntensity = cap.calcSliding(cancellance, &intensity[1], &intensity[ELCOUNT - REFCOUNT - 1], 
    &lastintensity[1], &lastintensity[ELCOUNT - REFCOUNT - 1], &slidingRight, &slidingLeft);
    slideIntensity = (uint16_t) (slideIntensity * MULTIPLIER);
  
    if (slidingRight) 
    {
      outwardsintensity += slideIntensity;
      inwardsintensity += slideIntensity;
    }
    else if (slidingLeft)
    {
      if (slideIntensity >= outwardsintensity) 
      {
        outwardsintensity = 0;
        inwardsintensity = 0;
      }
      else 
      {
        outwardsintensity -= slideIntensity;
        inwardsintensity -= slideIntensity;
      }
    } 
  }
  else if (!turned_on && !turning)
  {
    inwardsintensity = 0;
    outwardsintensity = 0;
  }
  else if (turned_on && turning) //while turning off...
  { 
    inwardsintensity = 255;
    outwardsintensity = 255;
  }
  else if (!turned_on && turning && (ticking == 0)) //while turning on...
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

  // PWM duty cycle above 170/255 yields little visible increase in radiance, but increases noise greatly:
  // Let's avoid it!
  if (inwardsintensity >= 170){
    inwardsintensity = 170;
    digitalWrite(INWARDS, HIGH);
  }
  else if (inwardsintensity <= 0){
    inwardsintensity = 0;
    digitalWrite(INWARDS, LOW);
  }
  else {
    analogWrite(INWARDS, inwardsintensity);
  }
  if (outwardsintensity >= 170){ 
    outwardsintensity = 170;
    digitalWrite(OUTWARDS, HIGH);
  }
  else if (outwardsintensity <= 0){
    outwardsintensity = 0;
    digitalWrite(OUTWARDS, LOW);
  }
  else {
    analogWrite(OUTWARDS, outwardsintensity);
  }
  
  #ifdef _LOGGING_
  if (count == 0) {
    /* comment out next line to print all config + electrode registers to terminal */
    /*cap.printStatus(); */ 

    Serial.print("multiplier: "); Serial.println(MULTIPLIER, DEC);
    Serial.print("Inwards intensity: "); Serial.println(inwardsintensity, DEC);
    Serial.print("Outwards intensity: "); Serial.println(outwardsintensity, DEC);
    for (uint8_t i = 0; i < ELCOUNT; i++) {
      Serial.print("Electrode "); Serial.print(i); 
      if (currwake & _BV(i)) {Serial.print(" awake,");}
      Serial.print(" intensity: "); Serial.println(intensity[i], DEC);
    }
    Serial.print("Cancellance: "); Serial.println(cancellance);

    /* comment out next line to calc statistics and print to terminal */
    /* cap.calcStatistic(ELCOUNT, 6); */
    /* low std deviation w/o change in touch conditions indicates low noise and good stability */
    count = 2000;
  }
  count--;
  #endif /** _LOGGING_ */
  
  // reset our state
  lasttouched = currtouched;
  lastwake = currwake;
}

ISR(TIMER2_OVF_vect)          // timer overflow interrupt service routine
{
  if (ticking > 0) {
    ticking--; 
    // Serial print occupies ISR for too long! Enable only for debugging.
    // Serial.print("ticking: "); 
    // Serial.println(ticking);
    }
  
  if ((ticking == 0) && turned_on && turning) {
    turned_on = 0;
    turning = 0;
    // Serial.println("Turned off."); 
  }
  else if ((ticking == 0) && !turned_on && turning && (blink_count == 0)) {
    turned_on = 1;
    turning = 0;
    // Serial.println("Turned on."); 
  }
}