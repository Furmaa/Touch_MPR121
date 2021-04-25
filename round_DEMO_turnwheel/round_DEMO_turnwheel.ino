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

// timing of status print
uint16_t count = 0;

// You can have up to 4 on one i2c bus.
Adafruit_MPR121 cap = Adafruit_MPR121();

// Keeps track of the last pins touched
// so we know when buttons are 'released'
uint16_t lasttouched = 0;
uint16_t currtouched = 0;
uint16_t lastwake = 0;
uint16_t currwake = 0;

// Number of electrodes to be used, max. 12: 0,1,2....11
const uint8_t ELCOUNT = 9;
// Number of ref electrodes out of this: 0,1,2.....10 
// always attach to higher PINs on board! E.g. touch PINs 0,1 and ref PINs 2,3,4... 
const uint8_t REFCOUNT = 1; 
uint8_t k = 0;

// Initialize intensity values of the INWARDS and outwards
// facing LED series for the square-round DEMO board.
uint16_t inwardsintensity = 0;
uint16_t outwardsintensity = 0;
uint16_t cancellance = 0;
const uint8_t CANCELLANCE_TH = 1;
const uint8_t INWARDS_BASE = 100;
uint16_t slideIntensity = 0;
float multiplier = 2.0;

// initialize arrays for basline and filtered output values of electrodes
uint16_t firstbaseline[ELCOUNT] = {0};
uint16_t currbaseline[ELCOUNT] = {0};
uint16_t filtered[ELCOUNT] = {0};
uint16_t intensity[ELCOUNT] = {0};
uint16_t lastintensity[ELCOUNT] = {0};

// Change touch and release from default if needed in range 0-255
// Touch condition: Baseline - filtered output > touch threshold
// Release condition: Baseline - filtered output < release threshold
const uint8_t TOUCH = 3;
const uint8_t RELEASE = 2;
// uint8_t touch = MPR121_TOUCH_THRESHOLD_DEFAULT;
// uint8_t release = MPR121_RELEASE_THRESHOLD_DEFAULT;
// (Product Specification sheet section 5.6)

// keep track of the timer overflow interrupts. Max value 255 for timers 0,2 on Arduino (here timer 2 is used)
volatile uint8_t ticking = 0; //because it is modified in ISR needs to be defined as volatile!
const uint8_t TICK_OFF = 61; //61 ticks means a second, 1 tick is one counter overflow
// turn ON blinks
uint8_t blink_count = 0;
const uint8_t BLINKS = 3;
const uint8_t BLINK_TICKS = 10;
const uint8_t PAUSE_TICKS = 20;
//total turning on time: tick_ON = BLINKS * ( BLINK_TICKS + PAUSE_TICKS )

// Touch and release debounce values: how many times
// MPR121 rejects the change of touch and release status.
// Increases resistance to noise, slows detection. Range 0-7.
const uint8_t DEBOUNCE_TOUCH = 1;
const uint8_t DEBOUNCE_RELEASE = 0;
// (Product Specification sheet section 5.7)

// definition of ON/OFF state
volatile bool turned_on = false; //because it is modified in ISR needs to be defined as volatile!
volatile bool turning = false; //because it is modified in ISR needs to be defined as volatile!
// definition of turnwheel/sliding state
bool slidingLeft = false;
bool slidingRight = false;

// Active electrode configuration: see Adafruit_MPR121_mod.h for details
const uint8_t ECR = MPR121_BL_TRACKING_ALL + MPR121_ELEPROX_EN_OFF + ELCOUNT; 
// uint8_t ecr =  MPR121_ECR_SETTING_DEFAULT;
// (Product Specification sheet section 5.11)

// Definition of the LED control pins, see schematics.
const uint8_t INWARDS =  10;      
const uint8_t OUTWARDS = 6;       

void setup()
{
    Serial.begin(115200);
    Wire.begin();

    while (!Serial) { // needed to keep leonardo/micro from starting too fast!
        delay(10);
    }

    // .open starts communication with board at given address
    cap.open(MPR121_I2CADDR_ADAFRUIT);
    // Addressing: see Adafruit_MPR121_mod.h and Product Specification sheet section 5.11

    // .init makes soft reset and writes default config values, leaves board in Stop Mode
    if (!cap.init(TOUCH, TOUCH)) {
        Serial.println("MPR121 not found, check wiring?");
        while(!cap.init(TOUCH, TOUCH)){
          delay(1000);
          }
    }
    Serial.println("MPR121 found!"); 

    // comment AUTOCONFIG in Adafruit_MPR121_mod.cpp and 
    // alter values to adapt e.g. to Vdd != 3.3V
    #ifndef AUTOCONFIG
        cap.writeRegister(MPR121_AUTOCONFIG0, 0x0B);

        // baseline search values 
        cap.writeRegister(MPR121_UPLIMIT, 250);     // ((Vdd - 0.7)/Vdd) * 256 for Vdd = 3.3V : 200
        cap.writeRegister(MPR121_TARGETLIMIT, 180); // UPLIMIT * 0.9 for Vdd = 3.3V : 180
        cap.writeRegister(MPR121_LOWLIMIT, 130);    // UPLIMIT * 0.65 for Vdd = 3.3V : 130
        // baseline should be then between UPLIMIT and LOWLIMIT!
    #endif

    //if Baseline tracking is too fast: 
    //baseline value gets adjusted during the touch action, hence no touch detection!
    //try to set ESI value from Adafruit default 000 (1 ms, maximum scanning freq.) to slower... 
    //with encoding 100 (16ms, datasheet default) it works pretty well: recognizes swift and slow touches alike
    //cap.writeRegister(MPR121_CONFIG2, 0x33);
    
    //or it might be better to play around with the baseline filter values: 
    // with that the baseline tracking characteristics can be optimized.
    //Overwrite example:
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
    //(Application Note AN3891, Product Specification sheet section 5.5) 

    //if baseline tracking disabled, baseline values can be hardcoded!
    //cap.writeRegister(MPR121_BASELINE_0 + electrodeNumber, 181);

    //set own touch and release thresholds for 8th ref electrode
    cap.writeRegister(MPR121_TOUCHTH_0 + 2 * 8, 1);
    cap.writeRegister(MPR121_RELEASETH_0 + 2 * 8, 0);


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
    // it if *is* touched and *wasnt* touched before, alert!
    if ((currtouched & _BV(i)) && !(lasttouched & _BV(i)) ) {
      Serial.print(i); Serial.print(" touched! Reading:"); Serial.println(cap.filteredData(i));
      // printStatus(&cap);
    }
    // if it *was* touched and now *isnt*, alert!
    if (!(currtouched & _BV(i)) && (lasttouched & _BV(i)) ) {
      Serial.print(i); Serial.print(" released! Reading:");Serial.println(cap.filteredData(i));
      // printStatus(&cap);
    }

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
      /* Serial.print("filteredDataAveraged: "); Serial.println(filtered[i]);*/      currbaseline[i] = cap.baselineData(i);
      if (currbaseline[i] > (filtered[i] + TOUCH)) {intensity[i] = currbaseline[i] - filtered[i] - TOUCH;}
      else {intensity[i] = 0;}     
    }
    else
    {
      intensity[i] = 0;
    }
  }

//  // turned_on while 0th electrode touched!
//  if ((currtouched & _BV(0)) && !(lasttouched & _BV(0)) && !(currtouched & _BV(1)) && !(currtouched & _BV(7)) ){
//    turned_on = true;
//  }
//  if (!(currtouched & _BV(0)) && (lasttouched & _BV(0)) ) {
//    turned_on = false;
//  }

  // ON/OFF as the 0th electrode is touched
  if ( (currtouched & _BV(0)) && !(lasttouched & _BV(0)) && !(currtouched & ~_BV(0)) )
  {
    if (manners(turned_on, &turning, &ticking, PAUSE_TICKS, TICK_OFF, &inwardsintensity, &outwardsintensity, &blink_count, BLINKS)) 
    {
      Serial.println("ERROR: this fella has no manners...");
    }
  }
  
  cancellance = intensity[8];
  
  if (turned_on && !turning) 
  {
    slideIntensity = calcSliding(cancellance, &intensity[1], &intensity[ELCOUNT - REFCOUNT - 1], 
    &lastintensity[1], &lastintensity[ELCOUNT - REFCOUNT - 1], &slidingRight, &slidingLeft);
    slideIntensity *= multiplier;
  
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
  
  if (count == 0) {
    //printStatus(&cap);
    Serial.print("Multiplier: "); Serial.println(multiplier, DEC);
    Serial.print("Inwards intensity: "); Serial.println(inwardsintensity, DEC);
    Serial.print("Outwards intensity: "); Serial.println(outwardsintensity, DEC);
    for (uint8_t i = 0; i < ELCOUNT; i++) {
      Serial.print("Electrode "); Serial.print(i); 
      if (currwake & _BV(i)) {Serial.print(" awake,");}
      Serial.print(" intensity: "); Serial.println(intensity[i], DEC);
    }
    Serial.print("Cancellance: "); Serial.println(cancellance);
    //printStatistic(&cap, ELCOUNT, 6);
    count = 2000;
  }
  count--;
  
  // reset our state
  lasttouched = currtouched;
  lastwake = currwake;
}

ISR(TIMER2_OVF_vect)          // timer overflow interrupt service routine
{
  if (ticking > 0) {
    ticking--; 
    // Serial.print("ticking: "); // Serial print occupies ISR for too long! Enable only for debugging.
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

uint16_t calcSliding(uint16_t cancelval, uint16_t * intstart, uint16_t * intend, 
uint16_t * lastintstart, uint16_t * lastintend, bool * rightslide, bool * leftslide)
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

uint8_t manners (volatile bool turnedon, volatile bool * turn, volatile uint8_t * ticks, uint8_t pause, uint8_t offticks, uint16_t * in_intensity, uint16_t * out_intensity, uint8_t * blinkcount, uint8_t blinkinit) {
  if (!turnedon && !(*turn) ) { //then start turning on...
    *turn = 1;
    *ticks = pause;
    *in_intensity = 0; //and say goodday!
    *out_intensity = 0;
    *blinkcount = (blinkinit<<1);
    Serial.println("Turning on!"); 
  }
  else if (turnedon && !(*turn) ) { //then start turning off...
    *turn = 1;
    *ticks = offticks;
    *in_intensity = 255; //and say goodbye!
    *out_intensity = 255;    
    Serial.println("Turning off!"); 
  }
  return (uint8_t) 0;
}

void printStatistic (Adafruit_MPR121 * obj, uint8_t eleccount, uint8_t count) {

    if (count > 6) {count = 6;}

    uint16_t sum;
    uint16_t avg;
    uint8_t stddev;
    int8_t diff;  
    uint16_t varsum;
    uint16_t datapoints = 1 << count;
    uint16_t filtered [eleccount][datapoints];
    Serial.print("Number of active electrodes: "); Serial.println(eleccount);
    Serial.print("Number of datapoints: "); Serial.println(datapoints);

    for (uint8_t i = 0; i < eleccount; i++) {
      sum = 0;
      avg = 0;
      stddev = 0;
      diff = 0;
      varsum = 0;
    
      for (uint8_t j = 0; j < datapoints; j++) { 
        filtered [i][j] = obj->filteredData(i);
        sum += filtered[i][j];
        // Serial.print("electrode ");
        // Serial.print(i);
        // Serial.print(", datapoint ");
        // Serial.print(j);
        // Serial.print("- value:");
        // Serial.println(filtered[i][j]);
        // Serial.print("Sum: ");
        // Serial.println(sum);
      }
      
      avg = sum >> count; 
      
      for (uint8_t j = 0; j < datapoints; j++) { 
        diff = filtered[i][j] - avg;
        varsum = varsum + pow(diff,2);
      }      
      stddev = sqrt(varsum >> count);

      Serial.print(i); Serial.print(" electrode std dev value: "); Serial.println(stddev); Serial.println();
    }

}

void printStatus (Adafruit_MPR121 * obj) {
  
    // Print touch status registers: covered pretty much already above, just for checking
    Serial.println("TOUCH STATUS REGISTERS");

    for (uint8_t i = 0x00; i < 0x02; i++) {
        Serial.print("$"); Serial.print(i, HEX);
        Serial.print(": 0x"); Serial.println(obj->readRegister8(i), HEX);
        }
    
    Serial.println();

    Serial.println("ERROR STATUS REGISTERS");

    // Print error status registers! see application sheet 10-11 pages 
    // tried with 1 electrode wired in will probaby raise flags! See if connected electrode port raises any!
    for (uint8_t i = 0x02; i < 0x04; i++) {
        Serial.print("$"); Serial.print(i, HEX);
        Serial.print(": 0x"); Serial.println(obj->readRegister8(i), HEX);
        }

    Serial.println();

    // Print output registers
    Serial.println("FILTERED OUTPUT");
    for (uint8_t i = 0; i < 12; i++) {
      Serial.print("ELE"); Serial.print(i, DEC);
      Serial.print(": "); Serial.println(obj->filteredData(i), DEC);
    }

    Serial.println();

    // Print baseline register values
    Serial.println("BASELINE VALUES");
    for (uint8_t i=0; i<12; i++) {
      Serial.print("ELE"); Serial.print(i, DEC);
      Serial.print(": "); Serial.println(obj->baselineData(i), DEC);
    }

    Serial.println();
}
