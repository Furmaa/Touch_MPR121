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

#define TICKSEC 61 //61 ticks is one second approx. with below configuration for timer 2  

// timing of status print
uint16_t count = 0;

// You can have up to 4 on one i2c bus.
Adafruit_MPR121 cap = Adafruit_MPR121();

// Keeps track of the last pins touched
// so we know when buttons are 'released'
uint16_t lasttouched = 0;
uint16_t currtouched = 0;

// Initialize intensity values of the INWARDS and outwards
// facing LED series for the square-round DEMO board.
uint8_t inwardsintensity = 0;
uint8_t outwardsintensity = 0;
uint8_t intensitylvl = 50;
const uint8_t INTENSITY_UPPER = 200;
const uint8_t INTENSITY_LOWER = 20;
const uint8_t INTENSITYSTEP = 30;
uint8_t offset = 0;
uint8_t cancellance = 0;
const uint8_t CANCELLANCE_TH = 2;
const uint8_t INWARDS_BASE = 0;

// Change touch and release from default if needed in range 0-255
// Touch condition: Baseline - filtered output > touch threshold
// Release condition: Baseline - filtered output < release threshold
const uint8_t TOUCH = 10;
const uint8_t RELEASE = 6;
// uint8_t touch = MPR121_TOUCH_THRESHOLD_DEFAULT;
// uint8_t release = MPR121_RELEASE_THRESHOLD_DEFAULT;
// (Product Specification sheet section 5.6)

// Number of electrodes to be used, max. 12: 1,2....12
const uint8_t ELCOUNT = 9; 

// initialize arrays for basline and filtered output values of electrodes
uint16_t firstbaseline[12] = {0};
uint16_t currbaseline[12] = {0};
uint16_t filtered[12] = {0};
uint16_t intensity[12] = {0};

// keep track of the timer overflow interrupts. Max value 255 for timers 0,2 on Arduino (here timer 2 is used)
uint8_t ticking = 0;
uint8_t firsttick = false;
const uint8_t TICK_OFF = TICKSEC; //61 ticks means a second, 1 tick is one counter overflow
// turn ON blinks
uint8_t blink_count = 0;
const uint8_t BLINKS = 3;
const uint8_t BLINK_TICKS = 10;
const uint8_t PAUSE_TICKS = 20;
//total turning on time: tick_ON = blink_count * ( blink_ticks + PAUSE_TICKS )

// Touch and release debounce values: how many times
// MPR121 rejects the change of touch and release status.
// Increases resistance to noise, slows detection. Range 0-7.
const uint8_t DEBOUNCE_TOUCH = 1;
const uint8_t DEBOUNCE_RELEASE = 0;
// (Product Specification sheet section 5.7)

// definition of ON/OFF
bool turned_on = false;
bool turning_on = false;
bool turning_off = false;
// other pressbuton states
const uint8_t BUTTONS = 3;
bool PRESSED[BUTTONS] = {false, false, false};
bool * const PRESSEDp = PRESSED;
bool * const SINEWAVE_OUT = &PRESSED[0];
bool * const SINEWAVE_IN = &PRESSED[1];
bool * const INTERMITT = &PRESSED[2];
bool sineend = true; // end of sinus wave flag

// sinewave pulsatance
float om = (float) (PI/TICKSEC/2);
const float OMMAX = (float) (PI/TICKSEC*2);  
const float OMMIN = (float) (PI/TICKSEC/8);
// sinewave intensity 
double sinintensity = 0;
// intermittent pulse 
const float PULSE1_POS = (float) ( PI/3 );
const float PULSE2_POS = (float) ( PI/2 );
const float PULSE3_POS = (float) ( 2*PI/3);
uint8_t pulse1_tick = 0;
uint8_t pulse2_tick = 0;
uint8_t pulse3_tick = 0;
const uint8_t HALFPULSEWIDTH = 1;
uint8_t intermitick = 0;

// Active electrode configuration: see Adafruit_MPR121_mod.h for details
const uint8_t ECR = MPR121_BL_TRACKING_ALL + MPR121_ELEPROX_EN_OFF + ELCOUNT; 
// uint8_t ecr =  MPR121_ECR_SETTING_DEFAULT;
// (Product Specification sheet section 5.11)

// Definition of the LED control pins, see schematics.
const uint8_t INWARDS =  10;      
const uint8_t OUTWARDS = 6;       

void setup()
{
    Serial.begin(9600);
    Wire.begin();

    while (!Serial) { // needed to keep leonardo/micro from starting too fast!
        delay(10);
    }

    // .open starts communication with board at given address
    cap.open(MPR121_I2CADDR_ADAFRUIT);
    // Addressing: see Adafruit_MPR121_mod.h and Product Specification sheet section 5.11

    // .init makes soft reset and writes default config values, leaves board in Stop Mode
    while (!cap.init(TOUCH, RELEASE)) {
        Serial.println("MPR121 not found, check wiring?");
        delay(1000);
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
    cap.writeRegister(MPR121_NCLR, 0x10);
    cap.writeRegister(MPR121_FDLR, 0xB0);
    
    cap.writeRegister(MPR121_NCLF, 0x10);
    cap.writeRegister(MPR121_FDLF, 0xB0);
    
    cap.writeRegister(MPR121_NHDT, 0x00); 
    cap.writeRegister(MPR121_NCLT, 0x00);
    cap.writeRegister(MPR121_FDLT, 0x00);
    //(Application Note AN3891, Product Specification sheet section 5.5) 

    //if baseline tracking disabled, baseline values can be hardcoded!
    //cap.writeRegister(MPR121_BASELINE_0 + electrodeNumber, 181);

    //set own touch and release thresholds for 8th ref electrode
    // cap.writeRegister(MPR121_TOUCHTH_0 + 2 * 8, 1);
    // cap.writeRegister(MPR121_RELEASETH_0 + 2 * 8, 0);


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
      filtered[i] = cap.filteredData(i);
      currbaseline[i] = cap.baselineData(i);
      if (currbaseline[i] > (filtered[i] + CANCELLANCE_TH)) {intensity[i] = currbaseline[i] - filtered[i] - CANCELLANCE_TH;}
      else {intensity[i] = 0;}
    }
    else {
      filtered[i] = 0;
      currbaseline[i] = 0;
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
  if //(!(currtouched & _BV(8)) && (currtouched & _BV(0)) && !(lasttouched & _BV(0)) && !(currtouched & _BV(1)) && !(currtouched & _BV(7)) && (tick_count2 >= TICK_OFF)){
    ( (currtouched & _BV(0)) && !(lasttouched & _BV(0)) && !(currtouched & ~_BV(0)) ){ //only the 0th electrode has been touched
    if (manners(TICK_OFF, &ticking, turned_on, &turning_on, &turning_off, &blink_count, BLINKS, &firsttick, PRESSEDp, BUTTONS)) {
      //interrupts();
      }
    else {Serial.println("ERROR: this fella has no manners...");}
  }
  
  if (turned_on && !turning_off) {
    // different light shows for electrodes 1...7

    // 1st electrode: increasing light intensity
    if ( (currtouched & _BV(1)) && !(lasttouched & _BV(1)) && !(currtouched & ~_BV(1)) ) { //only the 1th electrode has been touched 
      if ( INTENSITY_UPPER >= ((uint16_t)(intensitylvl + 2*INTENSITYSTEP + offset)) ) {
        intensitylvl += INTENSITYSTEP; 
        Serial.print("Maximal intensity: "); Serial.println(intensitylvl);
      }
      else {intensitylvl = INTENSITY_UPPER; Serial.print("Intensity at maximum: "); Serial.println(intensitylvl);}
      //turning_on = true;
      //blink_count = 4;
      if (!(*SINEWAVE_IN) && !(*SINEWAVE_OUT)) {
        *SINEWAVE_IN = true;
        *SINEWAVE_OUT = true;
      }    
    }

    // 2nd electrode: decreasing light intensity
    else if ( (currtouched & _BV(2)) && !(lasttouched & _BV(2)) && !(currtouched & ~_BV(2)) ) { //only the 2th electrode has been touched 
      if ( (uint16_t)(INTENSITY_LOWER + 2*INTENSITYSTEP + offset) <= intensitylvl ) {
        intensitylvl -= INTENSITYSTEP;
        Serial.print("Maximal intensity: "); Serial.println(intensitylvl); 
      }
      else {intensitylvl = INTENSITY_LOWER; Serial.print("Intensity at minimum: "); Serial.println(intensitylvl);}
      //turning_on = true;
      //blink_count = 2;
      if (!(*SINEWAVE_IN) && !(*SINEWAVE_OUT)) {
        *SINEWAVE_IN = true;
        *SINEWAVE_OUT = true;
      }
    }

      // 3rd electrode: sinus light outwards
    else if ( (currtouched & _BV(3)) && !(lasttouched & _BV(3)) && !(currtouched & ~_BV(3)) ) { //only the 3th electrode has been touched 
      (*SINEWAVE_OUT) = !(*SINEWAVE_OUT);
       }

      // 4th electrode: sinus light inwards
    else if ( (currtouched & _BV(4)) && !(lasttouched & _BV(4)) && !(currtouched & ~_BV(4)) ) { //only the 4th electrode has been touched 
      (*SINEWAVE_IN) = !(*SINEWAVE_IN);
       }

      // 5th electrode: increasing frequency of sine wave
    else if ( (currtouched & _BV(5)) && !(lasttouched & _BV(5)) && !(currtouched & ~_BV(5)) ) { //only the 5th electrode has been touched 
      if ( ((double)(2*OMMAX)) >= om) {
        om *= 2; 
        Serial.print("Frequency increased: "); Serial.println(om);
      }
      else {Serial.print("Frequency at maximum value: "); Serial.println(om);}

      pulse1_tick = (uint8_t) ( (double)(PULSE1_POS / om) );
      pulse2_tick = (uint8_t) ( (double)(PULSE2_POS / om) );
      pulse3_tick = (uint8_t) ( (double)(PULSE3_POS / om) );

      if (!(*SINEWAVE_IN) && !(*SINEWAVE_OUT)) {
        *SINEWAVE_IN = true;
        *SINEWAVE_OUT = true;
        *INTERMITT = true;
      }
    }

    // 6th electrode: decreasing frequency of for sine wave
    else if ( (currtouched & _BV(6)) && !(lasttouched & _BV(6)) && !(currtouched & ~_BV(6)) ) { //only the 6h electrode has been touched 
      if (om >= ((double)(OMMIN*2))) {
        om /= 2; 
        Serial.print("Frequency decreased: "); Serial.println(om);
      }
      else {Serial.print("Frequency at minimum value: "); Serial.println(om);}

      pulse1_tick = (uint8_t) ( (double)(PULSE1_POS / om) );
      pulse2_tick = (uint8_t) ( (double)(PULSE2_POS / om) );
      pulse3_tick = (uint8_t) ( (double)(PULSE3_POS / om) );

      if (!(*SINEWAVE_IN) && !(*SINEWAVE_OUT)) {
        *SINEWAVE_IN = true;
        *SINEWAVE_OUT = true;
        *INTERMITT = true;
      }
     }
      // 7th electrode: 
    else if (  (currtouched & _BV(7)) && !(lasttouched & _BV(7)) && !(currtouched & ~_BV(7)) ) { //only the 7th electrode has been touched 
      (*INTERMITT) = !(*INTERMITT);
    }
  } 
  
  else if (!turned_on && !turning_on)
  {
    inwardsintensity = 0;
    outwardsintensity = 0;
  }
  else if (turned_on && turning_off && !turning_on) {//&& (ticking == 0)) { //while turning off...
    inwardsintensity = INTENSITY_UPPER;
    outwardsintensity = INTENSITY_UPPER;
  }
  
  else if (turning_on && !turning_off && (ticking == 0) && firsttick) { //while turning on...
//    inwardsintensity = ~(inwardsintensity|0);
//    outwardsintensity = ~(outwardsintensity|0);
    blink(&blink_count, &ticking, &inwardsintensity, &outwardsintensity, PAUSE_TICKS, BLINK_TICKS, intensitylvl);
    firsttick = false;
  }
  
 if (((*SINEWAVE_OUT || *SINEWAVE_IN) || !sineend) && !turning_off) {
    sineend = false;
    sinintensity = (double) ( (intensitylvl - offset) * sin( (float)(om*ticking) ) );
    sinintensity = abs(sinintensity);
    if (*SINEWAVE_OUT || (sineend && (outwardsintensity!=0))) 
    {
      outwardsintensity = (uint8_t) sinintensity;
    }
    if (*SINEWAVE_IN || (sineend && (inwardsintensity!=0))) 
    {
        inwardsintensity = (uint8_t) sinintensity;
    }
    if (*INTERMITT) {
      if (firsttick) {intermitick++; firsttick = false;}
      if ( ((intermitick >= (pulse1_tick-HALFPULSEWIDTH)) && (intermitick <= (pulse1_tick+HALFPULSEWIDTH) )) || 
      ((intermitick >= (pulse2_tick-HALFPULSEWIDTH)) && (intermitick <= (pulse2_tick+HALFPULSEWIDTH) )) || 
      ((intermitick >= (pulse3_tick-HALFPULSEWIDTH)) && (intermitick <= (pulse3_tick+HALFPULSEWIDTH) )) )
      {
        inwardsintensity = 0;
        outwardsintensity = 0;
      }
      else if (intermitick > (pulse3_tick+HALFPULSEWIDTH)) {intermitick = 0;}
    }

    if (!ticking) {ticking = (uint8_t) (TICKSEC<<2); sineend = true;}
  }

  if (inwardsintensity > INTENSITY_UPPER){
    inwardsintensity = INTENSITY_UPPER;
  }
  else if (inwardsintensity < INTENSITY_LOWER){
    inwardsintensity = 0;
  }

  if (outwardsintensity > INTENSITY_UPPER){
    outwardsintensity = INTENSITY_UPPER;
  }
  else if (outwardsintensity < INTENSITY_LOWER){
    outwardsintensity = 0;
  }

  analogWrite(OUTWARDS, outwardsintensity);
  analogWrite(INWARDS, inwardsintensity);

    
/*   if (count == 0) {
    printStatus(&cap);
    Serial.print("Inwards intensity:"); Serial.println(inwardsintensity, DEC);
    Serial.print("Outwards intensity:"); Serial.println(outwardsintensity, DEC);
    for (uint8_t i = 0; i < ELCOUNT; i++) {
      Serial.print("Electrode "); Serial.print(i); Serial.print(" intensity: "); Serial.println(intensity[i], DEC);
    }
    Serial.print("Cancellance: "); Serial.println(cancellance);
    Serial.print("Maximal intensity: "); Serial.println(intensitylvl);
    printStatistic(&cap, ELCOUNT, 6);
    count = 2000;
  }
  count--; */

  // reset our state
  lasttouched = currtouched;
}

ISR(TIMER2_OVF_vect)          // timer overflow interrupt service routine
{
  if (ticking > 0) {
    ticking--; 
    //Serial.print("ticking: "); 
    //Serial.println(ticking);
    firsttick = true;
    }
  
  if ((ticking == 0) && turned_on && turning_off) {
    turned_on = false;
    turning_off = false;
    Serial.println("Turned off."); 
  }
  else if ((ticking == 0) && !turned_on && turning_on && (blink_count == 0)) {
    turned_on = true;
    turning_on = false;
    Serial.println("Turned on."); 
  }
  else if ((ticking == 0) && turned_on && turning_on && (blink_count == 0)) {
    turning_on = false;
    Serial.print("Light intensity changed. New level: "); Serial.println(intensitylvl);
  }
}

void blink (uint8_t * blinkcount, uint8_t * ticks,  uint8_t * in_intensity,  uint8_t * out_intensity, uint8_t pausetime, uint8_t blinktime, uint8_t maxintensity) {
  if ( (*blinkcount) > 0) {
    if ( ( (*blinkcount) % 2 ) == 0 ) {
        (*in_intensity) = maxintensity; 
        (*out_intensity) = maxintensity; 
        (*ticks) = blinktime; 
        (*blinkcount)--;
        Serial.print("Blink count:"); Serial.println(*blinkcount);
        }
      else {
        (*in_intensity) = 0; 
        (*out_intensity) = 0; 
        (*ticks) = pausetime; 
        (*blinkcount)--;
        Serial.print("Blink count:"); Serial.println(*blinkcount);
        }
  }
}

uint8_t manners (uint8_t offtime, uint8_t * ticks, bool turnedon, bool * turnon, bool * turnoff, uint8_t * blinkcount, uint8_t blinkinit, uint8_t * firstflag, bool * const arr, int size) {
  if (!turnedon && !(*turnon) && !(*turnoff) ) { //then start turning on...
    (*turnon) = true;
    (*blinkcount) = (blinkinit<<1);
    (*firstflag) = true;
    Serial.println("Turning on!"); 
  }
  else if (turnedon && !(*turnoff) && !(*turnon) ) { //then start turning off...
    (*ticks) = offtime;
    (*turnoff) = true;
    for (uint8_t i = 0; i < size; i++) {
      arr[i] = false;
    }
    Serial.println("Turning off!"); 
  }
  return (uint8_t) 1;
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
