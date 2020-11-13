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

// You can have up to 4 on one i2c bus
Adafruit_MPR121 cap = Adafruit_MPR121();

// Keeps track of the last pins touched
// so we know when buttons are 'released'
uint16_t lasttouched = 0;
uint16_t currtouched = 0;

uint16_t count = 0;

int intensity = 0;
int proximity = 0;
int touchval = 0;
int ref1val = 0;
int ref2val = 0;
int diff = 0;
double baseline_print = 0;
double filtered_print = 0;
uint16_t curr_touch_baseline = 0;
uint16_t currfiltered = 0;
uint16_t first_touch_baseline  = 0;
uint16_t curr_ref1_baseline = 0;
uint16_t first_ref1_baseline = 0;
uint16_t curr_ref1_filtered = 0;
uint16_t curr_ref2_baseline = 0;
uint16_t first_ref2_baseline = 0;
uint16_t curr_ref2_filtered = 0;

// define touch and release in range 0-255:
uint8_t touch = 3; 
uint8_t release = 2; 
// Touch condition: Baseline - filtered output > touch threshold
// Release condition: Baseline - filtered output < release threshold
// uint8_t touch = MPR121_TOUCH_THRESHOLD_DEFAULT;
// uint8_t release = MPR121_RELEASE_THRESHOLD_DEFAULT;

uint8_t elcount = 3; // number of electrodes to be used, max. 12
uint8_t eltouch = 0; // PIN number to which main (touch) electrode is connected
uint8_t elref1 = 1;  // PIN number to which reference (active shielding) electrode is connected
uint8_t elref2 = 2;

// define debounce: higher number may improve noise resistance. Range 0-15
uint8_t debounce_touch = 5; // number of consecutively sensed touches to be discarded.
uint8_t debounce_release = 2; // number of consecutively sensed releases to be discarded.

uint8_t ecr = MPR121_BL_TRACKING_5MSB + MPR121_ELEPROX_EN_OFF + elcount; 
// MPR121_BL_TRACKING_OFF disables baseline-tracking
// MPR121_BL_TRACKING_5MSB enables baseline-tracking with baseline value initalized: 5 MSB loaded from touch to baseline 
// MPR121_BL_TRACKING_ALL enables baseline-tracking with baseline value initalized: all loaded from touch to baseline 
// uint8_t ecr =  MPR121_ECR_SETTING_DEFAULT; // set to Adafruit default

// define LED control pins on Arduino
uint8_t ArrayPin =  3;      
uint8_t RingPin = 6;
uint8_t RowPin = 10;

void setup()
{
    pinMode(ArrayPin, OUTPUT);
    pinMode(RowPin, OUTPUT);
    pinMode(RingPin, OUTPUT);

    digitalWrite(ArrayPin, HIGH);
    digitalWrite(RingPin, HIGH);
    digitalWrite(RowPin, HIGH);

    Serial.begin(9600);
    Wire.begin();

    while (!Serial) { // needed to keep leonardo/micro from starting too fast!
        delay(10);
    }

    // Adafruit board:
    // Default address is 0x5A, if tied to 3.3V its 0x5B
    // If tied to SDA its 0x5C and if SCL then 0x5D 
    // robotshop board:
    // default 0x5B! ADDR pin connected to VDD
    cap.open(MPR121_I2CADDR_ADAFRUIT);

    // .init makes soft reset and writes default config values, leaves board in Stop Mode
    if (!cap.init(touch, release)) {
        Serial.println("MPR121 not found, check wiring?");
        while(!cap.init(touch, release)){
          delay(1000);
          }
    }
    Serial.println("MPR121 found!"); 

    // comment AUTOCONFIG and alter values to adapt e.g. to Vdd != 3.3V
    #ifndef AUTOCONFIG
        cap.writeRegister(MPR121_AUTOCONFIG0, 0x0B);

        // baseline search values 
        cap.writeRegister(MPR121_UPLIMIT, 250);     // ((Vdd - 0.7)/Vdd) * 256 for Vdd = 3.3V : 200
        cap.writeRegister(MPR121_TARGETLIMIT, 180); // UPLIMIT * 0.9 for Vdd = 3.3V : 180
        cap.writeRegister(MPR121_LOWLIMIT, 130);    // UPLIMIT * 0.65 for Vdd = 3.3V : 130
        // baseline should be then between UPLIMIT and LOWLIMIT!
    #endif

    //for sensitive Demo Touch Panel sampling and Baseline tracking is too fast: 
    //baseline value gets adjusted if touched too slow, hence no touching is sensed!
    //try to set ESI value from Adafruit default 000 (1 ms, maximum scanning freq.) to slower... 
    //with encoding 100 (16ms, datasheet default) it works pretty well: recognizes swift and slow touches alike
    //cap.writeRegister(MPR121_CONFIG2, 0x33);
    //or it might be better to play around with the baseline filter values: 
    // with that the baseline tracking characteristics can be optimized.
    //Overwrite example:
    cap.writeRegister(MPR121_MHDR, 0x02);
    cap.writeRegister(MPR121_NHDR, 0x01);
    cap.writeRegister(MPR121_NCLR, 0x05);
    cap.writeRegister(MPR121_FDLR, 0xC0);
    
    cap.writeRegister(MPR121_MHDF, 0x02);
    cap.writeRegister(MPR121_NHDF, 0x01);
    cap.writeRegister(MPR121_NCLF, 0x05);
    cap.writeRegister(MPR121_FDLF, 0xC0);
    
    cap.writeRegister(MPR121_NHDT, 0x00); 
    cap.writeRegister(MPR121_NCLT, 0x00);
    cap.writeRegister(MPR121_FDLT, 0x00);
    //(Application Note AN3891, Product Specification sheet section 5.5) 


    //if baseline tracking disabled, baseline values must be hardcoded!
    //cap.writeRegister(MPR121_BASELINE_0 + eltouch, 181);

    //writing debounce values to 
    cap.writeRegister(MPR121_DEBOUNCE, debounce_release<<4 + debounce_touch);
  
    // .begin writes ecr settings to put board in Run Mode
    cap.begin(ecr);

    Serial.println();

    delay(1000); // to make sure auto-config runs through

    // Print configuration registers
    Serial.println("CONFIG REGISTERS");
    for (uint8_t i = 0x2B; i < 0x80; i++) {
      Serial.print("$"); Serial.print(i, HEX);
      Serial.print(": 0x"); Serial.println(cap.readRegister8(i), HEX);
    }

    Serial.println();

    digitalWrite(ArrayPin, LOW);
    digitalWrite(RingPin, LOW);
    digitalWrite(RowPin, LOW);

    first_touch_baseline = cap.baselineData(eltouch);
    first_ref1_baseline = cap.baselineData(elref1);
    first_ref2_baseline = cap.baselineData(elref2);

    // setup timer1 -- interferes with pwm pins 9, 10!

    noInterrupts(); 
    TCCR1A = 0;
    TCCR1B = 0;
    TCNT1  = 0;
    TIMSK1 = 0;
    OCR1A = 32500;
    TCCR1B |= (1 << WGM12);   // CTC mode
    TCCR1B |= (1 << CS12); // 256 prescaler 
    TIMSK1 |= (1 << OCIE1A);  // enable timer compare interrupt
    interrupts();

}

void loop() {
//   print electrode values

for (uint8_t i = 0; i < elcount; i++) {
  diff = cap.baselineData(i) - cap.filteredData(i);
//  Serial.print(cap.filteredData(i));
//  Serial.print("\t");
  Serial.print(cap.baselineData(i));
  Serial.print("\t");
//  Serial.print(diff);
//  Serial.print("\t");

}
Serial.println();
  
}

void printStatistic (Adafruit_MPR121 * obj, uint8_t ecr, uint8_t count) {

    if (count > 6) {count = 6;}

    uint16_t sum;
    uint16_t avg;
    uint8_t stddev;
    int8_t diff;  
    uint16_t varsum;
    uint16_t datapoints = 1 << count;
    uint8_t elcount = ecr << 4;
    elcount = elcount >> 4;
    uint16_t filtered [elcount][datapoints];
    Serial.print("Number of active electrodes: "); Serial.println(elcount);
    Serial.print("Number of datapoints: "); Serial.println(datapoints);

    for (uint8_t i = 0; i < elcount; i++) {
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

ISR(TIMER1_COMPA_vect)          // timer compare interrupt service routine

{
    digitalWrite(ArrayPin, digitalRead(ArrayPin) ^ 1);// toggle LED pin

}
