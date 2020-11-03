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

#include <Wire.h>
#include "Adafruit_MPR121_mod.h"

// tool for easy bit choose e.g. for electrode 
#ifndef _BV
#define _BV(bit) (1 << (bit)) 
#endif

// timing of status print
uint16_t count = 0;

// You can have up to 4 on one i2c bus.
Adafruit_MPR121 cap = Adafruit_MPR121();

// Keeps track of the last pins touched
// so we know when buttons are 'released'
uint16_t lasttouched = 0;
uint16_t currtouched = 0;

// Initialize intensity values of the inwards and outwards
// facing LED series for the square-round DEMO board.
int inwardsintensity = 0;
int outwardsintensity = 0;

// initialize arrays for basline and filtered output values of electrodes
uint16_t firstbaseline[9] = {0,0,0,0,0,0,0,0,0};
uint16_t currbaseline[9] = {0,0,0,0,0,0,0,0,0};
uint16_t filtered[9] = {0,0,0,0,0,0,0,0,0};
int intensity[9] = {0,0,0,0,0,0,0,0,0};

// Change touch and release from default if needed in range 0-255
// Touch condition: Baseline - filtered output > touch threshold
// Release condition: Baseline - filtered output < release threshold
uint8_t touch = 3;
uint8_t release = 2;
// uint8_t touch = MPR121_TOUCH_THRESHOLD_DEFAULT;
// uint8_t release = MPR121_RELEASE_THRESHOLD_DEFAULT;
// (Product Specification sheet section 5.6)

// Number of electrodes to be used, max. 12: 0,1,2....11
uint8_t elcount = 9; 

// Touch and release debounce values: how many times
// MPR121 rejects the change of touch and release status.
// Increases resistance to noise, slows detection. Range 0-7.
uint8_t debounce_touch = 2;
uint8_t debounce_release = 1;
// (Product Specification sheet section 5.7)

// definition of ON/OFF state
bool TURNED_ON = false;

// Active electrode configuration: see Adafruit_MPR121_mod.h for details
uint8_t ecr = MPR121_BL_TRACKING_ALL + MPR121_ELEPROX_EN_OFF + elcount; 
// uint8_t ecr =  MPR121_ECR_SETTING_DEFAULT;
// (Product Specification sheet section 5.11)

// Definition of the LED control pins, see schematics.
uint8_t Inwards =  10;      //Pin 10: Timer 2
uint8_t Outwards = 6;       //Pin 6: Timer 4

void setup()
{

    

  // Timer 1
  noInterrupts();           // Alle Interrupts temporär abschalten
  TCCR1A = 0;
  TCCR1B = 0;
  TCNT1 = 0;                // Register mit 0 initialisieren
  OCR1A = 31250;            // Output Compare Register vorbelegen 
  TCCR1B |= (1 << CS12);    // 256 als Prescale-Wert spezifizieren
  TIMSK1 |= (1 << OCIE1A);  // Timer Compare Interrupt aktivieren
  interrupts();             // alle Interrupts scharf schalten

  
    Serial.begin(9600);
    Wire.begin();

    while (!Serial) { // needed to keep leonardo/micro from starting too fast!
        delay(10);
    }

    // .open starts communication with board at given address
    cap.open(MPR121_I2CADDR_ADAFRUIT);
    // Addressing: see Adafruit_MPR121_mod.h and Product Specification sheet section 5.11

    // .init makes soft reset and writes default config values, leaves board in Stop Mode
    if (!cap.init(touch, release)) {
        Serial.println("MPR121 not found, check wiring?");
        while(!cap.init(touch, release)){
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
    cap.writeRegister(MPR121_NCLR, 0x10);
    cap.writeRegister(MPR121_FDLR, 0xB0);
    
    cap.writeRegister(MPR121_NCLF, 0x10);
    cap.writeRegister(MPR121_FDLF, 0x10);
    
    cap.writeRegister(MPR121_NHDT, 0x00); 
    cap.writeRegister(MPR121_NCLT, 0x00);
    cap.writeRegister(MPR121_FDLT, 0x00);
    //(Application Note AN3891, Product Specification sheet section 5.5) 

    //if baseline tracking disabled, baseline values can be hardcoded!
    //cap.writeRegister(MPR121_BASELINE_0 + electrodeNumber, 181);

    //applying debounce settings
    cap.writeRegister(MPR121_DEBOUNCE, debounce_release<<4 + debounce_touch);
  
    // .begin writes ecr settings to put board in Run Mode
    cap.begin(ecr);

    delay(1000); // to make sure auto-config runs through

    // Print configuration registers
    Serial.println("CONFIG REGISTERS");
    for (uint8_t i = 0x2B; i < 0x80; i++) {
      Serial.print("$"); Serial.print(i, HEX);
      Serial.print(": 0x"); Serial.println(cap.readRegister8(i), HEX);
    }

    Serial.println();

    pinMode(Inwards, OUTPUT);
    digitalWrite(Inwards, LOW);
    pinMode(Outwards, OUTPUT);
    digitalWrite(Outwards, LOW);

    for (uint8_t i = 0; i < elcount; i++) {
      firstbaseline[i] = cap.baselineData(i);
    }

}

ISR(TIMER1_COMPA_vect)        
{
  TCNT1 = 0;                // Register mit 0 initialisieren   
  TURNED_ON = !TURNED_ON;
  digitalWrite(Inwards, TURNED_ON); // LED ein und aus  
}

void loop() {
//   Get the currently touched pads
  currtouched = cap.touched();
  
  for (uint8_t i=0; i<elcount; i++) {
    // it if *is* touched and *wasnt* touched before, alert!
    if ((currtouched & _BV(i)) && !(lasttouched & _BV(i)) ) {
      Serial.print(i); Serial.print(" touched! Reading:"); Serial.println(cap.filteredData(i));
      // printStatus(&cap);
    }
    // if it *was* touched and now *isnt*, alert!
    if (!(currtouched & _BV(i)) && (lasttouched & _BV(i)) ) {
      Serial.print(i); Serial.print(" released! Reading:");Serial.println(cap.filteredData(i));
      filtered[i] = 0;
      currbaseline[i] = 0;
      intensity[i] = 0;
      // printStatus(&cap);
    }

    if (currtouched & _BV(i)) {
      filtered[i] = cap.filteredData(i);
      currbaseline[i] = cap.baselineData(i);
      intensity[i] = currbaseline[i] - filtered[i];
    }
    else {
      filtered[i] = 0;
      currbaseline[i] = 0;
      intensity[i] = 0;
    }
  }

  // ON/OFF as the 0th electrode is touched
  if ((currtouched & _BV(0)) && !(lasttouched & _BV(0)) && !(currtouched & _BV(1)) && !(currtouched & _BV(7))){
    TURNED_ON = !TURNED_ON;
  }

  // ON/OFF 

  inwardsintensity = 0;
  outwardsintensity = 0;

  
  if (TURNED_ON) {

  }

  
  analogWrite(Outwards, outwardsintensity);

  if (count == 0) {
    printStatus(&cap);
    Serial.print("Inwards intensity:"); Serial.println(inwardsintensity, DEC);
    Serial.print("Outwards intensity:"); Serial.println(outwardsintensity, DEC);
    for (uint8_t i = 0; i < elcount; i++) {
      Serial.print("Electrode "); Serial.print(i); Serial.print(" intensity: "); Serial.println(intensity[i], DEC);
    }
//    printStatistic(&cap, ecr, 6);
    count = 2000;
  }
  count--;
  
  // reset our state
  lasttouched = currtouched;
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
