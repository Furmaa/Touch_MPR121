#include "Arduino.h"

// Definition of the LED control pins, see schematics.
const uint8_t Drive1 =  10;      
const uint8_t Drive2 = 6;       
const uint8_t Drive3 = 3;    


void setup() {
  // put your setup code here, to run once:
  pinMode(Drive1, OUTPUT);
  digitalWrite(Drive1, LOW);
  pinMode(Drive2, OUTPUT);
  digitalWrite(Drive2, LOW);
  pinMode(Drive3, OUTPUT);
  digitalWrite(Drive3, LOW);
}

void loop() {
  // put your main code here, to run repeatedly:
  digitalWrite(Drive1, HIGH);
  digitalWrite(Drive2, HIGH);
  digitalWrite(Drive3, HIGH);
}
