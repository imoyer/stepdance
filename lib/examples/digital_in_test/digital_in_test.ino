/*
Digital In Test

Displays current digital values, on A1->A4, D1->D2

A part of the Mixing Metaphors Project

// (c) 2025 Ilan Moyer, Jennifer Jacobs, Devon Frost
*/

// #define module_driver   // tells compiler we're using the Stepdance Driver Module PCB
#define module_basic   // tells compiler we're using the Stepdance Basic Module PCB
                        // This configures pin assignments for the Teensy 4.1

#include "stepdance.hpp"  // Import the stepdance library

Button digital_a1;
Button digital_a2;
Button digital_a3;
Button digital_a4;
Button digital_d1;
Button digital_d2;

void setup() {
  digital_a1.begin(IO_A1, INPUT_PULLDOWN);
  digital_a2.begin(IO_A2, INPUT_PULLDOWN);
  digital_a3.begin(IO_A3, INPUT_PULLDOWN);
  digital_a4.begin(IO_A4, INPUT_PULLDOWN);
  digital_d1.begin(IO_D1, INPUT_PULLDOWN);
  digital_d2.begin(IO_D2, INPUT_PULLDOWN);
  Serial.begin(115200);

  dance_start();
}


void loop() {
  report_digital_values();
  Serial.println(" ");
  delay(500);
}

void report_digital_values(){
  Serial.print("A1: ");
  Serial.println(digital_a1.read());
  Serial.print("A2: ");
  Serial.println(digital_a2.read());
  Serial.print("A3: ");
  Serial.println(digital_a3.read());
  Serial.print("A4: ");
  Serial.println(digital_a4.read());
  Serial.print("D1: ");
  Serial.println(digital_d1.read());
  Serial.print("D2: ");
  Serial.println(digital_d2.read());
}