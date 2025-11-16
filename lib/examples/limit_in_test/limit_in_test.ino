/*
Digital In Test

Displays current digital values, on LIMIT_A --> LIMIT_D

A part of the Mixing Metaphors Project

// (c) 2025 Ilan Moyer, Jennifer Jacobs, Devon Frost
*/

#define module_driver   // tells compiler we're using the Stepdance Driver Module PCB
                        // This configures pin assignments for the Teensy 4.1

#include "stepdance.hpp"  // Import the stepdance library

Button limit_a;
Button limit_b;
Button limit_c;
Button limit_d;

void setup() {
  limit_a.begin(LIMIT_A);
  limit_b.begin(LIMIT_B);
  limit_c.begin(LIMIT_C);
  limit_d.begin(LIMIT_D);
  Serial.begin(115200);

  dance_start();
}


void loop() {
  report_digital_values();
  Serial.println(" ");
  delay(500);
}

void report_digital_values(){
  Serial.print("LIMIT A: ");
  Serial.println(limit_a.read());
  Serial.print("LIMIT B: ");
  Serial.println(limit_b.read());
  Serial.print("LIMIT C: ");
  Serial.println(limit_c.read());
  Serial.print("LIMIT D: ");
  Serial.println(limit_d.read());
}