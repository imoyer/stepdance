/*
Analog In Test

Displays current analog values

A part of the Mixing Metaphors Project

// (c) 2025 Ilan Moyer, Jennifer Jacobs, Devon Frost
*/

#define module_driver   // tells compiler we're using the Stepdance Driver Module PCB
// #define module_basic   // tells compiler we're using the Stepdance Basic Module PCB
                        // This configures pin assignments for the Teensy 4.1

#include "stepdance.hpp"  // Import the stepdance library

AnalogInput analog_a1;
AnalogInput analog_a2;
AnalogInput analog_a3;
AnalogInput analog_a4;

LoopDelay overhead_delay;

void setup() {
  analog_a1.begin(IO_A1);
  analog_a1.set_floor(0);
  analog_a1.set_ceiling(10);

  analog_a2.begin(IO_A2);
  analog_a2.set_floor(0);
  analog_a2.set_ceiling(10);

  analog_a3.begin(IO_A3);
  analog_a3.set_floor(0);
  analog_a3.set_ceiling(10);

  analog_a4.begin(IO_A4);
  analog_a4.set_floor(0);
  analog_a4.set_ceiling(10);

  dance_start();
}


void loop() {
  overhead_delay.periodic_call(&report_overhead, 100);

  dance_loop();
}

void report_overhead(){
  Serial.print("A1: ");
  Serial.println(analog_a1.read());
  Serial.print("A2: ");
  Serial.println(analog_a2.read());
  Serial.print("A3: ");
  Serial.println(analog_a3.read());
  Serial.print("A4: ");
  Serial.println(analog_a4.read());
}