/*
Set Motor Currents Utility

This utility helps with setting the drive currents on motors, by reading back the VREF values

!!! MOTOR POWER MUST BE PLUGGED IN FOR ACCURATE READINGS !!!

A part of the Mixing Metaphors Project

// (c) 2025 Ilan Moyer, Jennifer Jacobs, Devon Frost
*/

#define module_driver   // tells compiler we're using the Stepdance Driver Module PCB
                        // This configures pin assignments for the Teensy 4.1

#include "stepdance.hpp"  // Import the stepdance library

#define VREF_GAIN_BIGTREETECH_TMC2209_AMPS_PEAK_PER_VOLT   1    // TMC2209

void set_current_gain(OutputPort* this_output_port); //forward declaration

const float32_t VREF_GAIN_AMPS_PEAK_PER_VOLT = VREF_GAIN_BIGTREETECH_TMC2209_AMPS_PEAK_PER_VOLT;

OutputPort output_a;
OutputPort output_b;
OutputPort output_c;
OutputPort output_d;

void setup() {
  output_a.begin(OUTPUT_A);
  output_b.begin(OUTPUT_B);
  output_c.begin(OUTPUT_C);
  output_d.begin(OUTPUT_D);
  Serial.begin(115200);
  iterate_across_all_output_ports(&set_current_gain);
}


void loop() {
  iterate_across_all_output_ports(&report_drive_current);
  Serial.println(" ");
  delay(500);
}

void report_drive_current(OutputPort* this_output_port){
  Serial.print(this_output_port->port_name);
  Serial.print(": ");
  Serial.print(this_output_port->read_drive_current_amps());
  Serial.println("A (PEAK)");
}

void set_current_gain(OutputPort* this_output_port){
  this_output_port->set_drive_current_gain(VREF_GAIN_AMPS_PEAK_PER_VOLT);
}