/*
Stepdance Driver Module Template

Template for writing programs for the Stepdance System

A part of the Mixing Metaphors Project

// (c) 2026 Ilan Moyer, Jennifer Jacobs, Emilie Yu, Alejandro Aponte
*/

#define module_driver   // tells compiler we're using the Stepdance Driver Module PCB
                        // This configures pin assignments for the Teensy 4.1

#include "stepdance.hpp"  // Import the stepdance library

// -- Define Output Ports --


// -- Define Motion Channels --

// -- Define Kinematics --

// -- Define Input Devices--

// -- Define Generators --

void setup() {
  // -- Configure and start the output ports --
 


  // Enable the output drivers
  enable_drivers();

  // -- Configure and start the channels --
  

  // -- Configure and start the Input Devices --
  

  // -- Configure and start the kinematics--
 

  // -- Configure and start generators--
  

  // -- Start the stepdance library --
  // This activates the system.
  dance_start();
}

LoopDelay overhead_delay;

void loop() {
  // uncomment to report overhead
  // overhead_delay.periodic_call(&report_overhead, 500); 

  dance_loop(); // Stepdance loop provides convenience functions, and should be called at the end of the main loop
}



void report_overhead(){
  Serial.println(stepdance_get_cpu_usage(), 4);
}