/*
Configuration Module of the StepDance Control System

This module helps with configuring for various possible boards

[More Details to be Added]

A part of the Mixing Metaphors Project
(c) 2025 Ilan Moyer, Jennifer Jacobs, Devon Frost

*/


#ifndef module_configurations_h //prevent importing twice
#define module_configurations_h


// -- BOARD DEFINITIONS --

#define TEENSY_PIN_NULL   255   //used to indicate that a pin is not defined for a particular module

// Teensy 4.1 pin numbers

#ifdef module_driver
  #define IO_A1     24
  #define IO_A2     26
  #define IO_A3     27
  #define IO_A4     25
  #define IO_D1     28
  #define IO_D2     12
  #define OUTPUT_A_VREF     15
  #define OUTPUT_B_VREF     41
  #define OUTPUT_C_VREF     40
  #define OUTPUT_D_VREF     38
  #define OUTPUT_A_ENABLE   14
  #define OUTPUT_B_ENABLE   13
  #define OUTPUT_C_ENABLE   39
  #define OUTPUT_D_ENABLE   33
  #define OUTPUT_A_LIMIT    29
  #define OUTPUT_B_LIMIT    30
  #define OUTPUT_C_LIMIT    31
  #define OUTPUT_D_LIMIT    32
#endif

#ifdef module_legacy
  #define IO_A  16
  #define IO_B  15
  #define IO_C  14
  #define IO_D  41
  #define IO_E  40
#endif

#ifdef module_basic
  #define IO_A1     14
  #define IO_A2     15
  #define IO_A3     20
  #define IO_A4     21
  #define IO_D1     13
  #define IO_D2     12
  #define OUTPUT_A_VREF     TEENSY_PIN_NULL
  #define OUTPUT_B_VREF     TEENSY_PIN_NULL
  #define OUTPUT_C_VREF     TEENSY_PIN_NULL
  #define OUTPUT_D_VREF     TEENSY_PIN_NULL
  #define OUTPUT_A_ENABLE   TEENSY_PIN_NULL
  #define OUTPUT_B_ENABLE   TEENSY_PIN_NULL
  #define OUTPUT_C_ENABLE   TEENSY_PIN_NULL
  #define OUTPUT_D_ENABLE   TEENSY_PIN_NULL
  #define OUTPUT_A_LIMIT    TEENSY_PIN_NULL
  #define OUTPUT_B_LIMIT    TEENSY_PIN_NULL
  #define OUTPUT_C_LIMIT    TEENSY_PIN_NULL
  #define OUTPUT_D_LIMIT    TEENSY_PIN_NULL
#endif

// // CONFIGURATION FUNCTIONS
// void configure_module(uint8_t module){
//   // configures stepdance for a specific module.
//   // This function will be called automatically when configuration.hpp is included in the main program, based on the defines.
// }

#endif