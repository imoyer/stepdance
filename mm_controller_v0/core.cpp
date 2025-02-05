#include "core_pins.h"
#include <functional>
#include <sys/_stdint.h>
#include "core.hpp"
#include "IntervalTimer.h"
/*
Core Module of the StepDance Control System

This contains core system functions such as the frame interrupt timer.

A part of the Mixing Metaphors Project
(c) 2025 Ilan Moyer, Jennifer Jacobs, Devon Frost
*/

// -- FRAME INTERRUPT --
IntervalTimer core_frame_timer;

frame_function_pointer frame_functions[MAX_NUM_FRAME_FUNCTIONS];
uint8_t num_registered_frame_functions = 0;

void add_function_to_frame(frame_function_pointer target_function){
  // Adds a function to be executed on the frame
  if(num_registered_frame_functions < MAX_NUM_FRAME_FUNCTIONS){
    frame_functions[num_registered_frame_functions] = target_function;
    num_registered_frame_functions ++;
  } // NOTE: should add a return value if it works
}

void on_frame(){
  for(uint8_t function_index = 0; function_index<num_registered_frame_functions; function_index++){
    frame_functions[function_index]();
  }
}

// -- OVERALL SYSTEM --

void stepdance_start(){
  // Starts all core timers etc
  core_frame_timer.priority(128);
  core_frame_timer.begin(on_frame, CORE_FRAME_PERIOD_US);
}


