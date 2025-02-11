#include "imxrt.h"
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
  stepdance_interrupt_entry_cycle_count = ARM_DWT_CYCCNT;
  for(uint8_t function_index = 0; function_index<num_registered_frame_functions; function_index++){
    frame_functions[function_index]();
  }
  // metrics
  uint32_t interrupt_duration_cycles = ARM_DWT_CYCCNT - stepdance_interrupt_entry_cycle_count;
  float cpu_fraction = (float)(interrupt_duration_cycles * CORE_FRAME_FREQ_HZ) / (float)(F_CPU);
  if(cpu_fraction > stepdance_max_cpu_usage){
    stepdance_max_cpu_usage = cpu_fraction;
  }
}

// -- OVERALL SYSTEM --

void stepdance_start(){
  // Starts all core timers etc
  core_frame_timer.priority(128);
  core_frame_timer.begin(on_frame, CORE_FRAME_PERIOD_US);
}

// -- METRICS --
float stepdance_get_cpu_usage(){
  // Returns the CPU usage as a fraction 0-1
  return stepdance_max_cpu_usage;
}

void stepdance_metrics_reset(){
  stepdance_max_cpu_usage = 0;
}

// -- PLUGINS --
uint8_t Plugin::num_registered_plugins = MAX_NUM_PLUGINS;

void Plugin::register_plugin(){
  if(num_registered_plugins < MAX_NUM_PLUGINS){
    registered_plugins[num_registered_plugins] = this;
    num_registered_plugins ++;
  } // NOTE: should add a return value if it works
}

void Plugin::run_plugins(){
  for(uint8_t plugin_index = 0; plugin_index < num_registered_plugins; plugin_index++){
    registered_plugins[plugin_index]->run();
  }
}
