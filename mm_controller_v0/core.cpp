#include <cstddef>
#include "arm_math.h"
#include "imxrt.h"
#include "core_pins.h"
#include <functional>
#include <sys/_stdint.h>
#include "core.hpp"
#include "IntervalTimer.h"
#include "channels.hpp"

/*
Core Module of the StepDance Control System

This contains core system functions such as the frame interrupt timer.

A part of the Mixing Metaphors Project
(c) 2025 Ilan Moyer, Jennifer Jacobs, Devon Frost
*/

// -- FRAME INTERRUPT --
IntervalTimer core_frame_timer;
IntervalTimer kilohertz_timer;

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

void dance_start(){
  // activate all pre-channel frame plugins
  add_function_to_frame(Plugin::run_pre_channel_frame_plugins);
  // activate channels
  activate_channels();
  // activate all post-channel frame plugins
  add_function_to_frame(Plugin::run_post_channel_frame_plugins);
  
  // Start core frame timer
  core_frame_timer.priority(128);
  core_frame_timer.begin(on_frame, CORE_FRAME_PERIOD_US);

  // Start kilohertz plugin timer
  kilohertz_timer.priority(130);
  // kilohertz_timer.begin(Plugin::run_kilohertz_plugins, KILOHERTZ_PLUGIN_PERIOD_US);
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
Plugin::Plugin(){};

uint8_t Plugin::num_registered_pre_channel_frame_plugins = 0;
Plugin* Plugin::registered_pre_channel_frame_plugins[MAX_NUM_PRE_CHANNEL_FRAME_PLUGINS];

uint8_t Plugin::num_registered_post_channel_frame_plugins = 0;
Plugin* Plugin::registered_post_channel_frame_plugins[MAX_NUM_POST_CHANNEL_FRAME_PLUGINS];

uint8_t Plugin::num_registered_kilohertz_plugins = 0;
Plugin* Plugin::registered_kilohertz_plugins[MAX_NUM_KILOHERTZ_PLUGINS];

void Plugin::register_plugin(){ //default to pre-channel frame plugin
  register_plugin(PLUGIN_FRAME_PRE_CHANNEL);
}

void Plugin::register_plugin(uint8_t execution_target){
  switch(execution_target){
    case PLUGIN_FRAME_PRE_CHANNEL:
      if(num_registered_pre_channel_frame_plugins < MAX_NUM_PRE_CHANNEL_FRAME_PLUGINS){
        registered_pre_channel_frame_plugins[num_registered_pre_channel_frame_plugins] = this;
        num_registered_pre_channel_frame_plugins ++;
      }
      break;

    case PLUGIN_FRAME_POST_CHANNEL:
      if(num_registered_post_channel_frame_plugins < MAX_NUM_POST_CHANNEL_FRAME_PLUGINS){
        registered_post_channel_frame_plugins[num_registered_post_channel_frame_plugins] = this;
        num_registered_post_channel_frame_plugins ++;
      }
      break;

    case PLUGIN_KILOHERTZ:
      if(num_registered_kilohertz_plugins < MAX_NUM_KILOHERTZ_PLUGINS){
        registered_kilohertz_plugins[num_registered_kilohertz_plugins] = this;
        num_registered_kilohertz_plugins ++;
      }
      break;   
  }
}

void Plugin::run(){};

void Plugin::run_pre_channel_frame_plugins(){
  for(uint8_t plugin_index = 0; plugin_index < num_registered_pre_channel_frame_plugins; plugin_index++){
    registered_pre_channel_frame_plugins[plugin_index]->run();
  }
}

void Plugin::run_post_channel_frame_plugins(){
  for(uint8_t plugin_index = 0; plugin_index < num_registered_post_channel_frame_plugins; plugin_index++){
    registered_post_channel_frame_plugins[plugin_index]->run();
  }
}

void Plugin::run_kilohertz_plugins(){
  for(uint8_t plugin_index = 0; plugin_index < num_registered_kilohertz_plugins; plugin_index++){
    registered_kilohertz_plugins[plugin_index]->run();
  }
}

// -- TRANSMISSION --
Transmission::Transmission(){};

void Transmission::begin(){
  begin(1, 1, nullptr);
}

void Transmission::begin(DecimalPosition *output_position){
  begin(1, 1, output_position);
}

void Transmission::begin(float input_units, float output_units){
  begin(input_units, output_units, nullptr);
}

void Transmission::begin(float input_units, float output_units, DecimalPosition *output_position){
  set_ratio(input_units, output_units);
  target = output_position;
}

void Transmission::set_ratio(float input_units, float output_units){
  transfer_ratio = static_cast<float64_t>(output_units / input_units);
}

void Transmission::set(float64_t input_value){
  if(target != nullptr){
    *target = input_value * transfer_ratio;
  }
}

float64_t Transmission::get(){
  if(target != nullptr){
    return *target / transfer_ratio;
  }else{
    return 0.0;
  }
}

void Transmission::increment(float64_t input_value){
  if(target != nullptr){
    *target += (input_value * transfer_ratio);
  }
}

float64_t Transmission::convert(float64_t input_value){
  return input_value * transfer_ratio;
}
float64_t Transmission::convert_reverse(float64_t output_value){
  return output_value / transfer_ratio;
}

// -- STEPDANCE LOOP FUNCTIONS AND CLASSES --

void dance_loop(){
  stepdance_loop_time_ms = 1000*(float)(ARM_DWT_CYCCNT - stepdance_loop_entry_cycle_count) / (float)(F_CPU);
  stepdance_loop_entry_cycle_count = ARM_DWT_CYCCNT;
}

LoopDelay::LoopDelay(){};

void LoopDelay::periodic_call(void (*callback_function)(), float interval_ms){
  //if the elapsed time since the last call exceeds interval_ms, then the function will be called.
  time_since_last_call_ms += stepdance_loop_time_ms;
  if(time_since_last_call_ms >= interval_ms){
    callback_function();
    time_since_last_call_ms = 0;
  }
}