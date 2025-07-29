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
  kilohertz_timer.begin(Plugin::run_kilohertz_plugins, KILOHERTZ_PLUGIN_PERIOD_US);
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

uint8_t Plugin::num_registered_loop_plugins = 0;
Plugin* Plugin::registered_loop_plugins[MAX_NUM_LOOP_PLUGINS];

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
    
    case PLUGIN_LOOP:
      if(num_registered_loop_plugins < MAX_NUM_LOOP_PLUGINS){
        registered_loop_plugins[num_registered_loop_plugins] = this;
        num_registered_loop_plugins ++;
      }
      break;
  }
}

void Plugin::run(){};

void Plugin::loop(){};

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

void Plugin::run_loop_plugins(){
  for(uint8_t plugin_index = 0; plugin_index < num_registered_loop_plugins; plugin_index++){
    registered_loop_plugins[plugin_index]->loop();
  }  
}

// -- BLOCKPORT --
BlockPort::BlockPort(){};

void BlockPort::write(float64_t value, uint8_t mode){
  // externally updates the incremental or absolute buffers
  if(mode == INCREMENTAL){
    if(incremental_buffer_is_read){ //if this buffer has been read internally, then we assume it's being set by a new frame, so we clear it first.
      incremental_buffer = (value * block_to_world_ratio);
      incremental_buffer_is_read = false;
    }else{
      incremental_buffer += (value * block_to_world_ratio);
    }
  }else{
    absolute_buffer = (value * block_to_world_ratio);
  }
}

float64_t BlockPort::read(uint8_t mode){
  // Externally reads the incremental or absolute buffers
  if(mode == INCREMENTAL){
    return (incremental_buffer / block_to_world_ratio);
  }else{
    return (absolute_buffer / block_to_world_ratio);
  }
}

void BlockPort::set_absolute(float64_t value){
  // Internally updates the value of the absolute_buffer
  absolute_buffer = value;
}

float64_t BlockPort::get(uint8_t mode){
  // Internally reads the incremental or absolute buffers
  if(mode == INCREMENTAL){
    incremental_buffer_is_read = true; //flags that the buffer has been internally read.
    return incremental_buffer;
  }else{
    return absolute_buffer;
  }
}

void BlockPort::set_ratio(float block_units, float world_units){
  block_to_world_ratio = static_cast<float64_t>(block_units / world_units);
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
  if(get_function == nullptr){ //no get override function
    if(target != nullptr){
      return *target / transfer_ratio;
    }else{
      return 0.0;
    }
  }else{
    return get_function();
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
  Plugin::run_loop_plugins(); //run all plugins that execute in the main loop
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