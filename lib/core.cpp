#include "wiring.h"
#include <cstddef>
#include "arm_math.h"
#include "imxrt.h"
#include "core_pins.h"
#include <functional>
#include <sys/_stdint.h>
#include "core.hpp"
#include "IntervalTimer.h"
#include "channels.hpp"
#include "rpc.hpp"

/*
Core Module of the StepDance Control System

This contains core system functions such as the frame interrupt timer.

A part of the Mixing Metaphors Project
(c) 2025 Ilan Moyer, Jennifer Jacobs, Devon Frost, Emilie Yu
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
  // activate input port plugins
  add_function_to_frame(Plugin::run_input_port_frame_plugins);
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
uint8_t Plugin::num_registered_input_port_frame_plugins = 0;
Plugin* Plugin::registered_input_port_frame_plugins[MAX_NUM_INPUT_PORT_FRAME_PLUGINS];

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
    case PLUGIN_INPUT_PORT:
      if(num_registered_input_port_frame_plugins < MAX_NUM_INPUT_PORT_FRAME_PLUGINS){
        registered_input_port_frame_plugins[num_registered_input_port_frame_plugins] = this;
        num_registered_input_port_frame_plugins ++;
      }
      else {
        Serial.println("WARNING: failed to register a plugin (nb of plugins registered > max number).");
      }
      break;

    case PLUGIN_FRAME_PRE_CHANNEL:
      if(num_registered_pre_channel_frame_plugins < MAX_NUM_PRE_CHANNEL_FRAME_PLUGINS){
        registered_pre_channel_frame_plugins[num_registered_pre_channel_frame_plugins] = this;
        num_registered_pre_channel_frame_plugins ++;
      }
      else {
        Serial.println("WARNING: failed to register a plugin (nb of plugins registered > max number).");
      }
      break;

    case PLUGIN_FRAME_POST_CHANNEL:
      if(num_registered_post_channel_frame_plugins < MAX_NUM_POST_CHANNEL_FRAME_PLUGINS){
        registered_post_channel_frame_plugins[num_registered_post_channel_frame_plugins] = this;
        num_registered_post_channel_frame_plugins ++;
      }
      else {
        Serial.println("WARNING: failed to register a plugin (nb of plugins registered > max number).");
      }
      break;

    case PLUGIN_KILOHERTZ:
      if(num_registered_kilohertz_plugins < MAX_NUM_KILOHERTZ_PLUGINS){
        registered_kilohertz_plugins[num_registered_kilohertz_plugins] = this;
        num_registered_kilohertz_plugins ++;
      }
      else {
        Serial.println("WARNING: failed to register a plugin (nb of plugins registered > max number).");
      }
      break;
    
    case PLUGIN_LOOP:
      if(num_registered_loop_plugins < MAX_NUM_LOOP_PLUGINS){
        registered_loop_plugins[num_registered_loop_plugins] = this;
        num_registered_loop_plugins ++;
      }
      else {
        Serial.println("WARNING: failed to register a plugin (nb of plugins registered > max number).");
      }
      break;
  }
}

void Plugin::disable(){};

void Plugin::enable(){};

void Plugin::run(){};

void Plugin::loop(){};

void Plugin::enroll(RPC *rpc, const String& instance_name){};

void Plugin::run_input_port_frame_plugins(){
  for(uint8_t plugin_index = 0; plugin_index < num_registered_input_port_frame_plugins; plugin_index++){
    registered_input_port_frame_plugins[plugin_index]->run();
  }
}

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

void Plugin::push_deep(){};
void Plugin::pull_deep(){};



// -- BLOCKPORT --
BlockPort::BlockPort(){};

// - User Functions -
// These are intended to be called from user code
void BlockPort::set_ratio(float world_units, float block_units){
  world_to_block_ratio = static_cast<float64_t>(world_units / block_units);
}

void BlockPort::map(BlockPort *map_target, uint8_t mode){
  target_BlockPort = map_target;
  this->mode = mode;
}

// - Library Functions -
// These are intended to be called from other library components, e.g. other blocks interfacing with this Blockport

void BlockPort::write(float64_t value, uint8_t mode){
  if(update_has_run){
    update_has_run = false;
    incremental_buffer = 0;
  }
  if(mode == INCREMENTAL){
    incremental_buffer += convert_world_to_block_units(value);
  }else{ //ABSOLUTE
    absolute_buffer = convert_world_to_block_units(value);
  }
}

float64_t BlockPort::read(uint8_t mode){
  if(update_has_run){
    // we're post-update, so we can just return the buffers directly, because they reflect what's actually happened.
    if(mode == INCREMENTAL){
      return convert_block_to_world_units(incremental_buffer);
    }else{
      return convert_block_to_world_units(absolute_buffer);
    }
  }else{
    // pre-update, so we provide an estimate of what the target state will be post-update.
    if(mode == INCREMENTAL){
      if (target == nullptr) {
        Serial.println("ERROR: target is nullptr");
        return 0.0;
      }
      // We return an estimate of the CHANGE to target. 
      return convert_block_to_world_units(incremental_buffer + (absolute_buffer - *target)); //this subtraction term will result in 0 if nothing has been written to the buffer

    }else{ //ABSOLUTE
      return convert_block_to_world_units(incremental_buffer + absolute_buffer);
    }
  }
}

float64_t BlockPort::read_absolute(){
  return read(ABSOLUTE);
}

float64_t BlockPort::read_target(){
  return *this->target;
}

// - Block Functions -
// Called by the block that has instantiated this BlockPort.
void BlockPort::begin(volatile float64_t *target, uint8_t direction, Plugin *parent){
  set_target(target);
  parent_Plugin = parent;
  blockport_direction = direction;
}

void BlockPort::set_target(volatile float64_t *target){
  this->target = target;
}

void BlockPort::update(){
  // Updates the state of the target based on the buffers, and vice-versa.
  if(update_has_run){ //nothing has changed since the last update.
    incremental_buffer = 0;
  }else{
    update_has_run = true; // flag that we've updated the buffer states
  }

  if(target != nullptr){ //make sure we even have a target.
    absolute_buffer += incremental_buffer; //update absolute buffer to reflect how we're about to set target
    incremental_buffer = absolute_buffer - *target; //update incremental buffer to reflect changes to target
    *target = absolute_buffer; //update target
  }
}

void BlockPort::reverse_update(){
  // Performs an update of the buffers based on direct changes made to the target position.
  update_has_run = true;
  if(target != nullptr){ //make sure we even have a target.
    incremental_buffer = *target - absolute_buffer;
    absolute_buffer = *target;
  }
}

void BlockPort::set(float64_t value, uint8_t mode){
  // add following 3 lines
  if (target == nullptr){
    Serial.println("ERROR: target is nullptr");
    return;
  }
  update_has_run = true; //we set this to reflect that the buffers contain the current state of the target
  if(mode == INCREMENTAL){
    incremental_buffer = value;
    *target += value;
    absolute_buffer = *target;
  }else{ //ABSOLUTE
    incremental_buffer = value - *target;
    absolute_buffer = value;
    *target = value;
  }
}

void BlockPort::reset(float64_t value, bool raw){
  //resets the target and absolute_buffer to a particular value, BUT clears increment_buffer
  noInterrupts();
  if(raw == false){ //input is in world units
    value = convert_world_to_block_units(value);
  }
  update_has_run = true;
  incremental_buffer = 0;
  absolute_buffer = value;
  *target = value;
  interrupts();
}

void BlockPort::push(uint8_t mode){
  // pushes the buffer state of this BlockPort onto a target.
  // Note that for pushing, we are using the incremental and absolute buffers slightly differently;
  // we don't have the notion of pre and post- update, because these values are being set internally.

  if((target_BlockPort != nullptr) && push_pull_enabled){
    if(mode == INCREMENTAL){
      target_BlockPort->write(convert_block_to_world_units(incremental_buffer), INCREMENTAL);
    }else{
      target_BlockPort->write(convert_block_to_world_units(absolute_buffer), ABSOLUTE);
    }
  }
}

void BlockPort::pull(uint8_t mode){
  // pulls the buffer state of a target BlockPort onto this BlockPort
  // THIS NEEDS TO BE CALLED BEFORE update();
  if((target_BlockPort != nullptr) && push_pull_enabled){
    write(target_BlockPort->read(mode), mode);
  }
}

void BlockPort::push_deep(DecimalPosition abs_value){
  switch(blockport_direction){
    case BLOCKPORT_UNDEFINED:
      reset(abs_value); //we assume this is an input, but at the terminus of the mapping chain (or terminus of components that support synchronization)
      break;
    
    case BLOCKPORT_INPUT:
      if(parent_Plugin != nullptr){
        reset(abs_value); //set our position. input value is provided in world units.
        parent_Plugin->push_deep(); //this method in parent plugin will run e.g. kinematic transform, then call push_deep on its blockports
      }
      break;
    
    case BLOCKPORT_OUTPUT:
      if(target_BlockPort != nullptr){
        reset(abs_value, true); //input value provided by parent_Plugin.push_deep(), and comes in raw
        target_BlockPort->push_deep(read(ABSOLUTE)); //has target
      }
      break;    
  }
}

DecimalPosition BlockPort::pull_deep(){
  switch(blockport_direction){
    case BLOCKPORT_INPUT:
      if(parent_Plugin != nullptr){
        parent_Plugin->pull_deep(); //this method in parent plugin will call pull_deep on its blockports, and then run e.g. kinematic transform
        return read(ABSOLUTE); //return our position
      }
      break;
    
    case BLOCKPORT_OUTPUT:
      if(target_BlockPort != nullptr){
        DecimalPosition read_value = target_BlockPort->pull_deep();
        reset(read_value); //resets internal position to match downstream position
        return read_value;
      }
      break;    
  }
  return read(ABSOLUTE);
}


void BlockPort::enable(){
  push_pull_enabled = true;
}

void BlockPort::disable(){
  push_pull_enabled = false;
}

void BlockPort::enroll(RPC *rpc, const String& instance_name){
  rpc->enroll(instance_name, "read", *this, &BlockPort::read_absolute); //for simplicity we're enrolling this as "read", but will return the absolute position.
  rpc->enroll(instance_name, "read_deep", *this, &BlockPort::read_deep);
  rpc->enroll(instance_name, "reset_deep", *this, &BlockPort::reset_deep);
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