#include <cstdint>
#include <sys/_stdint.h>
#include <sys/types.h>
#include "channels.hpp"
/*
Channels Module of the StepDance Control System

[More Details to be Added]

A part of the Mixing Metaphors Project
(c) 2025 Ilan Moyer, Jennifer Jacobs, Devon Frost
*/

// -- Registration --
channel* registered_channels[MAX_NUM_CHANNELS]; //stores all registered channels
uint8_t num_registered_channels = 0; //tracks the number of registered channels

// -- Channel Object Methods --
channel::channel(){};

void channel::initialize_state(){
  // Initializes the state of the channel.
  target_position = 0;
  target_position_2 = 0;
  current_position = 0;
  accumulator = 0;
  last_direction = 0;
  set_max_pulse_rate((float)PULSE_MAX_RATE);
}

void channel::set_max_pulse_rate(float max_pulses_per_sec){
  // sets the maximum pulse rate permissible on a channel.
  const float tick_time_seconds = (float) SIGNAL_FRAME_PERIOD_US / 1000000.0; //seconds per tick
  float pulses_per_tick = max_pulses_per_sec * tick_time_seconds; //steps per tick
  if(pulses_per_tick>1.0){
    //cap the velocity at 1 step per tick
    pulses_per_tick = 1.0;
  }
  accumulator_velocity = (float)((float)ACCUMULATOR_THRESHOLD * pulses_per_tick);
}

void channel::begin(){
  // Initializes the channel without an output.
  // This is useful in cases where we are using the channel as an intermediary.
  begin(nullptr, -1);
}

void channel::begin(output_port* target_output_port, uint8_t output_signal){
  // Initializes the channel object
  //
  // target_output_port -- a pointer to an output port on which this channel will generate signals.
  //                       If nullptr, the channel will not register for updates.
  // output_signal -- a signal ID
  
  if(target_output_port == nullptr){ // no output port provided
    initialize_state();
  }else{
    initialize_state();
    this->target_output_port = target_output_port;
    this->output_signal = output_signal;
    register_channel(); //register channel with pulse generator
  }
}

void channel::register_channel(){
  // Registers the channel with the pulse generator routine
  if(num_registered_channels < MAX_NUM_CHANNELS){
    registered_channels[num_registered_channels] = this;
    num_registered_channels ++;
  } // NOTE: should add a return value if it works
}

void channel::drive_to_target(){
  // 1. Calculate pulse distance between target and current position. Both target positions contribute.
  int32_t delta_position = target_position + target_position_2 - current_position;

  // 2. Determine direction of motion
  int direction;
  if(delta_position > 0){
    direction = DIRECTION_FORWARD;
  }else if (delta_position < 0){
    direction = DIRECTION_REVERSE;
  }else{
    direction = last_direction;
  }

  // 3. Try to close pulse distance
  if(delta_position != 0){
    // calculate active accumulator threshold
    float accumulator_active_threshold;
    if(direction != last_direction){
      accumulator_active_threshold = ACCUMULATOR_THRESHOLD * 2;
    }else{
      accumulator_active_threshold = ACCUMULATOR_THRESHOLD;
    }
  }
}