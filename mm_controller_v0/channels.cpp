#include "arm_math.h"
#include <cstdint>
#include <sys/_stdint.h>
#include <sys/types.h>
#include "channels.hpp"
#include "core.hpp"
/*
Channels Module of the StepDance Control System

[More Details to be Added]

A part of the Mixing Metaphors Project
(c) 2025 Ilan Moyer, Jennifer Jacobs, Devon Frost
*/

// -- Registration --
Channel* registered_channels[MAX_NUM_CHANNELS]; //stores all registered channels
uint8_t num_registered_channels = 0; //tracks the number of registered channels

// -- General Functions --
void drive_all_registered_channels(){
  for(uint8_t channel_index = 0; channel_index < num_registered_channels; channel_index ++){
    registered_channels[channel_index] ->drive_to_target();
  }
}

void activate_channels(){
  add_function_to_frame(drive_all_registered_channels);
  add_function_to_frame(transmit_frames_on_all_output_ports);
};

// -- Channel Object Methods --
Channel::Channel(){};

void Channel::initialize_state(){
  // Initializes the state of the channel.
  target_position = 0;
  target_position_2 = 0;
  current_position = 0;
  accumulator = 0;
  last_direction = 0;
  set_max_pulse_rate((float)PULSE_MAX_RATE);
}

void Channel::set_max_pulse_rate(float max_pulses_per_sec){
  // sets the maximum pulse rate permissible on a channel.
  const float tick_time_seconds = (float) CORE_FRAME_PERIOD_US / 1000000.0; //seconds per tick
  float pulses_per_tick = max_pulses_per_sec * tick_time_seconds; //steps per tick
  if(pulses_per_tick>1.0){
    //cap the velocity at 1 step per tick
    pulses_per_tick = 1.0;
  }
  accumulator_velocity = (float)((float)ACCUMULATOR_THRESHOLD * pulses_per_tick);
}

void Channel::begin(){
  // Initializes the channel without an output.
  // This is useful in cases where we are using the channel as an intermediary.
  begin(nullptr, -1);
}

void Channel::begin(OutputPort* target_output_port, uint8_t output_signal){
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

  // Initialize transmissions
  target_transmission.begin(&target_position);
  target_transmission_2.begin(&target_position_2);
}

void Channel::register_channel(){
  // Registers the channel with the pulse generator routine
  if(num_registered_channels < MAX_NUM_CHANNELS){
    registered_channels[num_registered_channels] = this;
    num_registered_channels ++;
  } // NOTE: should add a return value if it works
}

void Channel::drive_to_target(){
  // This function should be called every signal frame period.
  // It attempts to drive the channel's current position to the target position,
  // by generating a signal if a) there is a non-zero distance to the target, and
  // b) doing so would not violate the maximum pulse rate for the channel.

  // 1. Increment the accumulator. This is used to determine if generating a pulse
  //    signal would exceed the maximum pulse frequency on the channel. 
  if(accumulator < 2*ACCUMULATOR_THRESHOLD){ //only bother incrementing if meaningful (avoids overruns)
    accumulator += accumulator_velocity;
  }
  
  // 2. Calculate pulse distance between target and current position. Both target positions contribute.
  float64_t delta_position = target_position + target_position_2 - current_position;

  // 3. Determine direction of motion
  int direction;
  if(delta_position >= 0.5){
    direction = DIRECTION_FORWARD;
  }else if (delta_position < 0.5){
    direction = DIRECTION_REVERSE;
  }else{
    direction = last_direction;
  }

  // 4. Try to close pulse distance
  if(delta_position != 0){

    // calculate active accumulator threshold. This catches the case where we reverse direction.
    float accumulator_active_threshold;
    if(direction != last_direction){
      accumulator_active_threshold = ACCUMULATOR_THRESHOLD * 2;
    }else{
      accumulator_active_threshold = ACCUMULATOR_THRESHOLD;
    }

    // check if we're taking a pulse
    if(accumulator >= accumulator_active_threshold){
      pulse(direction);
      accumulator = 0;
    }
  }
}

void Channel::pulse(int8_t direction){
  // Generates a step pulse, and releases a signal on the output channel
  if(direction == DIRECTION_FORWARD){
    current_position ++;
    last_direction = DIRECTION_FORWARD;
    target_output_port->add_signal(output_signal, DIRECTION_FORWARD);
  }else{
    current_position --;
    last_direction = DIRECTION_REVERSE;
    target_output_port->add_signal(output_signal, DIRECTION_REVERSE);
  }
}