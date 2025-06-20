#include <cmath>
#include "arm_math.h"
#include "core.hpp"
/*
Interpolator Module of the StepDance Control System

This module provides buffered motion interpretation.

[More Details to be Added]

A part of the Mixing Metaphors Project
(c) 2025 Ilan Moyer, Jennifer Jacobs, Devon Frost
*/

#include "core_pins.h"
#include <sys/_stdint.h>
#include "interpolators.hpp"
#include "core.hpp"

TimeBasedInterpolator::TimeBasedInterpolator(){};

void TimeBasedInterpolator::map(uint8_t output_index, Transmission* target_transmission){
  output_transmissions[output_index] = target_transmission;
}

int16_t TimeBasedInterpolator::add_block(struct motion_block* block_to_add){
  // Adds a motion block to the interpolator
  //
  // block_to_add -- a pointer to a TimeBasedInterpolator::motion_block struct.
  //
  // Returns the number of available slots in the motion queue AFTER the block was added.
  // A return value of -1 indicates that the block was not loaded due to lack of space.
  if(slots_remaining){
    block_queue[next_write_index] = *block_to_add; //shallow copy, not positive this will work
    slots_remaining --;
    advance_head(&next_write_index);
    return (int16_t)slots_remaining;
  }
  else{
    return -1;
  }
}

void TimeBasedInterpolator::advance_head(volatile uint16_t* target_head){
  (*target_head)++;
  if(*target_head == TBI_BLOCK_QUEUE_SIZE){
    *target_head = 0;
  }
}

void TimeBasedInterpolator::reset_block_queue(){
  slots_remaining = TBI_BLOCK_QUEUE_SIZE;
  next_read_index = 0;
  next_write_index = 0;
}

void TimeBasedInterpolator::run(){
  if((in_block == 0) && (slots_remaining < TBI_BLOCK_QUEUE_SIZE)){ //idle, but a new block is available
    pull_block();
  }
  if(in_block){ //when a block is first loaded, we're already at the end of the move's first frame, so we need to do something.
    run_frame_on_active_block();
  }
}

void TimeBasedInterpolator::pull_block(){
  //Pulls a block from the queue and configures active registers for a move

  float32_t block_time_s = block_queue[next_read_index].block_time_s;
  if(block_time_s>0){ //catch case where a 0 or negative-time block gets issued
    active_block_id = block_queue[next_read_index].block_id;
    active_block_type = block_queue[next_read_index].block_type;

    //manually populate target distances
    active_axes_remaining_distance_mm[TBI_AXIS_X] = block_queue[next_read_index].block_position_delta.x_mm;
    active_axes_remaining_distance_mm[TBI_AXIS_Y] = block_queue[next_read_index].block_position_delta.y_mm;
    active_axes_remaining_distance_mm[TBI_AXIS_Z] = block_queue[next_read_index].block_position_delta.z_mm;
    active_axes_remaining_distance_mm[TBI_AXIS_E] = block_queue[next_read_index].block_position_delta.e_mm;
    active_axes_remaining_distance_mm[TBI_AXIS_R] = block_queue[next_read_index].block_position_delta.r_mm;
    active_axes_remaining_distance_mm[TBI_AXIS_T] = block_queue[next_read_index].block_position_delta.t_rad;
    active_axes_remaining_distance_mm[TBI_AXIS_V] = 1; //virtual axis, always set to 1mm

    // configure the other active move registers
    for(uint8_t axis_index = 0; axis_index < TBI_NUM_AXES; axis_index++){ //iterate over all axes
      float64_t axis_distance_mm = active_axes_remaining_distance_mm[axis_index];
      if(axis_distance_mm != 0){
        active_axes[axis_index] = TBI_AXIS_ACTIVE; //flag active axes
        active_axes_velocity_mm_per_frame[axis_index] = (axis_distance_mm / block_time_s) * CORE_FRAME_PERIOD_S; //set velocity of each axis, in mm/frame
      }else{
        active_axes[axis_index] = TBI_AXIS_INACTIVE; //clear active flag
      }
    }

    slots_remaining ++; //increment slots remaining
    advance_head(&next_read_index); //advance read head
    in_block = 1; //flag that we are now in a block
  }
}

void TimeBasedInterpolator::run_frame_on_active_block(){
  //Run a frame of the active block
  
  //check for end of move
  uint8_t end_of_move = 0;
  active_axes_remaining_distance_mm[TBI_AXIS_V] -= (speed_overide*active_axes_velocity_mm_per_frame[TBI_AXIS_V]);
  if((active_axes_remaining_distance_mm[TBI_AXIS_V]) < (0.5*active_axes_velocity_mm_per_frame[TBI_AXIS_V])){ //end of move
    end_of_move = 1;
    in_block = 0; //flag to exit block
  }

  for(uint8_t axis_index = 0; axis_index < (TBI_NUM_AXES-1); axis_index++){ //iterate over all axes EXCEPT the virtual axis
    if(active_axes[axis_index] == TBI_AXIS_ACTIVE){
      if(end_of_move){
        if(output_transmissions[axis_index] != nullptr){
          output_transmissions[axis_index]->increment(active_axes_remaining_distance_mm[axis_index]); //send out whatever is remaining
        }
      }else{
        if(output_transmissions[axis_index] != nullptr){
          output_transmissions[axis_index]->increment(speed_overide*active_axes_velocity_mm_per_frame[axis_index]); //move by the frame velocity
        }
        active_axes_remaining_distance_mm[axis_index] -= (speed_overide*active_axes_velocity_mm_per_frame[axis_index]);
      }
    }
  }  
}

void TimeBasedInterpolator::begin(){
  reset_block_queue();
  register_plugin(PLUGIN_LOOP);
}