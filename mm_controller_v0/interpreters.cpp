#include <cmath>
#include "arm_math.h"
/*
Interpreter Module of the StepDance Control System

This module provides buffered motion interpretation.

[More Details to be Added]

A part of the Mixing Metaphors Project
(c) 2025 Ilan Moyer, Jennifer Jacobs, Devon Frost
*/

#include "core_pins.h"
#include <sys/_stdint.h>
#include "interpreters.hpp"
#include "core.hpp"

TimeBasedInterpreter::TimeBasedInterpreter(){};

int16_t TimeBasedInterpreter::add_block(struct motion_block* block_to_add){
  // Adds a motion block to the interpreter
  //
  // block_to_add -- a pointer to a TimeBasedInterpreter::motion_block struct.
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

void TimeBasedInterpreter::advance_head(volatile uint16_t* target_head){
  (*target_head)++;
  if(*target_head == TBI_BLOCK_QUEUE_SIZE){
    *target_head = 0;
  }
}

void TimeBasedInterpreter::reset_block_queue(){
  slots_remaining = TBI_BLOCK_QUEUE_SIZE;
  next_read_index = 0;
  next_write_index = 0;
}

void TimeBasedInterpreter::run(){
  if((in_block == 0) && (slots_remaining < TBI_BLOCK_QUEUE_SIZE)){ //idle, but a new block is available
    pull_block();
  }
  if(in_block){ //when a block is first loaded, we're already at the end of the move's first frame, so we need to do something.
    run_frame_on_active_block();
  }
}

void TimeBasedInterpreter::pull_block(){
  //Pulls a block from the queue and configures active registers for a move

  float32_t block_time_s = block_queue[next_read_index].block_time_s;
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
      active_axes_velocity_mm_per_frame[active_index] = (axis_distance_mm / block_time_s) * CORE_FRAME_PERIOD_S; //set velocity of each axis, in mm/frame
      active_axes_current_distance_mm[axis_index] = 0; //clear current distances
    }else{
      active_axes[axis_index] = TBI_AXIS_INACTIVE; //clear active flag
    }
  }

  slots_remaining ++; //increment slots remaining
  advance_head(&next_read_index); //advance read head

  if(active_frames){
    in_block = 1; //flag that we are now in a block
  }
}

void TimeBasedInterpreter::run_frame_on_active_block(){
  //Run a frame of the active block
  
  //check for end of move
  active_axes_remaining_distance_mm[TBI_AXIS_V] -= active_axes_velocity_mm_per_frame[TBI_AXIS_V];
  if(fabs(active_axes_remaining_distance_mm[TBI_AXIS_V]) < (fabs(active_axes_remaining_distance_mm[TBI_AXIS_V]) * 0.1)) //<--- HERE

  for(uint8_t axis_index = 0; axis_index < TBI_NUM_AXES; axis_index++){ //iterate over all axes
    if(active_axes[axis_index] == TBI_AXIS_ACTIVE){
      
    }
  }  
}

void TimeBasedInterpreter::begin(){
  reset_block_queue();
  register_plugin();
}