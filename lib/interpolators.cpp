#include <iterator>
#include <cmath>
#include "arm_math.h"
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
#include "rpc.hpp"

TimeBasedInterpolator::TimeBasedInterpolator(){};

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

int16_t TimeBasedInterpolator::add_move(uint8_t mode, float32_t move_velocity_per_s, DecimalPosition x, DecimalPosition y, DecimalPosition z, DecimalPosition e, DecimalPosition r, DecimalPosition t){
  return _add_move(mode, 0, move_velocity_per_s, x, y, z, e, r, t);
}

int16_t TimeBasedInterpolator::add_timed_move(uint8_t mode, float32_t move_time_s, DecimalPosition x, DecimalPosition y, DecimalPosition z, DecimalPosition e, DecimalPosition r, DecimalPosition t){
  return _add_move(mode, move_time_s, 0, x, y, z, e, r, t);
}


int16_t TimeBasedInterpolator::_add_move(uint8_t mode, float32_t move_time_s, float32_t move_velocity_per_s, DecimalPosition x, DecimalPosition y, DecimalPosition z, DecimalPosition e, DecimalPosition r, DecimalPosition t){
  struct motion_block new_block;
  new_block.block_position.x_mm = x;
  new_block.block_position.y_mm = y;
  new_block.block_position.z_mm = z;
  new_block.block_position.e_mm = e;
  new_block.block_position.r_mm = r;
  new_block.block_position.t_rad = t;
  new_block.block_time_s = move_time_s;
  new_block.block_velocity_per_s = move_velocity_per_s;
  switch(mode){
    case INCREMENTAL:
      new_block.block_type = BLOCK_TYPE_INCREMENTAL;
      break;
    case ABSOLUTE:
      new_block.block_type = BLOCK_TYPE_ABSOLUTE;
      break;
    case GLOBAL:
      new_block.block_type = BLOCK_TYPE_GLOBAL;
      break;
  }

  return add_block(&new_block);
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

bool TimeBasedInterpolator::is_idle(){
  //returns true if the interpolator is idle
  if(slots_remaining == TBI_BLOCK_QUEUE_SIZE && in_block == 0){
    return true;
  }else{
    return false;
  }
}

bool TimeBasedInterpolator::queue_is_full(){
  return (slots_remaining == 0);
};

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

  // Step 1: calculate remaining distance, based on mode
  uint8_t block_type = block_queue[next_read_index].block_type;
  if(block_type == BLOCK_TYPE_INCREMENTAL){ //INCREMENTAL positions were provided, copy directly
    active_axes_remaining_distance_mm[TBI_AXIS_X] = block_queue[next_read_index].block_position.x_mm;
    active_axes_remaining_distance_mm[TBI_AXIS_Y] = block_queue[next_read_index].block_position.y_mm;
    active_axes_remaining_distance_mm[TBI_AXIS_Z] = block_queue[next_read_index].block_position.z_mm;
    active_axes_remaining_distance_mm[TBI_AXIS_E] = block_queue[next_read_index].block_position.e_mm;
    active_axes_remaining_distance_mm[TBI_AXIS_R] = block_queue[next_read_index].block_position.r_mm;
    active_axes_remaining_distance_mm[TBI_AXIS_T] = block_queue[next_read_index].block_position.t_rad;
    active_axes_remaining_distance_mm[TBI_AXIS_V] = 1; //virtual axis, always set to 1mm    
  }else{ //ABSOLUTE OR GLOBAL positions provided
    if(block_type == BLOCK_TYPE_GLOBAL){ //synchronize state on all positions
      output_x.pull_deep();
      output_y.pull_deep();
      output_z.pull_deep();
      output_e.pull_deep();
      output_r.pull_deep();
      output_t.pull_deep();
    }
    active_axes_remaining_distance_mm[TBI_AXIS_X] = block_queue[next_read_index].block_position.x_mm - output_position_x;
    active_axes_remaining_distance_mm[TBI_AXIS_Y] = block_queue[next_read_index].block_position.y_mm - output_position_y;
    active_axes_remaining_distance_mm[TBI_AXIS_Z] = block_queue[next_read_index].block_position.z_mm - output_position_z;
    active_axes_remaining_distance_mm[TBI_AXIS_E] = block_queue[next_read_index].block_position.e_mm - output_position_e;
    active_axes_remaining_distance_mm[TBI_AXIS_R] = block_queue[next_read_index].block_position.r_mm - output_position_r;
    active_axes_remaining_distance_mm[TBI_AXIS_T] = block_queue[next_read_index].block_position.t_rad - output_position_t;
    active_axes_remaining_distance_mm[TBI_AXIS_V] = 1; //virtual axis, always set to 1mm   
  }

  float32_t block_time_s = block_queue[next_read_index].block_time_s;
  float32_t block_velocity_per_s = block_queue[next_read_index].block_velocity_per_s;
  if(block_time_s == 0){ //velocity-based move
    if(block_velocity_per_s > 0){ //need to calculate block time based on velocity and distance
      float64_t move_distance = 0;
      for(uint8_t axis_index = 0; axis_index < TBI_NUM_AXES; axis_index++){ //iterate over all axes to calc distance^2
        move_distance += (active_axes_remaining_distance_mm[axis_index] * active_axes_remaining_distance_mm[axis_index]);
      }
      move_distance = std::sqrt(move_distance);
      block_time_s = move_distance / block_velocity_per_s;
    }else{
      return; //both time and velocity are zero, skip block.
    }
  }

  // Expose the active block time to other plugins
  output_duration.set(block_time_s, ABSOLUTE);
  output_duration.push();

  output_parameter.set(0, ABSOLUTE);
  output_parameter.push();

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
        output_BlockPorts[axis_index]->set(active_axes_remaining_distance_mm[axis_index], INCREMENTAL);
      }else{
        output_BlockPorts[axis_index]->set(speed_overide*active_axes_velocity_mm_per_frame[axis_index], INCREMENTAL);
        active_axes_remaining_distance_mm[axis_index] -= (speed_overide*active_axes_velocity_mm_per_frame[axis_index]);
      }
      output_BlockPorts[axis_index]->push();
    }
  }  

  // Update the virtual axis value
  output_parameter.set(1.0 - active_axes_remaining_distance_mm[TBI_AXIS_V], ABSOLUTE);
  output_parameter.push();
}

void TimeBasedInterpolator::begin(){
  output_x.begin(&output_position_x, BLOCKPORT_OUTPUT);
  output_y.begin(&output_position_y, BLOCKPORT_OUTPUT);
  output_z.begin(&output_position_z, BLOCKPORT_OUTPUT);
  output_e.begin(&output_position_e, BLOCKPORT_OUTPUT);
  output_r.begin(&output_position_r, BLOCKPORT_OUTPUT);
  output_t.begin(&output_position_t, BLOCKPORT_OUTPUT);

  output_parameter.begin(&output_position_parameter, BLOCKPORT_OUTPUT);
  output_duration.begin(&output_value_duration, BLOCKPORT_OUTPUT);

  reset_block_queue();
  register_plugin();
}

void TimeBasedInterpolator::enroll(RPC *rpc, const String& instance_name){
  rpc->enroll(instance_name, "add_move", *this, &TimeBasedInterpolator::add_move);
  rpc->enroll(instance_name, "add_timed_move", *this, &TimeBasedInterpolator::add_timed_move);
  rpc->enroll(instance_name, "is_idle", *this, &TimeBasedInterpolator::is_idle);
  rpc->enroll(instance_name, "queue_is_full", *this, &TimeBasedInterpolator::queue_is_full);
  rpc->enroll(instance_name + ".speed_override", speed_overide);
  rpc->enroll(instance_name + ".slots_remaining", slots_remaining);
  output_x.enroll(rpc, instance_name + ".output_x");
  output_y.enroll(rpc, instance_name + ".output_y");
  output_r.enroll(rpc, instance_name + ".output_r");
  output_t.enroll(rpc, instance_name + ".output_t");
  output_z.enroll(rpc, instance_name + ".output_z");
  output_e.enroll(rpc, instance_name + ".output_e");
}