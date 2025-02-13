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
  if(*target_head == TIME_BASED_INTERPRETER_BLOCK_QUEUE_SIZE){
    *target_head = 0;
  }
}

void TimeBasedInterpreter::reset_block_queue(){
  slots_remaining = TIME_BASED_INTERPRETER_BLOCK_QUEUE_SIZE;
  next_read_index = 0;
  next_write_index = 0;
}

void TimeBasedInterpreter::run(){
  if(in_block == 0){ //not working on a block
    if(slots_remaining < TIME_BASED_INTERPRETER_BLOCK_QUEUE_SIZE){ //block available, load it!
      // manually copy the block, because active_block is volatile
      active_block.block_id = block_queue[next_read_index].block_id;
      active_block.block_position_delta.x_mm = block_queue[next_read_index].block_position_delta.x_mm;
      active_block.block_position_delta.y_mm = block_queue[next_read_index].block_position_delta.y_mm;
      active_block.block_position_delta.z_mm = block_queue[next_read_index].block_position_delta.z_mm;
      active_block.block_position_delta.e_mm = block_queue[next_read_index].block_position_delta.e_mm;
      active_block.block_position_delta.r_mm = block_queue[next_read_index].block_position_delta.r_mm;
      active_block.block_position_delta.t_rad = block_queue[next_read_index].block_position_delta.t_rad;
      active_block.block_time_s = block_queue[next_read_index].block_time_s;
      active_block.block_type = block_queue[next_read_index].block_type;

      slots_remaining ++;
      advance_head(&next_read_index);
      in_block = 1;
    }
  }
}

void TimeBasedInterpreter::begin(){
  reset_block_queue();
  register_plugin();
}