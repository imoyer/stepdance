#include "core_pins.h"
#include <sys/_stdint.h>
#include "interpreters.hpp"
#include "core.hpp"

TimeBasedInterpreter::TimeBasedInterpreter(){};

uint16_t TimeBasedInterpreter::add_block(struct motion_block* block_to_add){
  // Adds a motion block to the interpreter
  //
  // block_to_add -- a pointer to a TimeBasedInterpreter::motion_block struct.
  //
  // Returns the number of available slots in the motion queue
  if(slots_remaining){
    block_queue[next_write_index] = *block_to_add; //shallow copy, not positive this will work
    slots_remaining --;
    advance_head(&next_write_index);
  }
  return (uint16_t)slots_remaining;
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
      active_block.block_position.x_mm = block_queue[next_read_index].block_position.x_mm;
      active_block.block_position.y_mm = block_queue[next_read_index].block_position.y_mm;
      active_block.block_position.z_mm = block_queue[next_read_index].block_position.z_mm;
      active_block.block_position.e_mm = block_queue[next_read_index].block_position.e_mm;
      active_block.block_position.r_mm = block_queue[next_read_index].block_position.r_mm;
      active_block.block_position.t_rad = block_queue[next_read_index].block_position.t_rad;
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