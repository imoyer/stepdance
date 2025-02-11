#include <sys/_stdint.h>
#include "interpreters.hpp"

TimeBasedInterpreter::TimeBasedInterpreter();

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
  
}
