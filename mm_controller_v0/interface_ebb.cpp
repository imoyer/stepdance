#include <cstdlib>
#include <cstring>
#include <sys/_stdint.h>
/*
EiBotBoard Interface Module of the StepDance Control System

This module provides an input interface that emulates the EiBotBoard, which allows standard AxiDraw workflows
to provide direct input. It is anticipated that this interface will feed a standard downstream motion generation synthesizer.

[More Details to be Added]

A part of the Mixing Metaphors Project
(c) 2025 Ilan Moyer, Jennifer Jacobs, Devon Frost
*/

#include "interface_ebb.hpp"
#include "interpreters.hpp"

#define EBB_INPUT_IDLE 0 //waiting on a new line
#define EBB_INPUT_IN_COMMAND 1 //reading the one or two-character command
#define EBB_INPUT_IN_STRING 2 //reading the remaining string

// DEFINE ALL COMMANDS
// We put high-frequency commands such as "SM" first, to improve search time.
struct Eibotboard::command Eibotboard::all_commands[] = {
  {.command_string = "SM", .command_function = &Eibotboard::command_stepper_move}, 
  {.command_string = "V", .command_function = &Eibotboard::command_version}, //Version Query
  {.command_string = "QC", .command_function = &Eibotboard::command_query_current},
  {.command_string = "QB", .command_function = &Eibotboard::command_query_button},
  {.command_string = "QL", .command_function = &Eibotboard::command_query_variable}
};


Eibotboard::Eibotboard(){};

void Eibotboard::begin(TimeBasedInterpreter* interpreter){
  target_interpreter = interpreter;
  Serial.begin(115200);
  reset_input_buffer();
  initialize_all_commands_struct();
}

void Eibotboard::loop(){
  if(debug_port_identified == 0){ // identify the debug port based on the first port with an available character.
    if(Serial.available() > 0){
      uint8_t character = Serial.read();
      process_character(character);
      SerialUSB1.write(character); //echo
      debug_port_identified = 1;
      ebb_serial_port = &Serial;
      debug_serial_port = &SerialUSB1;
    }
    if(SerialUSB1.available() > 0){
      uint8_t character = SerialUSB1.read();
      process_character(character);
      Serial.write(character); //echo
      debug_port_identified = 1;
      ebb_serial_port = &SerialUSB1;
      debug_serial_port = &Serial;
    }
  }else{
    while(ebb_serial_port->available() > 0){
      uint8_t character = ebb_serial_port->read();
      debug_serial_port->write(character);
      if(character == 13){ //carriage return, let's add a new line for debugging
        debug_serial_port->write(10);
      }
      process_character(character);
    }
  }
}

void Eibotboard::process_character(uint8_t character){
  if(character == 13){ //carriage return; done receiving the entire command string
    input_buffer[input_buffer_write_index] = 0; //indicate end of string
    process_command(input_command_value); //process the command
    reset_input_buffer();

  }else{ //still in command
    if(input_state == EBB_INPUT_IDLE){ //if this is the first character, mark that we're now receiving the command
      input_state = EBB_INPUT_IN_COMMAND;
    }
    if(input_state == EBB_INPUT_IN_COMMAND){ //reading the command portion of the input block
      if(character == 44){ //comma, we're now out of the command portion
        input_state = EBB_INPUT_IN_STRING; // just change input state, but don't store the comma
      }else{ //
        input_command_value <<= 8;
        input_command_value += toupper(character);
      }
    }else{
      input_buffer[input_buffer_write_index] = character; //store the character
      input_buffer_write_index ++; //increment the write buffer index
    }
  }
}

void Eibotboard::process_command(uint16_t command_value){
  // Find and call the command.
  uint8_t num_commands = sizeof(Eibotboard::all_commands) / sizeof(Eibotboard::all_commands[0]);
  for(uint8_t command_index = 0; command_index < num_commands; command_index++){ //iterate over all commands
    if(command_value == Eibotboard::all_commands[command_index].command_value){
      (this->*all_commands[command_index].command_function)(); //call the function
      return;
    }   
  }
  // didn't find the requested command
  command_generic();
}

void Eibotboard::process_string_int32(){
  // Processes the input buffer into the parameter array. This version treats all parameters as int32 values.
  char *token = strtok(input_buffer, ","); //splits a string along a comma
  num_input_parameters = 0;
  while(token != nullptr && num_input_parameters < EBB_MAX_NUM_INPUT_PARAMETERS){
    input_parameters[num_input_parameters++] = static_cast<int32_t>(strtol(token, nullptr, 10)); //converts the token string into an int32_t
  }
}

void Eibotboard::reset_input_buffer(){
  input_buffer_write_index = 0;
  input_state = EBB_INPUT_IDLE;
  input_command_value = 0;
}

void Eibotboard::initialize_all_commands_struct(){
  // Initializes the all_commands structure by pre-populating the command values for each entry. This saves processing time
  // when finding and calling functions for actual moves.
  uint8_t num_commands = sizeof(Eibotboard::all_commands) / sizeof(Eibotboard::all_commands[0]);
  for(uint8_t command_index = 0; command_index < num_commands; command_index++){ //iterate over all commands
    for(uint8_t char_index = 0; char_index < EBB_COMMAND_SIZE; char_index ++){ //iterate over all characters
      uint8_t this_char = Eibotboard::all_commands[command_index].command_string[char_index];
      if(this_char){
        Eibotboard::all_commands[command_index].command_value <<= 8;
        Eibotboard::all_commands[command_index].command_value += this_char;
      }
    }
  }
}

// -- COMMANDS --
void Eibotboard::command_version(){
  ebb_serial_port->print("EBBv13_and_above EB Firmware Version 2.7.4\r\n");
}

void Eibotboard::command_query_current(){
  ebb_serial_port->print("0800,0800\r\nOK\r\n"); //just making some stuff up here
}

void Eibotboard::command_query_button(){
  ebb_serial_port->print("0\r\nOK\r\n"); //just making some stuff up here
}

void Eibotboard::command_query_variable(){
  ebb_serial_port->print("000\r\nOK\r\n"); //temporary for now
}

void Eibotboard::command_stepper_move(){
  //debug
  debug_serial_port->print("READ HEAD: ");
  debug_serial_port->println(target_interpreter->next_read_index);
  debug_serial_port->print("BLOCK ID: ");
  debug_serial_port->println(target_interpreter->active_block.block_id);
  debug_serial_port->print("MOVE TIME: ");
  debug_serial_port->println(target_interpreter->active_block.block_time_s);
  debug_serial_port->print("X POS: ");
  debug_serial_port->println(target_interpreter->active_block.block_position.x_mm);
  debug_serial_port->print("Y POS: ");
  debug_serial_port->println(target_interpreter->active_block.block_position.y_mm);
  debug_serial_port->print("CPU USAGE: ");
  debug_serial_port->println(stepdance_get_cpu_usage());
  
  
  float move_time;
  float axis_steps_1;
  float axis_steps_2;

  process_string_int32();

  if(num_input_parameters > 0){
    move_time = static_cast<float>(input_parameters[0]);
  }else{
    return;
  }

  if(num_input_parameters > 1){
    axis_steps_1 = static_cast<float>(input_parameters[1]);
  }else{
    axis_steps_1 = 0.0;
  }

  if(num_input_parameters > 2){
    axis_steps_2 = static_cast<float>(input_parameters[2]);
  }else{
    axis_steps_2 = 0.0;
  }

  TimeBasedInterpreter::motion_block this_block = {.block_id = block_id++, .block_time_s = move_time, .block_position = {.x_mm = axis_steps_1, .y_mm = axis_steps_2}};
  // need to somehow check if the buffer has available blocks
  target_interpreter->add_block(&this_block);
  target_interpreter->in_block = 0; //temp - release active block
  ebb_serial_port->print("OK\r\n");
}

void Eibotboard::command_generic(){
  ebb_serial_port->print("OK\r\n");
}