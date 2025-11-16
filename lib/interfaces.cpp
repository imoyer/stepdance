#include <cmath>
#include "WString.h"
#include "avr/pgmspace.h"
#include <cctype>
#include <cstddef>
#include "Arduino.h"
#include "usb_serial.h"
#include "core_pins.h"
#include "arm_math.h"
#include <cstdlib>
#include <cstring>
#include <sys/_stdint.h>
/*
Interface Module of the StepDance Control System

This module provides several interfaces for pre-planned control.

1) An Eibotboard interface, which allows standard AxiDraw workflows to provide direct input. It is anticipated that this 
interface will feed a standard downstream motion generation synthesizer. 

[More Details to be Added]

A part of the Mixing Metaphors Project
(c) 2025 Ilan Moyer, Jennifer Jacobs, Devon Frost
*/

#include "interfaces.hpp"
#include "interpolators.hpp"

#define EBB_INPUT_IDLE 0 //waiting on a new line
#define EBB_INPUT_IN_COMMAND 1 //reading the one or two-character command
#define EBB_INPUT_IN_STRING 2 //reading the remaining string

// DEFINE ALL COMMANDS
// We put high-frequency commands such as "SM" first, to improve search time.
struct Eibotboard::command Eibotboard::all_commands[] = {
  {.command_string = "SM", .command_function = &Eibotboard::command_stepper_move, .execution = EBB_EXECUTE_TO_QUEUE},
  {.command_string = "SP", .command_function = &Eibotboard::command_set_pen, .execution = EBB_EXECUTE_TO_QUEUE},
  {.command_string = "V", .command_function = &Eibotboard::command_version, .execution = EBB_EXECUTE_IMMEDIATE}, //Version Query
  {.command_string = "QC", .command_function = &Eibotboard::command_query_current, .execution = EBB_EXECUTE_IMMEDIATE},
  {.command_string = "QB", .command_function = &Eibotboard::command_query_button, .execution = EBB_EXECUTE_IMMEDIATE},
  {.command_string = "QL", .command_function = &Eibotboard::command_query_variable, .execution = EBB_EXECUTE_IMMEDIATE},
  {.command_string = "QP", .command_function = &Eibotboard::command_query_pen, .execution = EBB_EXECUTE_IMMEDIATE},
  {.command_string = "SC", .command_function = &Eibotboard::command_stepper_servo_configure, .execution = EBB_EXECUTE_IMMEDIATE}
};


Eibotboard::Eibotboard(){};

void Eibotboard::begin(){
  Serial.begin(115200);
  reset_input_buffer();
  initialize_all_commands_struct();
  ebb_serial_port = &Serial; //hard-code the primary and debug serial ports
  debug_serial_port = &SerialUSB1;
  register_plugin(PLUGIN_LOOP);
  target_interpolator.begin();
}

void Eibotboard::set_ratio_xy(float output_units_mm, float input_units_steps){
  xy_conversion_mm_per_step = output_units_mm / input_units_steps;
}

void Eibotboard::set_ratio_z(float output_units_mm, float input_units_steps){
  z_conversion_mm_per_step = output_units_mm / input_units_steps;
}

void Eibotboard::loop(){
  // if(debug_port_identified == 0){ // identify the debug port based on the first port with an available character.
  //   if(Serial.available() > 0){
  //     uint8_t character = Serial.read();
  //     process_character(character);
  //     SerialUSB1.write(character); //echo
  //     debug_port_identified = 1;
  //     ebb_serial_port = &Serial;
  //     debug_serial_port = &SerialUSB1;
  //   }
  //   if(SerialUSB1.available() > 0){
  //     uint8_t character = SerialUSB1.read();
  //     process_character(character);
  //     Serial.write(character); //echo
  //     debug_port_identified = 1;
  //     ebb_serial_port = &SerialUSB1;
  //     debug_serial_port = &Serial;
  //   }
  // }else{
    if(block_pending_flag){ // if a block is pending, call that function first
      (this->*pending_block_function)();
    }
    while(ebb_serial_port->available() > 0){
      uint8_t character = ebb_serial_port->read();
      debug_serial_port->write(character);
      if(character == 13){ //carriage return, let's add a new line for debugging
        debug_serial_port->write(10);
      }
      process_character(character);
    }
  // }
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
      if((block_pending_flag == 0) || (Eibotboard::all_commands[command_index].execution == EBB_EXECUTE_EMERGENCY)){
        //we run the command if either nothing is waiting to get into the queue, or the command will execute immediately under emergency status
        (this->*all_commands[command_index].command_function)(); //call the function
        return;
      }else{ //a block is pending and it is not an emergency, so we'll ignore the command
        return;
      }
    }   
  }
  // didn't find the requested command
  command_generic();
}

void Eibotboard::process_string_int32(){
  // Processes the input buffer into the parameter array. This version treats all parameters as int32 values.
  
  // reset the buffer
  for(uint8_t buffer_index = 0; buffer_index < EBB_MAX_NUM_INPUT_PARAMETERS; buffer_index ++){
    input_parameters[buffer_index] = 0;
  }

  // process the string
  char *token = strtok(input_buffer, ","); //splits a string along a comma
  num_input_parameters = 0;
  while(token != nullptr && num_input_parameters < EBB_MAX_NUM_INPUT_PARAMETERS){
    input_parameters[num_input_parameters++] = static_cast<int32_t>(strtol(token, nullptr, 10)); //converts the token string into an int32_t
    token = strtok(nullptr, ",");
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

void Eibotboard::command_query_pen(){
  ebb_serial_port->print("1\r\nOK\r\n"); //temporary for now
}
void Eibotboard::set_servo_position(uint16_t pulse_duration_83_3_ns, int32_t *servo_position_register){
  // sets a servo position register (e.g. top and bottom positions) based on eibotboard protocol units.
  //  pulse_duration_83_3_ns -- the pulse duration, in units of 83.3 ns
  //  servo_position_register -- target register to write result to.
  float pulse_duration_us = 0.0833 * pulse_duration_83_3_ns;
  *servo_position_register = static_cast<int32_t>((pulse_duration_us - EBB_SERVO_MIDPOINT_PULSE_DURATION_US));
}

void Eibotboard::set_servo_rate(uint16_t pulse_rate_us_per_ms, float *servo_rate_register){
  // sets a servo rate register (e.g. pen up and down slew rates) based on eibotboard protocol units.
  //  pulse_rate_us_per_ms -- this is in units of 1/12000ms per 24ms, or 3.4722us/S == 3.4722 steps/sec
  *servo_rate_register = 3.4722 * static_cast<float>(pulse_rate_us_per_ms);
}

void Eibotboard::command_stepper_servo_configure(){
  process_string_int32(); //parse string into integers in input_parameters
  uint8_t parameter_index = static_cast<uint8_t>(input_parameters[0]);
  uint16_t parameter_value = static_cast<uint16_t>(input_parameters[1]);
  switch(parameter_index){
    case 4: //set servo minimum value / pen up position
      set_servo_position(parameter_value, &servo_pen_up_position_steps);
      break;
    case 5: //set servo maximum value / pen down position
      set_servo_position(parameter_value, &servo_pen_down_position_steps);
      break;
    case 10: //set both servo rates
      set_servo_rate(parameter_value, &servo_rate_up_steps_per_sec);
      set_servo_rate(parameter_value, &servo_rate_down_steps_per_sec);
      break;
    case 11: //set servo up rate
      set_servo_rate(parameter_value, &servo_rate_up_steps_per_sec);
      break;
    case 12: //set servo down rate
      set_servo_rate(parameter_value, &servo_rate_down_steps_per_sec);
      break;
  }
  ebb_serial_port->print("OK\r\n"); 
}

void Eibotboard::command_set_pen(){
  static uint8_t loading_delay_flag = 0; // 0 -- loading actual move into buffer, 1 -- loading delay into buffer
  static uint16_t delay_ms = 0;

  if(block_pending_flag == 0){ //lets load a position block
    process_string_int32();
    uint8_t command_value = static_cast<uint8_t>(input_parameters[0]);
    delay_ms = static_cast<uint16_t>(input_parameters[1]);
    
    float64_t servo_delta_steps = 0;
    float move_time_s = 0;

    switch(command_value){
      case 0: //servo max, pen down
        servo_delta_steps = static_cast<float64_t>(servo_pen_down_position_steps - servo_position_steps);
        move_time_s = fabs(servo_delta_steps) / servo_rate_down_steps_per_sec;
        break;

      case 1: //servo min, pen up
        servo_delta_steps = static_cast<float64_t>(servo_pen_up_position_steps - servo_position_steps);
        move_time_s = fabs(servo_delta_steps) / servo_rate_up_steps_per_sec;
        break;   
    }
    servo_position_steps += servo_delta_steps;

    if(move_time_s == 0){
      //we're already in position, do nothing
      ebb_serial_port->print("OK\r\n");
      return;
    }else{ //load up the pending block
      pending_block = {.block_id = block_id++, .block_time_s = move_time_s, .block_position = {.x_mm = 0, .y_mm = 0, .z_mm = servo_delta_steps * z_conversion_mm_per_step, .e_mm = 0, .r_mm = 0, .t_rad = 0}};
      block_pending_flag = EBB_BLOCK_PENDING;
      pending_block_function = &Eibotboard::command_set_pen; // tag this function as having originated the pending block
    }
  }

  // Try adding the pending block to queue.
  int16_t available_slots = target_interpolator.add_block(&pending_block);

  if(available_slots >= 0){ //move successfully added
    if(delay_ms == 0){ //we don't need to add a delay
      ebb_serial_port->print("OK\r\n");
      block_pending_flag = 0; //release the hold on the pending block
      debug_buffer_full_flag = 0;
      debug_serial_port->println("PEN MOVE");
      debug_report_pending_block(false);
    }else{ // a delay was requested
      if(loading_delay_flag){ //the queued move was the delay move
        ebb_serial_port->print("OK\r\n");
        loading_delay_flag = 0;
        block_pending_flag = 0;
        debug_serial_port->println("PEN DELAY");
        debug_report_pending_block(false);
      }else{ // just loaded a move command, now we we need to load a delay
        debug_serial_port->println("PEN MOVE");
        debug_report_pending_block(false);
        float move_time_s = static_cast<float>(delay_ms) / 1000;
        pending_block = {.block_id = block_id++, .block_time_s = move_time_s, .block_position = {.x_mm = 0, .y_mm = 0, .z_mm = 0, .e_mm = 0, .r_mm = 0, .t_rad = 0}};
        block_pending_flag = EBB_BLOCK_PENDING;
        pending_block_function = &Eibotboard::command_set_pen; // tag this function as having originated the pending block
        loading_delay_flag = 1;       
      }
    }
  }
}

void Eibotboard::command_stepper_move(){
  if(block_pending_flag == 0){ //let's load up a block
    // Step 1:  Convert motion block string into parameters
    //          These get placed in the input_parameters array
    process_string_int32();

    // Step 2:  Read in parameters, and set defaults when needed
    float32_t move_time_ms;
    float64_t motor_1_delta_steps;
    float64_t motor_2_delta_steps;

    if(num_input_parameters > 0){
      move_time_ms = static_cast<float32_t>(input_parameters[0]);
    }else{
      return;
    }

    if(num_input_parameters > 1){
      motor_1_delta_steps = static_cast<float64_t>(input_parameters[1]);
    }else{
      motor_1_delta_steps = 0.0;
    }

    if(num_input_parameters > 2){
      motor_2_delta_steps = static_cast<float64_t>(input_parameters[2]);
    }else{
      motor_2_delta_steps = 0.0;
    }

    // Step 3: Convert parameters from command space (motor steps, move time in ms) to standard space (xy mm, move time in sec)
    //          NOTE: We assume an h-bot transform, which we hardcode here rather than use the kinematics module, for simplicity.
    float32_t move_time_s = move_time_ms / 1000;
    float64_t motor_1_delta_mm = motor_1_delta_steps * xy_conversion_mm_per_step;
    float64_t motor_2_delta_mm = motor_2_delta_steps * xy_conversion_mm_per_step;
    float64_t x_delta_mm = 0.5*(motor_1_delta_mm + motor_2_delta_mm);
    float64_t y_delta_mm = 0.5*(motor_1_delta_mm - motor_2_delta_mm);

    // Step 4:  Load parameters into a motion block
    pending_block = {.block_id = block_id++, .block_time_s = move_time_s, .block_position = {.x_mm = x_delta_mm, .y_mm = y_delta_mm, .z_mm = 0, .e_mm = 0, .r_mm = 0, .t_rad = 0}};
    block_pending_flag = EBB_BLOCK_PENDING;
    pending_block_function = &Eibotboard::command_stepper_move; // tag this function as having originated the pending block
  }
  
  // Step 5:  Try adding the pending block to queue.
  int16_t available_slots = target_interpolator.add_block(&pending_block);

  // Step 6: Check if block was added, and respond
  if(available_slots >= 0){ //move successfully added
    ebb_serial_port->print("OK\r\n");
    block_pending_flag = 0; //release the hold on the pending block
    debug_buffer_full_flag = 0;
    debug_report_pending_block(false);
  }else if(debug_buffer_full_flag == 0){ //message once that buffer is full
    debug_report_pending_block(true);
    debug_buffer_full_flag = 1;
  }
  //debug
}

void Eibotboard::command_generic(){
  ebb_serial_port->print("OK\r\n");
}

void Eibotboard::debug_report_pending_block(bool waiting_for_slot){
  if(waiting_for_slot){
    debug_serial_port->print("QUEUE FULL; BLOCK ");
    debug_serial_port->print(pending_block.block_id);
    debug_serial_port->println(" PENDING");    
  }else{
    debug_serial_port->print("QUEUED BLOCK ");
    debug_serial_port->println(pending_block.block_id);
  }
  debug_serial_port->print("  MOVE TIME: ");
  debug_serial_port->println(pending_block.block_time_s);
  debug_serial_port->print("  X DELTA: ");
  debug_serial_port->println(pending_block.block_position.x_mm);
  debug_serial_port->print("  Y DELTA: ");
  debug_serial_port->println(pending_block.block_position.y_mm);
  debug_serial_port->print("  Z DELTA: ");
  debug_serial_port->println(pending_block.block_position.z_mm);
  debug_serial_port->print("  CPU USAGE: ");
  debug_serial_port->print(stepdance_get_cpu_usage()*100);
  debug_serial_port->println("%");
  debug_serial_port->print("  SLOTS REMAINING: ");
  debug_serial_port->println(target_interpolator.slots_remaining);  
}

// ---- GCODE INTERFACE ----
struct GCodeInterface::code GCodeInterface::all_codes[] = {
  {.code_string = "G0", .code_function = &GCodeInterface::g0_rapid, .execution = EXECUTE_INTERPOLATOR},
  {.code_string = "G1", .code_function = &GCodeInterface::g1_move, .execution = EXECUTE_INTERPOLATOR},
  {.code_string = "$", .code_function = &GCodeInterface::_help, .execution = EXECUTE_NOW},
  {.code_string = "$$", .code_function = &GCodeInterface::_report_parameters, .execution = EXECUTE_NOW},
  {.code_string = "\x18", .code_function = &GCodeInterface::_soft_reset, .execution = EXECUTE_NOW},
  {.code_string = "?", .code_function = &GCodeInterface::_status_report, .execution = EXECUTE_NOW},
  {.code_string = "~", .code_function = &GCodeInterface::_cycle_start, .execution = EXECUTE_NOW},
  {.code_string = "!", .code_function = &GCodeInterface::_feed_hold, .execution = EXECUTE_NOW},
  {.code_string = "$G", .code_function = &GCodeInterface::_parser_state, .execution = EXECUTE_NOW},
  {.code_string = "G4", .code_function = &GCodeInterface::g4_dwell, .execution = EXECUTE_QUEUE}
};

GCodeInterface::GCodeInterface(){};

void GCodeInterface::begin(){
  begin(&Serial);
}

void GCodeInterface::begin(Stream *target_stream){
  this->gcode_stream = target_stream; //sets the RPC interface
  register_plugin(PLUGIN_LOOP); //this runs in the main program loop
  reset_input_line_buffer();
  target_interpolator.begin();
}

void GCodeInterface::begin(usb_serial_class *target_usb_serial){
  target_usb_serial -> begin(115200); //baud rate is unused
  this->gcode_stream = target_usb_serial; //sets the RPC interface
  register_plugin(PLUGIN_LOOP); //this runs in the main program loop
  reset_input_line_buffer();
  target_interpolator.begin();
};

void GCodeInterface::begin(HardwareSerialIMXRT *target_serial, uint32_t baud, uint16_t format){
  target_serial->begin(baud, format);
  this->gcode_stream = target_serial; //sets the RPC interface
  register_plugin(PLUGIN_LOOP); //this runs in the main program loop
  reset_input_line_buffer();
  target_interpolator.begin();
}

void GCodeInterface::reset_input_line_buffer(){
  input_line_buffer_index = 0;
}

void GCodeInterface::loop(){
  if(receiver_state == RECEIVER_READY){
    reset_input_line_buffer();
    receiver_state = RECEIVER_READING; //kick over to reading state immediately   
  }

  if(receiver_state == RECEIVER_READING){ //read in a line
    while(gcode_stream->available() > 0){
      uint8_t character = gcode_stream->read();
      if(strchr(REALTIME_LETTERS, character)){ //it's a realtime command. We extract and process seperately.
        execute_realtime(character);
        continue;
      }
      if(process_character(character)){ //newline character received, done with line
        receiver_state = RECEIVER_PROCESSING;
        break;
      }
    }
  }

  if(receiver_state == RECEIVER_PROCESSING){
    for (int i = 0; i<20; i++){
      SerialUSB1.printf("%02X ", (unsigned char)input_line_buffer[i]);
    }
    SerialUSB1.print('\n');
    if(tokenize_block()){ //a block with tokens, and at least one key token, was returned.
      if(preprocess_block()){  //load block with target function and execution scope. Returns false if no matching function
        if(dispatch_block()){ //block was successfully dispatched
          send_ok();
          receiver_state = RECEIVER_READY;
        }else{
          receiver_state = RECEIVER_BLOCKED;
        };
      }else{
        send_error(ERROR_UNSUPPORTED_CODE);
        receiver_state = RECEIVER_READY;
      }
    }else{
      if(input_line_buffer_index != 0){ //don't send an error if line is empty (as when only a realtime command is received)
        send_error(ERROR_NO_KEY);
      }
      receiver_state = RECEIVER_READY;
    }
  }

  if(receiver_state == RECEIVER_BLOCKED){
    if(dispatch_block()){
      send_ok();
      receiver_state = RECEIVER_READY;      
    }
  }

  if(!queue_is_empty()){ //start to execute on the queue
    if(next_block_execution() == EXECUTE_INTERPOLATOR){ //run if the interpolator has space
      if(!target_interpolator.queue_is_full()){
        struct block *target_block = pull_block();
        execute_block(target_block);
      }
    }else{ //run if the interpolator is idle
      if(target_interpolator.is_idle()){
        struct block *target_block = pull_block();
        execute_block(target_block);
      }
    }
  }
}

bool GCodeInterface::process_character(uint8_t character){
  if(character == '\n'){
    return true;
  }else{
    input_line_buffer[input_line_buffer_index] = character;
    input_line_buffer_index ++;
    return false;
  }
}

bool GCodeInterface::tokenize_block(){
  // processes the input line buffer into block tokens.
  
  //reset state
  int8_t token_index = -1;
  inbound_block.key_token_index = -1;
  uint8_t char_index = 0;
  bool token_lock = false; //if True, we'll stay in the current token until the lock is released.

  //iterate through the input line buffer
  for(uint8_t index = 0; index < input_line_buffer_index; index ++){
    char this_char = toupper(input_line_buffer[index]);
    if(this_char == ' '){ //skip spaces
      continue;
    }
    if(this_char == ';'){ //end line on a comment
      break;
    }

    if(this_char == '=' || this_char == '$'){ //we disable token lock when we receive an '=' or '$'
      token_lock = false;
    }

    if(strchr(TOKEN_LETTERS, this_char) && !token_lock){ //new token, if token lock is not enabled
      if(token_index > -1){ //close out the old token
        inbound_block.tokens[token_index].token_string[char_index] = '\0';
      }
      token_index ++;
      char_index = 0;
      if(token_index == MAX_NUM_TOKENS){ //stop processing when we reach the max num tokens.
        break;
      }
      if(this_char == 'G' || this_char == 'M' || this_char == '$'){ //identify the "key" token so we don't need to do it later.
        inbound_block.key_token_index = token_index;
      }
      if(this_char == '$'){
        token_lock = true; //we'll enter a token lock until we receive an '='
      }
    }
    inbound_block.tokens[token_index].token_string[char_index] = this_char;
    char_index++;
    if(char_index == MAX_TOKEN_SIZE){
      break;
    }
  }
  inbound_block.tokens[token_index].token_string[char_index] = '\0'; //close out the last token
  inbound_block.num_tokens = token_index + 1;
  if(token_index > -1 && inbound_block.key_token_index > -1){
    return true;
  }else{
    return false;
  }
}

int8_t GCodeInterface::find_code(char *code_string){
  uint8_t num_codes = sizeof(GCodeInterface::all_codes) / sizeof(GCodeInterface::all_codes[0]);
  for(uint8_t code_index = 0; code_index < num_codes; code_index++){ //iterate over all codes
    if(strcmp(code_string, all_codes[code_index].code_string) == 0){ //found the code
      return code_index;
    }
  }  
  return -1; //code not found 
}

bool GCodeInterface::preprocess_block(){
  int8_t code_index = find_code(inbound_block.tokens[inbound_block.key_token_index].token_string);
  if(code_index == -1){
    return false;
  }else{
    inbound_block.code_function = all_codes[code_index].code_function;
    inbound_block.execution = all_codes[code_index].execution;
    return true;
  }
}

bool GCodeInterface::dispatch_block(){
  if(inbound_block.execution == EXECUTE_NOW){ //execute now.
    execute_block(&inbound_block);
    return true;
  }else{
    if(queue_block()){  //put in queue
      return true;
    }else{
      return false;
    }
  }
}

bool GCodeInterface::queue_block(){
  if(block_queue_slots_remaining){ //OK to place block
    block *target_block = &block_queue[block_queue_write_index];
    memcpy(target_block, &inbound_block, sizeof(struct block));
    block_queue_slots_remaining --;
    advance_head(&block_queue_write_index);
    return true;
  }else{
    return false;
  }
}

struct GCodeInterface::block* GCodeInterface::pull_block(){
  struct block *return_block = &block_queue[block_queue_read_index];
  advance_head(&block_queue_read_index);
  block_queue_slots_remaining ++;
  return return_block;
}

void GCodeInterface::advance_head(uint8_t* target_head){
  (*target_head)++;
  if(*target_head == BLOCK_QUEUE_SIZE){
    *target_head = 0;
  }
}

void GCodeInterface::reset_block_queue(){
  block_queue_slots_remaining = BLOCK_QUEUE_SIZE;
  block_queue_read_index = 0;
  block_queue_write_index = 0;
}

void GCodeInterface::execute_block(block *target_block){
  load_tokens(target_block);
  (this->*target_block->code_function)(); //call code function in target block
}

void GCodeInterface::execute_realtime(char command){
  char str[2] = { command, '\0' };
  int8_t code_index = find_code(str);
  if(code_index > -1){
    (this->*all_codes[code_index].code_function)(); //run function
  }
}

void GCodeInterface::load_tokens(block *target_block){
  execution_tokens.clear();
  for(uint8_t token_index = 0; token_index < target_block->num_tokens; token_index ++){ //iterate over all tokens in block
    if(token_index != target_block->key_token_index){ //don't load the key token, b/c it may be non-numeric
      char letter = target_block->tokens[token_index].token_string[0];
      DecimalPosition value = std::strtod(&target_block->tokens[token_index].token_string[1], NULL);
      execution_tokens[String(letter)] = value;
    }
  }
}

bool GCodeInterface::queue_is_empty(){
  return !(block_queue_slots_remaining < BLOCK_QUEUE_SIZE);
}

uint8_t GCodeInterface::next_block_execution(){
  //returns the execution type of the next block
  return block_queue[block_queue_read_index].execution;
}

void GCodeInterface::send_ok(){
  gcode_stream->println("ok");
}

void GCodeInterface::send_error(uint8_t error_type){
  gcode_stream->print("error:");
  switch(error_type){
    case ERROR_NO_KEY:
      gcode_stream->println("1");
      break;
    case ERROR_BAD_FORMAT:
      gcode_stream->println("2");
      break;
    case ERROR_UNSUPPORTED_CODE:
      gcode_stream->println("20");
      break;
    case ERROR_NO_FEED_RATE:
      gcode_stream->println("22");
      break;      
  }
}

// --- GCODE FUNCTIONS ---

void GCodeInterface::g0_rapid(){
  auto result = execution_tokens.find("X"); //find X
  if(result != execution_tokens.end()){
    Serial.println(result->second);
  }
}
void GCodeInterface::g1_move(){
  struct TimeBasedInterpolator::motion_block interpolator_block;
  const char* AXES = "XYZE"; //need to be in same order as TimeBasedInterpolator::position
  const char* FEED_AXES = "XYZ";
  DecimalPosition sum_feed_delta_squared = 0; //used for calculating euclidean distance of "feed" axes (i.e. axes whose distance is used in feed rate calc.)
  DecimalPosition* current_position = &machine_position.x_mm; //pointer to first member of machine position
  DecimalPosition* delta_position = &interpolator_block.block_position.x_mm; //pointer to first member of block delta position

  // 1. Update feedrate
  auto feed_token = execution_tokens.find("F");
  if(feed_token != execution_tokens.end()){
    modal_feedrate_mm_per_min = feed_token->second;
  }
  if(modal_feedrate_mm_per_min <= 0){ //feed must be positive
    send_error(ERROR_NO_FEED_RATE);
    return;
  }

  // 2. Load all delta positions
  for(uint8_t axis_index =0; axis_index < sizeof(AXES); axis_index ++){
    auto result = execution_tokens.find(String(AXES[axis_index]));
    if(result != execution_tokens.end()){ //token was found
      delta_position[axis_index] = result->second - current_position[axis_index];
      current_position[axis_index] = result->second; //update current position
      if(strchr(FEED_AXES, AXES[axis_index])){ //feeding axis
        sum_feed_delta_squared += (delta_position[axis_index] * delta_position[axis_index]);
      }
    }else{
      delta_position[axis_index] = 0; //not moving in this axis
    }
  }

  // 3. Calculate move time
  float move_time_s = 0;
  if(sum_feed_delta_squared > 0){ //we're moving in the XYZ plane, so base move time on that.
    move_time_s = std::sqrt(sum_feed_delta_squared) / (modal_feedrate_mm_per_min / 60);
  }else if(interpolator_block.block_position.e_mm != 0){ //we're moving in E, base move time on that
    move_time_s = fabs(interpolator_block.block_position.e_mm) / (modal_feedrate_mm_per_min / 60);
  }
  interpolator_block.block_time_s = move_time_s;

  // 4. Load block
  if(move_time_s > 0){
    target_interpolator.add_block(&interpolator_block);
  }
};

void GCodeInterface::g4_dwell(){
  //need to implement
}

// -- SYSTEM FUNCTIONS --
void GCodeInterface::_help(){
  Serial.println("HELP");
}

void GCodeInterface::_report_parameters(){
  Serial.println("REPORT");
}

void GCodeInterface::_soft_reset(){

}

void GCodeInterface::_status_report(){
  gcode_stream->print("<");
  if(target_interpolator.is_idle()){
    gcode_stream->print("Idle|MPos:");
  }else{
    gcode_stream->print("Run|MPos:");
  }
  gcode_stream->print(target_interpolator.output_x.read_target(),3);
  gcode_stream->print(",");
  gcode_stream->print(target_interpolator.output_y.read_target(),3);
  gcode_stream->print(",");
  gcode_stream->print(target_interpolator.output_z.read_target(),3);
  gcode_stream->print(",|FS:");
  gcode_stream->print(modal_feedrate_mm_per_min, 1);
  gcode_stream->println(">");
}

void GCodeInterface::_parser_state(){
  gcode_stream->println("[GC:G0 G54 G17 G21 G90 G94 M0 M5 M9]"); //TEMPORARY. NEED TO UPDATE WITH ACTUAL STATE
}

void GCodeInterface::_cycle_start(){

}

void GCodeInterface::_feed_hold(){

}