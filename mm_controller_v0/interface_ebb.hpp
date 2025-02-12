#include "Stream.h"
#include "Arduino.h"
#include "core.hpp"
#include "interpreters.hpp"

#ifndef interface_ebb_h //prevent importing twice
#define interface_ebb_h


/*
EiBotBoard Interface Module of the StepDance Control System

This module provides an input interface that emulates the EiBotBoard, which allows standard AxiDraw workflows
to provide direct input. It is anticipated that this interface will feed a standard downstream motion generation synthesizer.

[More Details to be Added]

A part of the Mixing Metaphors Project
(c) 2025 Ilan Moyer, Jennifer Jacobs, Devon Frost
*/
#define EBB_COMMAND_SIZE  2 //the command portion of the input block is up to two characters long
#define EBB_MAX_NUM_INPUT_PARAMETERS 10 //maximum allowable number of parameters in an input string

class Eibotboard{
  public:
    Eibotboard();
    void begin(TimeBasedInterpreter* interpreter); // setup routine
    void loop(); // should be run inside loop
  
  private:
    // Serial Debug State
    uint8_t debug_port_identified; //1 if debug port has been ID'd, otherwise 0
    Stream *ebb_serial_port; //stores a pointer to the ebb serial port
    Stream *debug_serial_port; //pointer to the debug port

    // Interpreter
    TimeBasedInterpreter *target_interpreter;
    // Command Processing
    void process_character(uint8_t character);
    void reset_input_buffer(); //resets the input buffer state
    void process_command(uint16_t command_value);
    void process_string_int32(); //processes the block string (after the command word) into the input_parameters array.
    static void initialize_all_commands_struct(); //initializes the all_commands struct by pre-calculating the command values.

    struct command{
      char command_string[EBB_COMMAND_SIZE + 1]; //two-character string
      uint16_t command_value; //the command string, converted into a command value during initialization.
      void (Eibotboard::*command_function)(); //pointer to the command function to execute when this command value shows up.
    };
    char input_buffer[255]; //pre-allocate a string buffer to store the serial input stream.
    uint8_t input_buffer_write_index; //stores the current index of the write buffer
    uint8_t input_state; //tracks the current state of the input process
    uint16_t input_command_value; //tracks the value of the current command
    int32_t input_parameters[EBB_MAX_NUM_INPUT_PARAMETERS]; // parameters that have been parsed from the input string
    uint8_t num_input_parameters; // number of parameters in the string
    uint16_t block_id = 0; //stores the current block ID, which simply increments each time a new motion-containing block is received
    static struct command all_commands[]; //stores all available commands

    // -- COMMANDS --
    void command_query_current(); //'QC' returns the supply voltage and motor currents
    void command_query_button(); //'QB' returns whether the PRG button has been pressed
    void command_query_variable(); //'QL' returns the value of a variable stored in memory
    void command_stepper_move(); //'SM' moves the stepper motors
    void command_version(); //'V' returns the version of the EBB Board
    void command_generic(); //simply returns an OK
};

#endif