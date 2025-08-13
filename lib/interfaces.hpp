#include <stdint.h>
#include "Stream.h"
#include "Arduino.h"
#include "core.hpp"
#include "interpolators.hpp"

#ifndef interfaces_h //prevent importing twice
#define interfaces_h


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
#define EBB_EXECUTE_IMMEDIATE 0 //command to be executed on receipt, if no other command is pending
#define EBB_EXECUTE_EMERGENCY 1 //command to be executed on receipt, regardless of whether a command is pending
#define EBB_EXECUTE_TO_QUEUE 2 //command to be added to the queue
#define EBB_BLOCK_PENDING 1 //block is pending

#define EBB_SERVO_MAX_POSITION_STEPS 500 // +500us from neutral position (1500us pulse width)
#define EBB_SERVO_MIN_POSITION_STEPS -500 // -500us from neutral position (1500us pulse width)
#define EBB_SERVO_MIDPOINT_PULSE_DURATION_US  1500 //pulse duration for the servo midpoint.

class Eibotboard : public Plugin{
  public:
    Eibotboard();
    // void begin(TimeBasedInterpolator* interpolator); // setup routine
    void begin();
    void set_ratio_xy(float output_units_mm, float input_units_steps); //sets the xy conversion between steps and mm
    void set_ratio_z(float output_units_mm, float input_units_steps); //sets the z conversion between steps and mm

    BlockPort& output_x = target_interpolator.output_x;
    BlockPort& output_y = target_interpolator.output_y;
    BlockPort& output_z = target_interpolator.output_z;
    BlockPort& output_e = target_interpolator.output_e;
    BlockPort& output_r = target_interpolator.output_r;
    BlockPort& output_t = target_interpolator.output_t;

  protected:
    void loop(); // should be run inside loop
    
  private:
      // Interpolator
    TimeBasedInterpolator target_interpolator;

    // Serial Debug State
    uint8_t debug_port_identified; //1 if debug port has been ID'd, otherwise 0
    Stream *ebb_serial_port; //stores a pointer to the ebb serial port
    Stream *debug_serial_port; //pointer to the debug port

    float32_t xy_conversion_mm_per_step = 25.4 / 2874.0; //AxiDraw standard conversion
    float32_t z_conversion_mm_per_step = 1.0 / 50.0; //50 steps per mm of travel

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
      uint8_t execution; //0 -- immediate execution, 1 -- emergency execution, 2 -- add to queue
    };
    char input_buffer[255]; //pre-allocate a string buffer to store the serial input stream.
    uint8_t input_buffer_write_index; //stores the current index of the write buffer
    uint8_t input_state; //tracks the current state of the input process
    uint16_t input_command_value; //tracks the value of the current command
    int32_t input_parameters[EBB_MAX_NUM_INPUT_PARAMETERS]; // parameters that have been parsed from the input string
    uint8_t num_input_parameters; // number of parameters in the string

    // Block Generation
    uint16_t block_id = 0; //stores the current block ID, which simply increments each time a new motion-containing block is received
    TimeBasedInterpolator::motion_block pending_block; //stores motion block information that is pending being added to the queue
    uint8_t block_pending_flag = 0; //1 if a block is pending addition to the queue
    uint8_t debug_buffer_full_flag = 0; //1 if already sent a debug message
    void (Eibotboard::*pending_block_function)(); //pointer to the command function whose block is pending
    
    // Servo State
    int32_t servo_position_steps = 0; //tracks the current servo position. Unlike other moves, this one is provided in absolute coordinates.
    int32_t servo_pen_up_position_steps = 0; //absolute position of servo when the pen is up
    int32_t servo_pen_down_position_steps = 0; //absolute position of the servo when the pen is down
    float servo_rate_up_steps_per_sec = 100; //slew rate of the pen up in steps/sec
    float servo_rate_down_steps_per_sec = 100; //slew rate of the pen down in steps/sec

    // -- COMMANDS --
    static struct command all_commands[]; //stores all available commands
    void command_query_current(); //'QC' returns the supply voltage and motor currents
    void command_query_button(); //'QB' returns whether the PRG button has been pressed
    void command_query_variable(); //'QL' returns the value of a variable stored in memory
    void command_stepper_servo_configure(); //'SC' configures the stepper and servo motors
    void command_stepper_move(); //'SM' moves the stepper motors
    void command_set_pen(); //'SP' sets pen position
    void command_version(); //'V' returns the version of the EBB Board
    void command_generic(); //simply returns an OK

    // -- UTILITY FUNCTIONS --
    static void set_servo_position(uint16_t pulse_duration_83_3_ns, int32_t *servo_position_register); //sets a servo position register based on the pulse duration in increments of 83.3nS
    static void set_servo_rate(uint16_t pulse_rate_us_per_ms, float *servo_rate_register); //sets a servo rate register, see S2 command for units
    void debug_report_pending_block(bool waiting_for_slot); // outputs a report on the pending block to the debug port

};

#endif