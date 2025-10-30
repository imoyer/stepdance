#include "WString.h"
#include <sys/_stdint.h>
#include "arm_math.h"
#include <stdint.h>
#include "Stream.h"
#include "Arduino.h"
#include "core.hpp"
#include "interpolators.hpp"
#include <map>
#include <string>

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
    void command_query_pen(); //'QL' returns the pen state
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

class GCodeInterface : public Plugin{
  public:
    GCodeInterface();
    void begin(); //defaults to Serial as the input stream
    void begin(Stream *target_stream);
    void begin(usb_serial_class *target_usb_serial);
    void begin(HardwareSerialIMXRT *target_serial, uint32_t baud, uint16_t format = 0); //hardware serial

    //BlockPorts
    BlockPort& output_x = target_interpolator.output_x;
    BlockPort& output_y = target_interpolator.output_y;
    BlockPort& output_z = target_interpolator.output_z;
    BlockPort& output_e = target_interpolator.output_e;
    BlockPort& output_r = target_interpolator.output_r;
    BlockPort& output_t = target_interpolator.output_t;
  
  protected:
    void loop();
  
  private:

    // ---- COMMUNICATIONS ---
    Stream *gcode_stream; //serial port etc
    
    enum {
      RECEIVER_READY, //waiting for a new line to read
      RECEIVER_READING, //reading a line
      RECEIVER_PROCESSING, //processing a line
      RECEIVER_BLOCKED //a block is stuck in the inbound block buffer. Only process asynchronous control commands (e.g. ?)
    };

    uint8_t receiver_state = RECEIVER_READY;

    // 1. Receiving Block Lines
    //   We assume that each line contains at most one block, and that a newline represents the end of a block
    char input_line_buffer[255]; //pre-allocate a string buffer to store the serial input stream.
    uint8_t input_line_buffer_index; //stores the next position to write to in the input_line_buffer.
    void reset_input_line_buffer(); //resets the input line buffer.
    bool process_character(uint8_t character);

    // 2. Tokenize Block
    const char *GCODE_LETTERS = "GMXYZEABCSTHDFPN"; //a string containing all g-code letters
    static const uint8_t MAX_NUM_TOKENS = 10; //support up to 10 phrases in the incoming block
    static const uint8_t MAX_TOKEN_SIZE = 15; //max characters in a given token. For example, "E110292.6186" is 12 characters.
    struct token{ //stores a gcode phrase in the incoming block
      char token_string[MAX_TOKEN_SIZE + 1]; //up to 15 characters. 
    };

    struct block{
      struct token tokens[MAX_NUM_TOKENS]; //all tokens in the block
      int num_tokens = 0;
      int key_token_index = -1; //index of the "key" token (i.e. G or M) for the block.
      uint8_t execution; //context for execution (e.g. immediate, queue, interpolator)
      void (GCodeInterface::*code_function)(); //pointer to the command function to execute when this command value shows up.
    };

    struct block inbound_block; //stores an inbound block
    bool tokenize_block(); //places the input line buffer into inbound_block. Returns true if block has tokens and at least one is a key token.

    // 3. Pre-Process Block
    enum{
      EXECUTE_NOW, //runs the target function on receipt
      EXECUTE_QUEUE, //runs the target function in the block queue, when the interpolator is idle
      EXECUTE_INTERPOLATOR //runs the target function on the interpolator
    };

    struct code{
      char code_string[MAX_TOKEN_SIZE + 1]; //e.g. G0, G1, M82
      void (GCodeInterface::*code_function)(); //pointer to the command function to execute when this code shows up.
      uint8_t execution;
    };

    static struct code all_codes[]; //stores all available codes. This is defined in the .cpp file

    bool preprocess_block(); //determines the target function for the block, and the execution scope. Returns true if the block matches to an existant code
    int8_t find_code(char *code_string); //returns the index of the code in all_codes, or -1 if not found

    // 4. Dispatch
    bool dispatch_block(); //dispatches the inbound_block

    // 5. Queue
    static const uint8_t BLOCK_QUEUE_SIZE = 6;
    uint8_t block_queue_read_index = 0; //next block to read
    uint8_t block_queue_write_index = 0; //next block to write
    uint8_t block_queue_slots_remaining = BLOCK_QUEUE_SIZE;
    bool queue_block(); //queues the inbound block. Returns true if queue was successful
    bool queue_is_empty();
    struct block* pull_block();
    struct block block_queue[BLOCK_QUEUE_SIZE];
    void advance_head(uint8_t* target_head);
    void reset_block_queue();
    uint8_t next_block_execution();

    // 6. Execution
    std::map<String, DecimalPosition> execution_tokens;
    void execute_block(block *target_block);
    void load_tokens(block *target_block); //loads tokens into the execution_tokens map.

    // 7. Responses
    enum{
      ERROR_NO_KEY, //GRBL 1
      ERROR_BAD_FORMAT, //GRBL 2
      ERROR_UNSUPPORTED_CODE //GRBL 20
    };

    void send_ok();
    void send_error(uint8_t error_type);

    // GCode Commands
    void g0_rapid();
    void g1_move();

    // Interpolator
    TimeBasedInterpolator target_interpolator; 

    // Execution State
    struct TimeBasedInterpolator::position machine_position; //interpreter machine positional state
    float64_t modal_feedrate_mm_per_min = 0; // sets the current feedrate, which persists across blocks
};

#endif