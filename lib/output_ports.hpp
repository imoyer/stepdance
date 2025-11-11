#include "arm_math.h"
#include <sys/_stdint.h>
#include "analog_in.hpp"
#include "string.h"
#include "configuration.hpp"

/*
Output Ports Module of the StepDance Control System

This module is responsible for outputting a pulse-width encoded stream of step and direction pulses on a
physical microcontroller pin.

[More Details to be Added]

A part of the Mixing Metaphors Project
(c) 2025 Ilan Moyer, Jennifer Jacobs, Devon Frost

*/

#ifndef output_ports_h // prevent importing twice
#define output_ports_h
/** \cond */
/**
 * This function will be hidden from Doxygen documentation.
 */
struct output_port_info_struct
{ // we use this structure to store hardware-specific information for each available port
  // Port Name (for reporting)
  char PORT_NAME[10];

  // Physical IO
  uint8_t STEP_TEENSY_PIN; // TEENSY Pin #s
  uint8_t DIR_TEENSY_PIN;

  // Connecting FlexIO to Physical IO
  uint8_t STEP_FLEXIO_PIN; // FlexIO Module Pin #s (remember, IMXRT modules have pins, which attach to physical "pads" on the chip)
  uint8_t DIR_FLEXIO_PIN;
  volatile uint32_t *STEP_PIN_IOMUXC_REGISTER; // mux register pointer for the step pin
  volatile uint32_t *DIR_PIN_IOMUXC_REGISTER;  // mux register pointer for the step pin
  uint8_t STEP_PIN_IOMUXC_VALUE;               // the mux register value for the step pin
  uint8_t DIR_PIN_IOMUXC_VALUE;                // the mux register value for the step pin

  // FlexIO Module Setup
  volatile uint32_t *STEP_SHIFTCTRL_REGISTER; // step shifter control register pointer
  volatile uint32_t *DIR_SHIFTCTRL_REGISTER;  // step shifter control register pointer
  volatile uint32_t *STEP_SHIFTCFG_REGISTER;  // step shifter config register pointer
  volatile uint32_t *DIR_SHIFTCFG_REGISTER;   // step shifter config register pointer
  volatile uint32_t *STEP_SHIFTBUF;           // step shift output buffer
  volatile uint32_t *DIR_SHIFTBUF;            // dir shift output buffer
  uint8_t TIMER_ID;                           // 0-3 to identify the timer being used for this output port
  volatile uint32_t *TIMCMP_REGISTER;         // timer compare register pointer, sets the output length and frequency
  volatile uint32_t *TIMCTL_REGISTER;         // timer control register pointer
  volatile uint32_t *TIMCFG_REGISTER;         // timer config register
  uint8_t VREF_TEENSY_PIN;                    // current vref pin on teensy
  uint8_t ENABLE_TEENSY_PIN;                  // enable pin on teensy
  uint8_t LIMIT_TEENSY_PIN;                   // limit switch pin on teensy
};

struct output_format_struct
{
  uint8_t FRAME_LENGTH_US;          // length of the frame
  uint8_t STEP_PULSE_START_TIME_US; // time from start of frame to start of the step pulse
  uint8_t DIR_PULSE_START_TIME_US;  // time from start of frame to start of the dir pulse
  uint8_t SIGNAL_MIN_WIDTH_US;      // pulse width in microseconds of shortest signal (index = 0)
  uint8_t SIGNAL_GAP_US;            // gap in microseconds between signals
  uint8_t RATE_SHIFT;               // 2^N bits per microsecond
};

// #define OUTPUT_FORMAT_STEPDANCE 0 //outputting a pulse-length encoded step stream, which supports multiple signals over a single stream
// #define OUTPUT_FORMAT_DRIVER    1 //outputting a standard stepper driver stream.

// OUTPUT SPEED SETTINGS
#define OUTPUT_FRAME_32US 0 // frame is 32us long. This is the standard stepdance output frame, for a 25KHz framerate.
#define OUTPUT_FRAME_16US 1 // 16us frame, supports up to 50KHz framerate
#define OUTPUT_FRAME_8US 2  // 8us frame, supports up to 100KHz framerate
#define OUTPUT_FRAME_4US 3  // 4us frame, supports up to 200KHz framerate

// OUTPUT TRIGGERING
#define OUTPUT_TRANSMIT_ON_FRAME 1 // transmits each stepdance frame
#define OUTPUT_TRANSMIT_MANUAL 2   // transmits only with manual call to transmit();

#define OUTPUT_A 0
#define OUTPUT_B 1
#define OUTPUT_C 2
#define OUTPUT_D 3
#define OUTPUT_A_LEGACY 0
#define OUTPUT_B_LEGACY 1

#define NUM_SIGNALS 6 // total number of signal types

#define DIRECTION_FORWARD 1
#define DIRECTION_REVERSE 0

#define NUM_AVAILABLE_OUTPUT_PORTS 4 // max available output ports. NOTE: Should make this dynamic based on TEENSY version
/** \endcond */

//!  OutputPort
/*!
  OutputPorts are modules that convert internal step commands into a frame of pulse signals on the physical output port on the **machine controller module** or **basic module**. Output ports contain step and direction signal flags for each of the six output signal types (X, Y, Z, E, R, and T).
 * Here's an example of how to instantiate and configure an OutputPort:
 * @snippet snippets.cpp OutputPort
 */

// Main Output Port Class

class OutputPort : public Plugin{
   /**
   * @brief Default constructor for OutputPort. Initializes an OutputPort instance with default, unconfigured state. This does not configure hardware or register the port; call begin() to initialize hardware-specific settings and register the port with the system.
   */
  public:
    OutputPort();
    // -- STANDARD PORT FUNCTIONS --
/**
   * @brief Initialize the OutputPort with a port number corresponding to the target physical port on the Stepdance Board.
   * @param port_number Index of the physical output port (OUTPUT_A, OUTPUT_B, OUTPUT_C, OUTPUT_D).
   */
    void begin(uint8_t port_number); // initialize using only port number

/** \cond */
  /**
   * This function will be hidden from Doxygen documentation.
   */
    void begin(uint8_t port_number, uint8_t output_format, uint8_t transmit_mode); //complete initializer
    
    void add_signal(uint8_t signal_index, uint8_t signal_direction); //adds a signal to the current active frame
    void transmit_frame(); //encodes and transmits the active frame
    void step_now(uint8_t direction); //shortcut to immediately output a step at the minimum signal size
    void step_now(uint8_t direction, uint8_t signal_index);
    
    // -- DRIVER FUNCTIONS --
    float32_t read_drive_current_amps(); // returns the last drive current reading
    float32_t read_drive_current_amps(float32_t drive_current_gain_amps_per_volt); // sets the gain and then returns the last drive current reading
    void set_drive_current_gain(float32_t amps_per_volt); // sets the drive current gain, in amps/volt
    void enable_driver(); //enables the motor driver
    void disable_driver(); //disables the motor driver
    bool read_limit_switch(); //returns the current value of the limit switch

    char port_name[10]; //stores the name of the port
    void enroll(RPC *rpc, const String& instance_name);
  /** \endcond */

private:
  // -- CONFIGURATION PARAMETERS --
  uint8_t port_number;                                       // the output port ID number
  uint8_t format_index;                                      // an index into the output_formats struct array.
  uint8_t transmit_mode;                                     // transmit on frame, or transmit manually
  uint8_t FRAME_LENGTH_US;                                   // length of the frame
  uint8_t STEP_PULSE_START_TIME_US;                          // time from start of frame to start of the step pulse
  uint8_t DIR_PULSE_START_TIME_US;                           // time from start of frame to start of the dir pulse
  uint8_t SIGNAL_MIN_WIDTH_US;                               // pulse width in microseconds of shortest signal (index = 0)
  uint8_t SIGNAL_GAP_US;                                     // gap in microseconds between signals
  uint8_t RATE_SHIFT;                                        // 2^N bits per microsecond
  static const struct output_port_info_struct port_info[];   // stores setup information for all four output ports
  static const struct output_format_struct output_formats[]; // output formats for different frame sizes

  // -- STATE VARIABLES --
  volatile uint8_t active_signals[NUM_SIGNALS];
  volatile uint8_t active_signal_directions[NUM_SIGNALS];
  volatile uint32_t active_encoded_frame_step; // these get populated by the encode function
  volatile uint32_t active_encoded_frame_dir;
  volatile float32_t last_drive_current_reading_amps;

  // DRIVER CONFIG AND STATE
  // for reading driver-related peripherals
  // storing this state allows us to configure on-the-fly when relevant functions get called.
  volatile bool vref_is_configured = false;
  volatile bool enable_is_configured = false;
  volatile bool limitsw_is_configured = false;
  float32_t drive_current_gain_amps_per_volt = 0; // stores the drive current gain setting.

  // -- METHODS --
  void encode();               // encodes the active_signal arrays into the active_encoded_frames
  void transmit();             // transmits the active encoded frame
  void clear_all_signals();    // clears all signals in the current active frame
  void register_output_port(); // registers the output port

  // -- ADC --
  AnalogInput analog_vref;
};

void transmit_frames_on_all_output_ports(); // transmits across all output ports

void iterate_across_all_output_ports(void (*target_function)(OutputPort *)); // allows user code to iterate across all output ports

void enable_drivers();  // enables drivers on all registered output ports
void disable_drivers(); // enables drivers on all registered output ports

#endif // output_ports_h

/**
 * @example analog_in_test/analog_in_test.ino An example demonstrating the OutputPort functionality.
 */