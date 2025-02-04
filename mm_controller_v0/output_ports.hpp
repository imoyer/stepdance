#include <sys/_stdint.h>
/*
Output Ports Module of the StepDance Control System

This module is responsible for outputting a pulse-width encoded stream of step and direction pulses on a
physical microcontroller pin.

[More Details to be Added]

A part of the Mixing Metaphors Project
(c) 2025 Ilan Moyer, Jennifer Jacobs, Devon Frost

*/

#ifndef output_ports_h //prevent importing twice
#define output_ports_h

struct output_port_info_struct{ //we use this structure to store hardware-specific information for each available port
  
  // Physical IO
  uint8_t STEP_TEENSY_PIN; //TEENSY Pin #s
  uint8_t DIR_TEENSY_PIN;
  
  // Connecting FlexIO to Physical IO
  uint8_t STEP_FLEXIO_PIN; //FlexIO Module Pin #s (remember, IMXRT modules have pins, which attach to physical "pads" on the chip)
  uint8_t DIR_FLEXIO_PIN;
  volatile uint32_t *STEP_PIN_IOMUXC_REGISTER; //mux register pointer for the step pin
  volatile uint32_t *DIR_PIN_IOMUXC_REGISTER; //mux register pointer for the step pin
  uint8_t STEP_PIN_IOMUXC_VALUE; //the mux register value for the step pin
  uint8_t DIR_PIN_IOMUXC_VALUE; //the mux register value for the step pin

  // FlexIO Module Setup
  volatile uint32_t *STEP_SHIFTCTRL_REGISTER; //step shifter control register pointer
  volatile uint32_t *DIR_SHIFTCTRL_REGISTER; //step shifter control register pointer
  volatile uint32_t *STEP_SHIFTCFG_REGISTER; //step shifter config register pointer
  volatile uint32_t *DIR_SHIFTCFG_REGISTER; //step shifter config register pointer
  volatile uint32_t *STEP_SHIFTBUF; //step shift output buffer
  volatile uint32_t *DIR_SHIFTBUF; //dir shift output buffer
  uint8_t TIMER_ID; //0-3 to identify the timer being used for this output port
  volatile uint32_t *TIMCMP_REGISTER; //timer compare register pointer, sets the output length and frequency
  volatile uint32_t *TIMCTL_REGISTER; //timer control register pointer
  volatile uint32_t *TIMCFG_REGISTER; //timer config register
};

#define OUTPUT_FORMAT_STEPDANCE 0 //outputting a pulse-length encoded step stream, which supports multiple signals over a single stream
#define OUTPUT_FORMAT_DRIVER    1 //outputting a standard stepper driver stream.

#define NUM_SIGNALS 6 //total number of signal types
#define SIGNAL_MIN_WIDTH_US 2 //pulse width in microseconds of shortest signal (index = 0)
#define SIGNAL_GAP_US 2 //gap in microseconds between signals

// Signal Indices
#define SIGNAL_X  0 //index of the X signal in the active_signal and signal_directions arrrays
#define SIGNAL_Y  1
#define SIGNAL_R  2 // Polar Radial Axis
#define SIGNAL_T  3 // Polar Theta Axis
#define SIGNAL_Z  4
#define SIGNAL_E  5 // Extruder

#define DIRECTION_FORWARD 1
#define DIRECTION_REVERSE 0

#define NUM_AVAILABLE_OUTPUT_PORTS 4 // max available output ports. NOTE: Should make this dynamic based on TEENSY version

// Main Output Port Class
class output_port{
  public:
    output_port();
    void begin(uint8_t port_number, uint8_t output_format); //complete initializer
    void begin(uint8_t port_number); // initialize using only port number
    void add_signal(uint8_t signal_index, uint8_t signal_direction); //adds a signal to the current active frame
    void transmit_frame(); //encodes and transmits the active frame

  private:
    // -- CONFIGURATION PARAMETERS --
    uint8_t port_number; //the output port ID number
    uint8_t output_format; //either standard or driver mode
    static const struct output_port_info_struct port_info[]; //stores setup information for all four output ports

    // -- STATE VARIABLES --
    volatile uint8_t active_signals[NUM_SIGNALS];
    volatile uint8_t active_signal_directions[NUM_SIGNALS];
    volatile uint32_t active_encoded_frame_step; //these get populated by the encode function
    volatile uint32_t active_encoded_frame_dir;

    // -- METHODS --
    void encode();// encodes the active_signal arrays into the active_encoded_frames
    void transmit(); //transmits the active encoded frame
    void clear_all_signals(); //clears all signals in the current active frame
    void register_output_port(); //registers the output port
};

void transmit_frames_on_all_output_ports(); //transmits across all output ports

#endif //output_ports_h