#include <sys/_stdint.h>
#include "output_ports.hpp"
#include "core.hpp"

/*
Channels Module of the StepDance Control System

[More Details to be Added]

A part of the Mixing Metaphors Project
(c) 2025 Ilan Moyer, Jennifer Jacobs, Devon Frost
*/

#ifndef channels_h //prevent importing twice
#define channels_h

#define MAX_NUM_CHANNELS 10 //just to start with, until we need more.

void drive_all_registered_channels(); //drives all registered channels to their target positions
void activate_channels(); //adds channels to the frame interrupt routine

class channel{
  public:
    // Public State
    volatile IntegerPosition target_position; //primary target position, in pulses.
    volatile IntegerPosition target_position_2; // secondary target position, used for coordinate transforms.
    volatile IntegerPosition current_position; //tracks the current position, in pulses.

    // Public Methods
    channel();
    void begin(); //channel with no output port
    void begin(output_port* target_output_port, uint8_t output_signal); //channel with an output port
    void set_max_pulse_rate(float max_pulses_per_sec); // sets the maximum allowable pulse rate
    void drive_to_target(); //Drives the current position to the target position by one pulse, and generates a signal
    void pulse(int8_t direction); // generates a step pulse and releases a signal on the output port
  
  private:
    // Constants
    const uint32_t ACCUMULATOR_THRESHOLD = 1000000;
    const uint32_t PULSE_MAX_RATE = 1000000 / CORE_FRAME_PERIOD_US;

    // Configuration
    int has_output = 0; //1 if channel has an output port, otherwise 0.
    output_port* target_output_port; //stores the target output port
    uint8_t output_signal = SIGNAL_X; //default to signal X
    
    // Private State
    volatile float accumulator;
    volatile float accumulator_velocity;
    volatile int last_direction;

    // Private Methods
    void initialize_state(); // initializes all state variables
    void register_channel(); // registers channel with the signal generator loop
};

#endif