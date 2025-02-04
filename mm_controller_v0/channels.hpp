#include "output_ports.hpp"

/*
Channels Module of the StepDance Control System

[More Details to be Added]

A part of the Mixing Metaphors Project
(c) 2025 Ilan Moyer, Jennifer Jacobs, Devon Frost
*/

#ifndef channels_h //prevent importing twice
#define channels_h

class channel{
  public:
    // Public State
    volatile int32_t target_position; //primary target position, in pulses.
    volatile int32_t target_position_2; // secondary target position, used for coordinate transforms.
    volatile int32_t current_position; //tracks the current position, in pulses.

    // Public Methods
    channel();
    void begin(); //channel with no output port
    void begin(output_port* target_output_port, uint8_t output_signal); //channel with an output port
    void set_max_pulse_rate(float max_pulses_per_sec); // sets the maximum allowable pulse rate

  private:
    // Constants
    const uint32_t ACCUMULATOR_THRESHOLD = 1000000;
    const int PULSE_GEN_FRAME_PERIOD_US = 40; //microseconds. This yields a max output step rate of 25k steps/sec.
    const long PULSE_GEN_MAX_RATE = 1000000 / STEP_GEN_TICK_PERIOD_US;  

    // Configuration
    int has_output = 0; //1 if channel has an output port, otherwise 0.
    output_port* target_output_port; //stores the target output port
    uint8_t output_signal = SIGNAL_X; //default to signal X
    
    // Private State
    volatile int32_t accumulator;
    volatile int32_t accumulator_velocity;
    volatile int last_direction;

    // Private Methods
    void initialize_state(); // initializes all state variables
}

#endif