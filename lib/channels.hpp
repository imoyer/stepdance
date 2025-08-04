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

void run_all_registered_channels(); //drives all registered channels to their target positions
void activate_channels(); //adds channels to the frame interrupt routine

class Channel{
  public:
    // Public State
    DecimalPosition target_position; //primary target position, in pulses.
    DecimalPosition target_position_2; // secondary target position, used for coordinate transforms.
    DecimalPosition current_position; //tracks the current position, in pulses.

    // Public Methods
    Channel();
    void begin(); //channel with no output port
    void begin(OutputPort* target_output_port, uint8_t output_signal); //channel with an output port
    void set_max_pulse_rate(float max_pulses_per_sec); // sets the maximum allowable pulse rate
    void run(); //Drives the current position to the target position by one pulse, and generates a signal
    void pulse(int8_t direction); // generates a step pulse and releases a signal on the output port
    void set_ratio(float input_units, float channel_units = 1.0); //sets the transmission ratio for all target transmissions
    void set_upper_limit(DecimalPosition upper_limit_input_units); //sets the upper limit, using input (world) units.
    void set_lower_limit(DecimalPosition lower_limit_input_units); //sets the lower limit, using input (world) units.
    void disable_upper_limit();
    void disable_lower_limit();
    inline void disable_limits(){
      disable_upper_limit();
      disable_lower_limit();
    }
    int8_t is_outside_limits(); //returns 0 if inside limits, 1 if outside upper limit, or -1 if outside lower limit.
    void invert_output(); //inverts the channel output direction
    void invert_output(bool invert);

    // BlockPorts
    BlockPort input_target_position;
    BlockPort input_target_position_2;
  
  private:
    // Constants
    const uint32_t ACCUMULATOR_THRESHOLD = 1000000;
    const uint32_t PULSE_MAX_RATE = 1000000 / CORE_FRAME_PERIOD_US;

    // Configuration
    int has_output = 0; //1 if channel has an output port, otherwise 0.
    OutputPort* target_output_port; //stores the target output port
    uint8_t output_signal = SIGNAL_X; //default to signal X
    uint8_t output_inverted = 0; //if 1, will invert the output direction of the channel

    // Private State
    volatile float accumulator;
    volatile float accumulator_velocity;
    volatile int last_direction;
    DecimalPosition upper_limit;
    DecimalPosition lower_limit;
    bool upper_limit_enabled = false;
    bool lower_limit_enabled = false;

    // Private Methods
    void initialize_state(); // initializes all state variables
    void register_channel(); // registers channel with the signal generator loop
};

#endif