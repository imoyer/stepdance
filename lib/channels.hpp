#include "arm_math.h"
#include <sys/_stdint.h>
#include "output_ports.hpp"
#include "core.hpp"

/*
Channels Module of the StepDance Control System

[More Details to be Added]

A part of the Mixing Metaphors Project
(c) 2025 Ilan Moyer, Jennifer Jacobs, Devon Frost, Emilie Yu
*/

#ifndef channels_h //prevent importing twice
#define channels_h

#define MAX_NUM_CHANNELS 10 //just to start with, until we need more.

void run_all_registered_channels(); //drives all registered channels to their target positions
void activate_channels(); //adds channels to the frame interrupt routine


/**
 * @brief Channels are modules that store the machine's positional state. 
 * @ingroup channels
 *
 * A Channel directly interfaces with an OutputPort by mapping to a particular signal flag (e.g. "X").Channels contain a target position variable that can be modified by upstream components such as motion generators. 
 * They also contain a variable representing their current position. Each interrupt frame, the channel increments or decrements the current position when needed to drive |target-current| < 0.5, and correspondingly sets step and direction flags on its mapped OutputPort. Any number of channels can be instantiated, often corresponding to axes of the machine machine. Basic Modules may contain multiple channels that map to distinct flags on the same OutputPort. Driver Modules typically have multiple channels that each map to a unique OutputPort, that then generate step/direction signals for a motor drive module.
 * Here's an example of how to instantiate and configure a Channel and map it to an OutputPort:
 * @snippet snippets.cpp Channel
 */

class Channel : public Plugin{
  public:
    // Public State
    /**
     * @brief Target position of the channel in pulses. Target positon is what the channel is driving toward.
     */
    DecimalPosition target_position; //primary target position, in pulses.
    /**
     * @brief Current position of the channel in pulses. Current position is where the channel is currently at.
     */
    DecimalPosition current_position; //tracks the current position, in pulses.
    /**
     * @brief Filtered target position of the channel in pulses. This is used when the enableFiltering() method is used to smooth out motion.
     */
    DecimalPosition filtered_target_position; // filtered target position
    /**
     * @brief Flag indicating whether filtering is enabled for the channel.
     */
    bool filtering_on = false;

/** \cond */
 /**
   * These properties will be hidden from Doxygen documentation.
   */
    DecimalPosition target_position_2; // secondary target position, used for coordinate transforms.
    float32_t num_averaging_samples = 20; //samples in the averaging window
    // BlockPorts
    BlockPort input_target_position;
    BlockPort input_target_position_2;
/** \endcond */

    // Public Methods
    Channel();
   /**
   * @brief Initialize the Channel with no output port. Not for normal use.

   */
    void begin(); //channel with no output port

    /**
   * @brief Initialize the Channel with a target OutputPort and output signal.
   * @param target_output_port Pointer to the target OutputPort.
   * @param output_signal Index of the output signal (SIGNAL_X, SIGNAL_Y, etc.).
   */
    void begin(OutputPort* target_output_port, uint8_t output_signal);
      /**
   * @brief Sets the maxium allowable pulse rate for the channel.
   * @param max_pulses_per_sec Maximum allowable pulse rate in pulses per second.
   */
    void set_max_pulse_rate(float max_pulses_per_sec); 
    /**
     * @brief Sets the transmission ratio for all target transmissions.
     * @param input_units Number of input units corresponding to channel_units.
     * @param channel_units Number of channel units corresponding to input_units. Default is 1.0.
     */
    void set_ratio(float input_units, float channel_units = 1.0); //sets the transmission ratio for all target transmissions
    /**
     * @brief Sets the upper position limit for the channel. Useful if you want the channel to stop transmitting motion streams at a certain upper threshold.
     * @param upper_limit_input_units Upper limit in input (world) units.
     */
    void set_upper_limit(DecimalPosition upper_limit_input_units); //sets the upper limit, using input (world) units.
        /**
     * @brief Sets the lower position limit for the channel. Useful if you want the channel to stop transmitting motion streams at a certain lower threshold.
     * @param lower_limit_input_units Lower limit in input (world) units.
     */
    void set_lower_limit(DecimalPosition lower_limit_input_units); //sets the lower limit, using input (world) units.
    /**
     * @brief Disables the upper position limit.
     */
    void disable_upper_limit();
    /**
     * @brief Disables the lower position limit.
     */
    void disable_lower_limit();
    /**
     * @brief Disables the channel.
     */
 /**
     * @brief Disables all position limits.
     */
      inline void disable_limits(){
      disable_upper_limit();
      disable_lower_limit();
    }
    /**
     * @brief Disables the channel.
     */
    void disable();
    /**
     * @brief Enables the channel.
     */
    void enable();
    /**
     * @brief Enables a moving average filter with a specified sample window. The bigger the window, the smoother the output but the less responsive the motion.
     * @param num_samples Number of samples for filtering. Default is 20.
     */
    void enable_filtering(uint16_t num_samples = 20);
    /**
     * @brief Disables the moving average filter.
     */
    void disable_filtering();
    /**  
     * @brief Checks if the current channel position is outside the set limits.
     * @return int8_t Returns 0 if inside limits, 1 if outside upper limit, or -1 if outside lower limit.
     */
    int8_t is_outside_limits();
    /**
     * @brief Inverts the channel output direction.
     */
     /**
     * @brief Inverts the channel output direction. Useful for reversing motor direction without changing wiring.
     */ 
    void invert_output();
    /**
     * @brief Inverts the channel output direction. Useful for reversing motor direction without changing wiring.
     * @param invert If true, inverts the output direction; if false, restores the normal direction.
     */ 
    void invert_output(bool invert);
  
   
/** \cond */
  /**
   * These functions and properties will be hidden from Doxygen documentation.
   */
   void enroll(RPC *rpc, const String& instance_name);     
   void run(); //Drives the current position to the target position by one pulse, and generates a signal
 /** \endcond */
  
  
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
    bool enabled = true;

    // Private Methods
    void initialize_state(); // initializes all state variables
    void register_channel(); // registers channel with the signal generator loop
    void pulse(int8_t direction); // generates a step pulse and releases a signal on the output port
 
};

#endif