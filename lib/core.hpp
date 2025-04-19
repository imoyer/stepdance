#include <functional>
#include "arm_math.h"
#include "Arduino.h"
/*
Core Module of the StepDance Control System

This contains core system functions such as the frame interrupt timer.

A part of the Mixing Metaphors Project
(c) 2025 Ilan Moyer, Jennifer Jacobs, Devon Frost
*/
#ifndef core_h //prevent importing twice
#define core_h

typedef volatile float64_t DecimalPosition; //used to store positions across the system. We are using double-precision to allow incremental moves with acceptable error (~0.05 steps/day at 25khz)
typedef volatile int32_t IntegerPosition; //previously used to store positions
typedef volatile float32_t ControlParameter; //controls plugin parameters, typically from an analog input value

typedef void (*frame_function_pointer)(); //defines function pointers that can be called at each frame

#define CORE_FRAME_PERIOD_US 40 //microseconds. This yields a max output step rate of 25k steps/sec.
#define CORE_FRAME_PERIOD_S CORE_FRAME_PERIOD_US / 1000000 //duration of each frame in seconds
#define CORE_FRAME_FREQ_HZ 1000000 / CORE_FRAME_PERIOD_US //framerate in Hz. This is 25k
#define MAX_NUM_FRAME_FUNCTIONS 10 //maximum number of functions that can be called on the frame interrupt

#define KILOHERTZ_PLUGIN_PERIOD_US 1000 //microseconds, for the kilohertz plugin timer

// Signal Indices
// This is ordered by pulse length.
#define SIGNAL_X  0 //index of the X signal in the active_signal and signal_directions arrrays
#define SIGNAL_Y  1
#define SIGNAL_R  2 // Polar Radial Axis
#define SIGNAL_T  3 // Polar Theta Axis
#define SIGNAL_Z  4
#define SIGNAL_E  5 // Extruder

#define PLUGIN_FRAME_PRE_CHANNEL  0 //runs on the frame, before channels are evaluated
#define PLUGIN_FRAME_POST_CHANNEL 1 //runs on the frame, after channels are evaluated
#define PLUGIN_KILOHERTZ          2 //runs in an independent 1khz context

void add_function_to_frame(frame_function_pointer target_function);
void dance_start();

void stepdance_metrics_reset(); //resets the CPU usage metrics
float stepdance_get_cpu_usage(); //returns a value from 0-1 indicating the maximum CPU usage.
static volatile float stepdance_max_cpu_usage = 0; //stores a running count of the maximum CPU usage, in the range 0-1;
static volatile uint32_t stepdance_interrupt_entry_cycle_count = 0; //stores the entry value of ARM_DWT_CYCCNT

// -- Plug-In Base Class --
//
// This provides a common interface for any filters, synthesizers, kinematics, etc that need access to the core frame.
#define MAX_NUM_PRE_CHANNEL_FRAME_PLUGINS   10 //plugins that execute in the frame, before the channels are evaluated
#define MAX_NUM_POST_CHANNEL_FRAME_PLUGINS  10 //plugins that execute in the frame, after the channels are evaluated
#define MAX_NUM_KILOHERTZ_PLUGINS 10 //plugins that execute at a 1khz rate, independent of the frame, and with a lower priority

class Plugin{
  // Base class for all plugins that need to run in the core frame.
  public:
    Plugin();
    static void run_pre_channel_frame_plugins(); //runs all pre-channel frame plugins, in the order they appear in the registered_plugins list
    static void run_post_channel_frame_plugins(); //runs all post-channel frame plugins, in the order they appear in the registered_plugins list
    static void run_kilohertz_plugins(); //runs all post-channel frame plugins, in the order they appear in the registered_plugins list
  private:
    static Plugin* registered_pre_channel_frame_plugins[MAX_NUM_PRE_CHANNEL_FRAME_PLUGINS]; //stores all registered pre-channel frame plugins
    static Plugin* registered_post_channel_frame_plugins[MAX_NUM_POST_CHANNEL_FRAME_PLUGINS]; //stores all registered post-channel frame plugins
    static Plugin* registered_kilohertz_plugins[MAX_NUM_KILOHERTZ_PLUGINS]; //stores all registered kilohertz plugins
    static uint8_t num_registered_pre_channel_frame_plugins; //tracks the number of registered pre-channel frame plugins
    static uint8_t num_registered_post_channel_frame_plugins; //tracks the number of registered post-channel frame plugins
    static uint8_t num_registered_kilohertz_plugins; //tracks the number of registered kilohertz plugins

  protected: //these need to be accessed from derived classes
    void register_plugin(); //registers the plugin
    void register_plugin(uint8_t execution_target); //registers the plugin
    virtual void run(); //this should be overridden in the derived class. Runs each frame.
};

// -- Transmission Class --
// Transmissions are a one-dimensional means of converting units from one domain (e.g. steps) to another (e.g. mm).

class Transmission{
  public:
    Transmission();
    void begin();
    void begin(DecimalPosition *output_position);
    void begin(float input_units, float output_units); //configures to convert from input to output
    void begin(float input_units, float output_units, DecimalPosition *output_position); //enables interacting with the target output position in mechanism units
    void set_ratio(float input_units, float output_units); //sets the transmission ratio
    void set(float64_t input_value); //set the output position
    void increment(float64_t input_value);
    float64_t get(); //gets the output position
    float64_t convert(float64_t input_value); //converts from input to output values
    float64_t convert_reverse(float64_t output_value); //converts from output to input values
    std::function<DecimalPosition()> get_function = nullptr; //points to a function that will override built-in get(). This is typically used within kinematics.
  
  private:
    float64_t transfer_ratio = 1.0; // output_units/input_units. Initialize with a unity transfer ratio
    DecimalPosition *target;
};

// -- LOOP FUNCTION AND CLASSES --
// These allow non-blocking functions to be called within the loop, at an approximately given frequency.
void dance_loop();

static volatile float stepdance_loop_time_ms; //tracks the time spent in the last loop
static volatile uint32_t stepdance_loop_entry_cycle_count = 0; //cycle count when dance_loop was last called

class LoopDelay{
  public:
    LoopDelay();
    void periodic_call(void (*callback_function)(), float interval_ms);
  
  private:
    float time_since_last_call_ms; //stores time since the function was last called
};

#endif