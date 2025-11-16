#include "WString.h"
#include <sys/_stdint.h>
#include <cstddef>
#include <stdint.h>
#include <functional>
#include "arm_math.h"
#include "Arduino.h"
/*
Core Module of the StepDance Control System

This contains core system functions such as the frame interrupt timer.

A part of the Mixing Metaphors Project
(c) 2025 Ilan Moyer, Jennifer Jacobs, Devon Frost, Emilie Yu
*/
#ifndef core_h //prevent importing twice
#define core_h

class RPC; //forward declaration of RPC from rpc.hpp

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

// Standard Step Ratio
#define STANDARD_RATIO_MM  0.01 // world mm / steps. At 25KHz this provides a max velocity of 250mm/sec.
#define STANDARD_RATIO_IN  0.0003937 // world inches / step

// Plugin Execution Context
#define PLUGIN_FRAME_PRE_CHANNEL  0 //runs on the frame, before channels are evaluated
#define PLUGIN_FRAME_POST_CHANNEL 1 //runs on the frame, after channels are evaluated
#define PLUGIN_KILOHERTZ          2 //runs in an independent 1khz context
#define PLUGIN_LOOP               3 //runs in the main loop
#define PLUGIN_INPUT_PORT         4 //runs on the frame, at the start before all other plugins

// Position Mode
// Throughout stepdance, there is a question of whether to operate incrementally or in absolute coordinates.
// By default, we operate incrementally. But some modules require data to be in absolute values (e.g. non-linear functions)

enum{
  INCREMENTAL, //indicates a position is being provided incrementally
  ABSOLUTE, //position should be interpreted as an absolute position, relative to the local state of the component
  GLOBAL //position should be interpreted as absolute, relative to the global (e.g. channel) state of the machine
};
// #define INCREMENTAL   0 //data is handled incrementally
// #define ABSOLUTE      1 //data is handled in absolute values

#define MIN   0
#define MAX   1

enum{
  BLOCKPORT_INPUT, //blockport is an input
  BLOCKPORT_OUTPUT, //blockport is an output
  BLOCKPORT_UNDEFINED //blockport is undefined
};

void add_function_to_frame(frame_function_pointer target_function);
void dance_start();

void stepdance_metrics_reset(); //resets the CPU usage metrics
float stepdance_get_cpu_usage(); //returns a value from 0-1 indicating the maximum CPU usage.
static volatile float stepdance_max_cpu_usage = 0; //stores a running count of the maximum CPU usage, in the range 0-1;
static volatile uint32_t stepdance_interrupt_entry_cycle_count = 0; //stores the entry value of ARM_DWT_CYCCNT

// -- Plug-In Base Class --
//
// This provides a common interface for any filters, synthesizers, kinematics, etc that need access to the core frame.
#define MAX_NUM_INPUT_PORT_FRAME_PLUGINS   10 //plugins that execute at the start of the frame. This is reserved for input ports
#define MAX_NUM_PRE_CHANNEL_FRAME_PLUGINS   10 //plugins that execute in the frame, before the channels are evaluated
#define MAX_NUM_POST_CHANNEL_FRAME_PLUGINS  10 //plugins that execute in the frame, after the channels are evaluated
#define MAX_NUM_KILOHERTZ_PLUGINS 10 //plugins that execute at a 1khz rate, independent of the frame, and with a lower priority
#define MAX_NUM_LOOP_PLUGINS 10 //plugins that execute in the main loop.
/** \cond */
/**
 * Plugin Base Class will be hidden from Doxygen documentation.
 */
class Plugin{
  // Base class for all plugins that need to run in the core frame.
  public:
    Plugin();
    static void run_input_port_frame_plugins(); //runs all input port frame plugins.
    static void run_pre_channel_frame_plugins(); //runs all pre-channel frame plugins, in the order they appear in the registered_plugins list
    static void run_post_channel_frame_plugins(); //runs all post-channel frame plugins, in the order they appear in the registered_plugins list
    static void run_kilohertz_plugins(); //runs all post-channel frame plugins, in the order they appear in the registered_plugins list
    static void run_loop_plugins(); //runs all loop plugins, in the order they appear in the registered_plugins list

    virtual void enable();
    virtual void disable();
    virtual void enroll(RPC *rpc, const String& instance_name); //enrolls the plugin in an RPC. This should be overridden by the derived class, and is responsible for enrolling any members.
    virtual void push_deep(); //deep push across the plugin (e.g. from input to output blockports) for state sync.
    virtual void pull_deep(); //performs a deep pull across the plugin (e.g. from output to input blockports) for state sync

  private:
    static Plugin* registered_input_port_frame_plugins[MAX_NUM_INPUT_PORT_FRAME_PLUGINS]; //stores all registered input port plugins
    static Plugin* registered_pre_channel_frame_plugins[MAX_NUM_PRE_CHANNEL_FRAME_PLUGINS]; //stores all registered pre-channel frame plugins
    static Plugin* registered_post_channel_frame_plugins[MAX_NUM_POST_CHANNEL_FRAME_PLUGINS]; //stores all registered post-channel frame plugins
    static Plugin* registered_kilohertz_plugins[MAX_NUM_KILOHERTZ_PLUGINS]; //stores all registered kilohertz plugins
    static Plugin* registered_loop_plugins[MAX_NUM_LOOP_PLUGINS]; //stores all registered loop plugins
    static uint8_t num_registered_input_port_frame_plugins; //tracks the number of registered input port frame plugins
    static uint8_t num_registered_pre_channel_frame_plugins; //tracks the number of registered pre-channel frame plugins
    static uint8_t num_registered_post_channel_frame_plugins; //tracks the number of registered post-channel frame plugins
    static uint8_t num_registered_kilohertz_plugins; //tracks the number of registered kilohertz plugins
    static uint8_t num_registered_loop_plugins; //tracks the number of registered loop plugins

  protected: //these need to be accessed from derived classes
    void register_plugin(); //registers the plugin
    void register_plugin(uint8_t execution_target); //registers the plugin
    virtual void run(); //this should be overridden in the derived class. Runs each frame.
    virtual void loop(); //this can be overridden in the derived class. Runs in the main loop context.
};
/** \endcond */

// -- BlockPort --
// BlockPorts provide a unified interface into and out of component blocks (i.e. "blocks")
// These are designed to be flexibly used depending on the component to which they belong.

/**
 * @brief BlockPorts provide a unified interface for mapping inputs and outputs of different StepDance components (E.g. Channels, Kinematics, Generators, etc)
 * @ingroup core
 *
 * @details BlockPorts manage push and pull of data between components automatically through the map() method, which when called assigns a target to the BlockPort. Each BlockPort has an internal position value. On each interrupt frame a component reads its input BlockPorts' position, performs internal calculations specific to the component, and then stores the result in its output BlockPorts' position. The component then calls BlockPort.push() for each output BlockPort which transmits the position value to the mapped target BlockPort. However BlockPorts can only have one target of their map function. To get around this you can also have BlockPort Inputs pull from multiple other BlockPorts by mapping an input BlockPort to either an input or an output. The Component will first call BlockPort.pull() on its input BlockPorts to retrieve the latest values from all mapped BlockPorts, then perform its internal calculations, and finally call BlockPort.push() on its output BlockPorts to transmit the results.
 *
 * @image html blockport_diagram.png "Typical mapping patterns of BlockPorts. Components (A-E) each have one BlockPort Input and one BlockPort Output. Multiple BlockPort Outputs can map to a single Input (e.g. A.output and C.output are both mapped to B.input). Inputs can map to multiple inputs or outputs (e.g. D.input is mapped to C.output)" width=400px
 *
 * You will never instantiate a BlockPort directly; instead, you can access the BlockPorts that are members of different components. The example below shows how to access and map an InputPort's BlockPort to a Channel's input target position BlockPort.
 * @snippet snippets.cpp BlockPort
 */

// Main BlockPort Class
class BlockPort{
  public:
    /**
     * @brief Default constructor for BlockPort. Initializes a BlockPort instance. You do not need to call this directly; BlockPorts are typically members of other components and are automatically initialized when the component is initialized.
     */
    BlockPort();

    // -- User Functions -- these are called within user code, not (just) the library
   /** 
    * @brief Sets the ratio between world units and block units for this BlockPort for automatic conversion. Default is 1:1. For example if ou want the BlockPort to transmit data in millimeters corresponding to a single step, you would set the ratio to 0.01 world units to 1 block_units (i.e. 1 step = 0.01mm).
    * @param world_units Number of world units.
    * @param block_units Number of block units. Default is 1.
    */
    void set_ratio(float world_units, float block_units = 1.0);  // sets the ratio between world and block units, for automatic conversion. Default is 1.
                                                                // conversion always happens within the write/read functions when data enters and exits the BlockPort
    /** 
     * @brief Maps this BlockPort to a target BlockPort with a specified mode (INCREMENTAL or ABSOLUTE).
     * @param map_target Pointer to the target BlockPort to map to.
     * @param mode Mode of operation: INCREMENTAL or ABSOLUTE.
     */
    void map(BlockPort *map_target, uint8_t mode); //maps this BlockPort's pipe to a target BlockPort
    inline void map(BlockPort *map_target){
      map(map_target, INCREMENTAL); //default internal mode is INCREMENTAL
    }
    
    // -- External Functions -- these are called outside the block that contains this BlockPort
/**
 * @brief Returns the position value of the BlockPort in world units. Reading in INCREMENTAL mode returns the change to the position since the last read or update; reading in ABSOLUTE mode returns the absolute position value.
 * @param mode Mode of operation: INCREMENTAL or ABSOLUTE.
 */
    float64_t read(uint8_t mode); // externally reads the BlockPort's target via the BlockPort buffers.
                                  // an incremental read will reflect the change to the target, either pending or after the block has run.
                                  // an absolute read will reflect the upcoming or last state of the target, depending on whether the block has run.
                                  // In some cases this function can be overridden by a custom read function.
/**
 * @brief Returns the absolute position value of the BlockPort in world units.
 */
    float64_t read_absolute(); // returns the absolute value of the BlockPort's target in world units.
                               // This is provided for simplified access via RPC.

/** \cond */
  /**
   * These functions will be hidden from Doxygen documentation.
   */
    void write(float64_t value, uint8_t mode); // writes to the BlockPort's absolute or incremental buffers

    void write_now(float64_t); //writes directly to the target. REMEMBER TO UPDATE ABSOLUTE_BUFFER AT SAME TIME.
    float64_t read_target(); //reads directly from the target
/** \endcond */

    // -- Internal Functions -- called by the block containing this BlockPort
/** \cond */
  /**
   * These functions will be hidden from Doxygen documentation.
   */
    void begin(volatile float64_t *target, uint8_t direction = BLOCKPORT_UNDEFINED, Plugin *parent = nullptr); //initializes the BlockPort
    void set_target(volatile float64_t *target); //sets a target variable for the BlockPort
    void update(); //called by the block, to update the target and the buffers. Note that this does not handle pulling or pushing, which must be done first or after update.
    void reverse_update(); //updates the buffers based on changes made by direct writes to the target. Used by input_ports, which run before all other blocks.
    void set(float64_t value, uint8_t mode); //sets a new value for the target.
    inline void set(float64_t value){ //default for set is ABSOLUTE
      set(value, ABSOLUTE);
    };
    void reset(float64_t value, bool raw = false); //resets the target, and updates buffers to reflect new value WITHOUT an incremental update.

    void push(uint8_t mode); // pushes this BlockPort's buffer state to a target.
    inline void push(){
      push(this->mode); //uses internal mode
    }

    void pull(uint8_t mode); // pulls a target BlockPort's buffer state into this BlockPort's buffers.
    inline void pull(){
      pull(this->mode); //uses internal mode
    }

    // State Synchronization Functions
    // Internal
    void push_deep(DecimalPosition abs_value); //Pushes an ABSOLUTE value across a mapping chain.
    DecimalPosition pull_deep(); //pulls an ABSOLUTE value thru from the terminal of the mapping chain.

    // User Facing
    inline void reset_deep(DecimalPosition abs_value){
      push_deep(abs_value);
    }
    inline DecimalPosition read_deep(){
      return pull_deep();
    }

    void enable(); // enables push/pull on blockport
    void disable(); // disables push/pull

    volatile float64_t incremental_buffer = 0;
    volatile float64_t absolute_buffer = 0; //contains a new value if absolute_buffer_is_written, otherwise the last value of the associated variable.

    inline float64_t convert_block_to_world_units(float64_t block_units){
      return block_units * world_to_block_ratio;
    }
    inline float64_t convert_world_to_block_units(float64_t world_units){
      return world_units / world_to_block_ratio;
    }

    volatile float64_t* target = nullptr;

    void enroll(RPC *rpc, const String& instance_name); //used to enroll the blockport in an RPC
/** \endcond */
  private:
    volatile bool update_has_run = false; //set to true when an update has run, and false when write() is called.
    uint8_t mode = INCREMENTAL; //default mode used by push and pull, unless specified in that function call. This is set by the map function.
    volatile uint8_t push_pull_enabled = true; //controlled by enable() and disable(). This enables/disables push and pull. NOTE: We could optimize by removing volatile,
                                          // but then this couldn't be operated inside any interrupts incl. the kilohertz interrupt, which could be confusing.
                                          // Can re-examine if we start running out of compute overhead.
    float64_t world_to_block_ratio = 1;

    BlockPort* target_BlockPort = nullptr;
    Plugin* parent_Plugin = nullptr; //This should only be set on INPUTS.
    uint8_t blockport_direction = BLOCKPORT_UNDEFINED; //direction is not explicitly set by the parent Plugin
};


// -- LOOP FUNCTION AND CLASSES --
// These allow non-blocking functions to be called within the loop, at an approximately given frequency.

/** \cond */
  /**
   * These functions and properties will be hidden from Doxygen documentation.
   */   
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

/** \endcond */
#endif