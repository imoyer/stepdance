#include <sys/_stdint.h>
/*
Input Ports Module of the StepDance Control System

This module is responsible for reading input streams on the input ports, and directing them to
specific channels.

[More Details to be Added]

A part of the Mixing Metaphors Project
(c) 2025 Ilan Moyer, Jennifer Jacobs, Devon Frost, Emilie Yu
*/
#include "channels.hpp"
#include "imxrt.h"

#ifndef input_ports_h //prevent importing twice
#define input_ports_h

#define NUM_AVAILABLE_INPUT_PORTS    6 //four in new PCB designs, plus one legacy port in old PCBs. If changed, must adjust ISR functions in input_port
#define INPUT_A         0
#define INPUT_B         1
#define INPUT_C         2
#define INPUT_D         3
#define INPUT_B_LEGACY  4 // Port B on legacy DPW Modules
#define INPUT_C_LEGACY  5 // PORT C on legacy DPW Modules

#define FLEXPWM_CHANNEL_X   0
#define FLEXPWM_CHANNEL_A   1
#define FLEXPWM_CHANNEL_B   2

#define FLEXPWM_CLOCK_MHZ 150

#define SIGNAL_MIN_WIDTH_US 2 //standard input format

/** \cond */
  /**
   * This struct will be hidden from Doxygen documentation.
   */
struct input_port_info_struct{ //we use this structure to store hardware-specific information for each available port
    // Physical IO
  uint8_t STEP_TEENSY_PIN; //TEENSY Pin #s
  uint8_t DIR_TEENSY_PIN;

  // FlexPWM Module Setup
  uint8_t FLEXPWM_NUM; //FlexPWM number, e.g. 1 for FlexPWM1
  IMXRT_FLEXPWM_t *FLEXPWM; // pointer to FlexPWM Module
  uint8_t SUBMODULE; // submodule # 0-3
  uint8_t FLEXPWM_CHANNEL; // submodule channel, use definitions like FLEXPWM_CHANNEL_X
  uint8_t STEP_PIN_MUX; //pad mux value for flexpwm module
  volatile uint32_t *SELECT_INPUT_REGISTER; //pwm pad select register, for pins that can connect to multiple pads
  uint32_t SELECT_REGISTER_VALUE; //appropriate pad select register value
  uint8_t IRQ; // the IRQ number for the interrupt source
  void (*STATIC_ISR)(); // interrupt service routine. This needs to be a static function. We do some acrobatics to be able to call class methods as ISRs.
};
/** \endcond */

/**
 * @brief InputPort components receive motion streams on physical Stepdance input ports, and map these signals to downstream components. 
 * @ingroup io
 *
 * Receiving and decoding of input streams occurs asynchronously.
 * Here's an example of how to instantiate and configure an InputPort and map it to an OutputPort:
 * @snippet snippets.cpp InputPort
 */
class InputPort : public Plugin{
  public:
     /**
   * @brief Default constructor for InputPort.
   *
   * Initializes an InputPort instance with default, unconfigured state. This does not
   * configure hardware or register the port; call begin() to
   * initialize hardware-specific settings and register the port with the system.
   *
   */
    InputPort();
    /** 
     * @brief Initialize the InputPort with a port number corresponding to the target physical port on the Stepdance Board.
     * @param port_number Index of the physical input port (INPUT_A for all Basic Modules and INPUT_A through INPUT_D for Machine Controllers).
     */
    void begin(uint8_t port_number);
    /** 
     * @brief Enables all input signals (SIGNAL_X, SIGNAL_Y, SIGNAL_Z,  SIGNAL_E, SIGNAL_R, SIGNAL_T). Signals are enabled by default.
     */
    void enable_all_signals();
    /** 
     * @brief Disables all input signals (SIGNAL_X, SIGNAL_Y, SIGNAL_Z,  SIGNAL_E, SIGNAL_R, SIGNAL_T).
     */
    void disable_all_signals();
       /** 
     * @brief Enables a specific input signal.
     * @param signal_index Index of the signal to enable (SIGNAL_X, SIGNAL_Y, SIGNAL_Z,  SIGNAL_E, SIGNAL_R, SIGNAL_T).
     */
    void enable_signal(uint8_t signal_index);
       /** 
     * @brief Disables a specific input signal.
     * @param signal_index Index of the signal to disable (SIGNAL_X, SIGNAL_Y, SIGNAL_Z,  SIGNAL_E, SIGNAL_R, SIGNAL_T).
     */
    void disable_signal(uint8_t signal_index);
        /** 
      * @brief Sets the ratio between input units and output units for all input signals. Default is 1:1.
      * @param output_units Number of world units per input unit.
      * @param input_units Number of input units. Default is 1.
      */    
    void set_ratio(float output_units, float input_units = 1.0); 
   /** \cond */
  /**
   * This method and property will be hidden from Doxygen documentation.
   */
    void enroll(RPC *rpc, const String& instance_name);

    volatile uint32_t input_interrupt_cycles; //measures the number of cycles spent in each input interrupt routine.
/** \endcond */
    
    // BlockPorts
    /**
     * @brief BlockPort that acts as output for the SIGNAL_X input.
     * @code
InputPort inputA;
inputA.begin(INPUT_A); // Initialize InputPort A
inputA.output_x.map(channelX.input_target_position); // Map SIGNAL_X to Channel X's input target position
     * @endcode
     */
    BlockPort output_x; // 2us signal
    /**
     * @brief BlockPort that acts as output for the SIGNAL_Y input.
     */
    BlockPort output_y; // 3us signal
    /**
     * @brief BlockPort that acts as output for the SIGNAL_R input.
     */
    BlockPort output_r; // 4us signal
    /**
     * @brief BlockPort that acts as output for the SIGNAL_T input.
     */
    BlockPort output_t; // 5us signal
    /**
     * @brief BlockPort that acts as output for the SIGNAL_Z input.
     */
    BlockPort output_z; // 6us signal
    /**
     * @brief BlockPort that acts as output for the SIGNAL_E input.
     */
    BlockPort output_e; // 7us signal

  


  private:
    // Configuration Parameters
    uint8_t port_number; //the output port ID number
    static const struct input_port_info_struct port_info[]; //stores setup information for all four input ports
    static InputPort *indexed_input_ports[NUM_AVAILABLE_INPUT_PORTS]; //keeps pointers to all active input ports, indexed by their port number. Only used for ISR routines.

    IMXRT_FLEXPWM_t *FLEXPWM;
    uint8_t SUBMODULE;
    uint8_t SUBMODULE_BIT;
    uint8_t FLEXPWM_CHANNEL;
    // The following variables are used within the pulse detection and routing ISR, but we declare them here to save ISR run time.
    volatile uint16_t last_pulse_width_count; //important that this is UINT16_T to calculate rollover correctly.

    // State Parameters
    BlockPort *signal_BlockPort_targets[NUM_SIGNALS] = {&output_x, &output_y, &output_r, &output_t, &output_z, &output_e}; //pointers to BlockPorts, indexed by signal number
    bool signal_enable_flags[NUM_SIGNALS] = {true, true, true};

    // Private Methods
    void isr(); //this is the actual ISR function
    static void input_A_isr();
    static void input_B_isr();
    static void input_C_isr();
    static void input_D_isr();
    static void input_B_legacy_isr();
    static void input_C_legacy_isr();

     
    // Input Position Registers
    DecimalPosition position_x;
    DecimalPosition position_y;
    DecimalPosition position_r;
    DecimalPosition position_t;
    DecimalPosition position_z;
    DecimalPosition position_e;

  protected:
  /** \cond */
  /**
   * This function will be hidden from Doxygen documentation.
   */
    void run();
    /** \endcond */
};



#endif