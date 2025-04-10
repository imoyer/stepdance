#include <sys/_stdint.h>
/*
Input Ports Module of the StepDance Control System

This module is responsible for reading input streams on the input ports, and directing them to
specific channels.

[More Details to be Added]

A part of the Mixing Metaphors Project
(c) 2025 Ilan Moyer, Jennifer Jacobs, Devon Frost
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

#define INPUT_DISABLED  0
#define INPUT_ENABLED   1

#define FLEXPWM_CLOCK_MHZ 150

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

class InputPort{
  public:
    InputPort();
    void begin(uint8_t port_number);
    void begin(uint8_t port_number, DecimalPosition* x_signal_target, DecimalPosition* y_signal_target, DecimalPosition* z_signal_target, DecimalPosition* e_signal_target, DecimalPosition* r_signal_target, DecimalPosition* t_signal_target);
    void map(uint8_t signal_index, DecimalPosition* signal_target); //maps a signal to a target DecimalPosition
    void enable_all_signals();
    void disable_all_signals();
    void enable_signal(uint8_t signal_index);
    void disable_signal(uint8_t signal_index);
    volatile uint32_t input_interrupt_cycles; //measures the number of cycles spent in each input interrupt routine.

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
    DecimalPosition *signal_position_targets[NUM_SIGNALS]; //pointers to target positions for each signal, e.g. a passthru might do X -> channel_x.target_position, Y-> channel_y
    volatile uint8_t signal_enable_flags[NUM_SIGNALS]; // for each signal, 1 == enabled, and 0 == disabled

    // Private Methods
    void isr(); //this is the actual ISR function
    static void input_A_isr();
    static void input_B_isr();
    static void input_C_isr();
    static void input_D_isr();
    static void input_B_legacy_isr();
    static void input_C_legacy_isr();
};



#endif