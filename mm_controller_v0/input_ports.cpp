#include <sys/_stdint.h>
#include "pins_arduino.h"
#include "core_pins.h"
#include "imxrt.h"
/*
Input Ports Module of the StepDance Control System

This module is responsible for reading input streams on the input ports, and directing them to
specific channels.

[More Details to be Added]

A part of the Mixing Metaphors Project
(c) 2025 Ilan Moyer, Jennifer Jacobs, Devon Frost
*/

#include "input_ports.hpp"

//--- ISR ACROBATICS ---
// ISR functions need to be static, rather than class methods. So we define a unique static method for each of the
// input ports, and then have _those_ call the bound class method isr().

InputPort * InputPort::indexed_input_ports[NUM_AVAILABLE_INPUT_PORTS] = {nullptr, nullptr, nullptr};

void InputPort::input_A_isr(){indexed_input_ports[INPUT_A]->isr();};
void InputPort::input_B_isr(){indexed_input_ports[INPUT_B]->isr();};
void InputPort::input_C_isr(){indexed_input_ports[INPUT_C]->isr();};
void InputPort::input_D_isr(){indexed_input_ports[INPUT_D]->isr();};
void InputPort::input_B_legacy_isr(){indexed_input_ports[INPUT_B_LEGACY]->isr();};
void InputPort::input_C_legacy_isr(){indexed_input_ports[INPUT_C_LEGACY]->isr();};

// ---- MCU-LEVEL INPUT PORT CONFIGURATION INFO ----
const struct input_port_info_struct InputPort::port_info[] = {
  // This structure contains an indexed list of available input ports.
  // Besides the four input ports available on the new StepDance modules, we also provide legacy configurations
  // for input Ports B and C on the original wheelprint modules. THESE LEGACY CONFIGURATIONS ARE MUTUALLY EXCLUSIVE
  // WITH THE NON-LEGACY CONFIGS, AND ALL INPUT PORTS SHOULD EITHER BE STANDARD OR NON-LEGACY. The reason for this is
  // that the original pinouts do not support four channels using the PWM timers as inputs.
  //
  // NOTE: WE CURRENTLY DON'T SUPPORT FLEXPWM CHANNEL X!
  // None of the pin assignments currently rely on this channel.
  { .STEP_TEENSY_PIN = 4, // INPUT_A
    .DIR_TEENSY_PIN = 6,
    .FLEXPWM_NUM = 2,
    .FLEXPWM = &IMXRT_FLEXPWM2,
    .SUBMODULE = 0,
    .FLEXPWM_CHANNEL = FLEXPWM_CHANNEL_A,
    .STEP_PIN_MUX = (1 | 0x10), //EMC_06 ALT1
    .SELECT_INPUT_REGISTER = &IOMUXC_FLEXPWM2_PWMA0_SELECT_INPUT,
    .SELECT_REGISTER_VALUE = 0,
    .IRQ = IRQ_FLEXPWM2_0,
    .STATIC_ISR = &InputPort::input_A_isr
  },
  { .STEP_TEENSY_PIN = 5, // INPUT_B
    .DIR_TEENSY_PIN = 7,
    .FLEXPWM_NUM = 2,
    .FLEXPWM = &IMXRT_FLEXPWM2,
    .SUBMODULE = 1,
    .FLEXPWM_CHANNEL = FLEXPWM_CHANNEL_A,
    .STEP_PIN_MUX = (1 | 0x10), //EMC_08 ALT1
    .SELECT_INPUT_REGISTER = &IOMUXC_FLEXPWM2_PWMA1_SELECT_INPUT,
    .SELECT_REGISTER_VALUE = 0,
    .IRQ = IRQ_FLEXPWM2_1,
    .STATIC_ISR = &InputPort::input_B_isr
  },
  { .STEP_TEENSY_PIN = 8, // INPUT_C
    .DIR_TEENSY_PIN = 10,
    .FLEXPWM_NUM = 1,
    .FLEXPWM = &IMXRT_FLEXPWM1,
    .SUBMODULE = 3,
    .FLEXPWM_CHANNEL = FLEXPWM_CHANNEL_A,
    .STEP_PIN_MUX = (6 | 0x10), //B1_00 ALT6
    .SELECT_INPUT_REGISTER = &IOMUXC_FLEXPWM1_PWMA3_SELECT_INPUT,
    .SELECT_REGISTER_VALUE = 4,
    .IRQ = IRQ_FLEXPWM1_3,
    .STATIC_ISR = &InputPort::input_C_isr
  },
  { .STEP_TEENSY_PIN = 9, // INPUT_D
    .DIR_TEENSY_PIN = 11,
    .FLEXPWM_NUM = 2,
    .FLEXPWM = &IMXRT_FLEXPWM2,
    .SUBMODULE = 2,
    .FLEXPWM_CHANNEL = FLEXPWM_CHANNEL_B,
    .STEP_PIN_MUX = (2 | 0x10), //B0_11 ALT2
    .SELECT_INPUT_REGISTER = &IOMUXC_FLEXPWM2_PWMB2_SELECT_INPUT,
    .SELECT_REGISTER_VALUE = 1,
    .IRQ = IRQ_FLEXPWM2_2,
    .STATIC_ISR = &InputPort::input_D_isr
  },
  { .STEP_TEENSY_PIN = 5, // INPUT_B_LEGACY
    .DIR_TEENSY_PIN = 4,
    .FLEXPWM_NUM = 2,
    .FLEXPWM = &IMXRT_FLEXPWM2,
    .SUBMODULE = 1,
    .FLEXPWM_CHANNEL = FLEXPWM_CHANNEL_A,
    .STEP_PIN_MUX = (1 | 0x10), //EMC_08 ALT1
    .SELECT_INPUT_REGISTER = &IOMUXC_FLEXPWM2_PWMA1_SELECT_INPUT,
    .SELECT_REGISTER_VALUE = 0,
    .IRQ =IRQ_FLEXPWM2_1,
    .STATIC_ISR = &InputPort::input_B_legacy_isr
  },
  { .STEP_TEENSY_PIN = 7, // INPUT_C_LEGACY
    .DIR_TEENSY_PIN = 6,
    .FLEXPWM_NUM = 1,
    .FLEXPWM = &IMXRT_FLEXPWM1,
    .SUBMODULE = 3,
    .FLEXPWM_CHANNEL = FLEXPWM_CHANNEL_B,
    .STEP_PIN_MUX = (6 | 0x10), //B1_01 ALT6
    .SELECT_INPUT_REGISTER = &IOMUXC_FLEXPWM1_PWMB3_SELECT_INPUT,
    .SELECT_REGISTER_VALUE = 4,
    .IRQ = IRQ_FLEXPWM1_3,
    .STATIC_ISR = &InputPort::input_C_legacy_isr
  },
};

// -- INPUT PORT CLASS FUNCTIONS --
InputPort::InputPort(){};

void InputPort::begin(uint8_t port_number, DecimalPosition* x_signal_target, DecimalPosition* y_signal_target, DecimalPosition* z_signal_target, DecimalPosition* e_signal_target, DecimalPosition* r_signal_target, DecimalPosition* t_signal_target){
  // store port number
  this->port_number = port_number;

  // Set up target channels
  signal_position_targets[SIGNAL_X] = x_signal_target;
  signal_position_targets[SIGNAL_Y] = y_signal_target;
  signal_position_targets[SIGNAL_R] = r_signal_target;
  signal_position_targets[SIGNAL_T] = t_signal_target;
  signal_position_targets[SIGNAL_Z] = z_signal_target;
  signal_position_targets[SIGNAL_E] = e_signal_target;
  enable_all_signals();

  // configure teensy pins
  pinMode(port_info[port_number].STEP_TEENSY_PIN, INPUT);
  pinMode(port_info[port_number].DIR_TEENSY_PIN, INPUT);

  // FLEXPWM CONFIGURATION
  // Enable Clock
  switch(port_info[port_number].FLEXPWM_NUM){
    case 1:
      CCM_CCGR4 |= CCM_CCGR4_PWM1(CCM_CCGR_ON);
      break;
    case 2:
      CCM_CCGR4 |= CCM_CCGR4_PWM2(CCM_CCGR_ON);
      break;
    case 3:
      CCM_CCGR4 |= CCM_CCGR4_PWM3(CCM_CCGR_ON);
      break;
    case 4:
      CCM_CCGR4 |= CCM_CCGR4_PWM4(CCM_CCGR_ON);
      break;
  }

  // Pad Mux
  *(portConfigRegister(port_info[port_number].STEP_TEENSY_PIN)) = port_info[port_number].STEP_PIN_MUX;

  // Select Pin
  if(port_info[port_number].SELECT_INPUT_REGISTER){
    *port_info[port_number].SELECT_INPUT_REGISTER = port_info[port_number].SELECT_REGISTER_VALUE;
  } 

  // FlexPWM Register Configuration
  // This section adapted from https://github.com/PaulStoffregen/FreqMeasureMulti
  FLEXPWM = port_info[port_number].FLEXPWM;
  SUBMODULE = port_info[port_number].SUBMODULE;
  SUBMODULE_BIT = 1 << SUBMODULE;
  FLEXPWM_CHANNEL = port_info[port_number].FLEXPWM_CHANNEL;

  FLEXPWM->FCTRL0 |= FLEXPWM_FCTRL0_FLVL(SUBMODULE_BIT);
	FLEXPWM->FSTS0 = SUBMODULE_BIT;
	FLEXPWM->MCTRL |= FLEXPWM_MCTRL_CLDOK(SUBMODULE_BIT);
	FLEXPWM->SM[SUBMODULE].CTRL2 = FLEXPWM_SMCTRL2_INDEP;
	FLEXPWM->SM[SUBMODULE].CTRL = FLEXPWM_SMCTRL_HALF;
	FLEXPWM->SM[SUBMODULE].INIT = 0;
	FLEXPWM->SM[SUBMODULE].VAL0 = 0;
	FLEXPWM->SM[SUBMODULE].VAL1 = 65535; //used to capture overflow, this is the X register
	FLEXPWM->SM[SUBMODULE].VAL2 = 0;
	FLEXPWM->SM[SUBMODULE].VAL3 = 0;
	FLEXPWM->SM[SUBMODULE].VAL4 = 0;
	FLEXPWM->SM[SUBMODULE].VAL5 = 0;
	FLEXPWM->MCTRL |= FLEXPWM_MCTRL_LDOK(SUBMODULE_BIT) | FLEXPWM_MCTRL_RUN(SUBMODULE_BIT);

  // Configure the capture functionality
  uint16_t capture_mode;
  switch(FLEXPWM_CHANNEL){
    // case FLEXPWM_CHANNEL_X:
    //   capture_mode = FLEXPWM_SMCAPTCTRLX_EDGX0(2) | FLEXPWM_SMCAPTCTRLX_EDGX1(1); //capture time between rising and falling edges
    //   FLEXPWM->SM[SUBMODULE].CAPTCTRLX = capture_mode | FLEXPWM_SMCAPTCTRLX_ARMX;
    //   break;
    case FLEXPWM_CHANNEL_A:
      capture_mode = FLEXPWM_SMCAPTCTRLA_EDGA0(2) | FLEXPWM_SMCAPTCTRLA_EDGA1(1); //capture time between rising and falling edges
      FLEXPWM->SM[SUBMODULE].CAPTCTRLA = capture_mode | FLEXPWM_SMCAPTCTRLA_ARMA;
      break;
    case FLEXPWM_CHANNEL_B:
      capture_mode = FLEXPWM_SMCAPTCTRLB_EDGB0(2) | FLEXPWM_SMCAPTCTRLB_EDGB1(1); //capture time between rising and falling edges
      FLEXPWM->SM[SUBMODULE].CAPTCTRLB = capture_mode | FLEXPWM_SMCAPTCTRLB_ARMB;
      break;
  }

  // Configure and Enable Interrupts
  uint16_t interrupt_enable_flags;
  switch(FLEXPWM_CHANNEL){
    // case FLEXPWM_CHANNEL_X:
    //   interrupt_enable_flags = FLEXPWM_SMINTEN_CX1IE; //interrupt on second capture
    //   FLEXPWM->SM[SUBMODULE].INTEN |= interrupt_enable_flags;
    //   break;
    case FLEXPWM_CHANNEL_A:
      interrupt_enable_flags = FLEXPWM_SMINTEN_CA1IE; //interrupt on second capture
      FLEXPWM->SM[SUBMODULE].INTEN |= interrupt_enable_flags;
      break;
    case FLEXPWM_CHANNEL_B:
      interrupt_enable_flags = FLEXPWM_SMINTEN_CB1IE; //interrupt on second capture
      FLEXPWM->SM[SUBMODULE].INTEN |= interrupt_enable_flags;
      break;
  }

  // Attach interrupts
  __disable_irq(); //disable interrupts
  indexed_input_ports[port_number] = this; //register this input port
  attachInterruptVector((IRQ_NUMBER_t) port_info[port_number].IRQ, port_info[port_number].STATIC_ISR);
  NVIC_SET_PRIORITY(port_info[port_number].IRQ, 48); //we give it a high priority (low number), so we don't miss anything!
  NVIC_ENABLE_IRQ(port_info[port_number].IRQ);
  __enable_irq(); //re-enable interrupts
}

void InputPort::isr(){
  // uint32_t interrupt_entry_cycle_count = ARM_DWT_CYCCNT; //UNCOMMENT FOR INSTRUMENTATION
  // read the direction pin
  int8_t dir = digitalReadFast(port_info[port_number].DIR_TEENSY_PIN);
  if(dir == 0){
    dir = -1;
  }


  // clear the interrupt flag (otherwise it hangs) and store pulse duration.
  switch(FLEXPWM_CHANNEL){
    // case FLEXPWM_CHANNEL_X:
    //   FLEXPWM->SM[SUBMODULE].STS = FLEXPWM_SMSTS_CFX1;
    //   break;
    case FLEXPWM_CHANNEL_A:
      FLEXPWM->SM[SUBMODULE].STS = FLEXPWM_SMSTS_CFA1;
      last_pulse_width_count = FLEXPWM->SM[SUBMODULE].CVAL3 - FLEXPWM->SM[SUBMODULE].CVAL2;
      break;
    case FLEXPWM_CHANNEL_B:
      FLEXPWM->SM[SUBMODULE].STS = FLEXPWM_SMSTS_CFB1;
      last_pulse_width_count = FLEXPWM->SM[SUBMODULE].CVAL5 - FLEXPWM->SM[SUBMODULE].CVAL4;
      break;
  }
  // -- Route signals --
  // Calculate nearest whole pulse width
  uint16_t last_pulse_width_whole_us = last_pulse_width_count / FLEXPWM_CLOCK_MHZ;
  uint8_t last_pulse_width_remainder_count = last_pulse_width_count % FLEXPWM_CLOCK_MHZ;
  if(last_pulse_width_remainder_count > (FLEXPWM_CLOCK_MHZ / 2)){
    last_pulse_width_whole_us ++;
  }
  // Convert into signal index
  uint8_t last_signal_index = last_pulse_width_whole_us - SIGNAL_MIN_WIDTH_US;

  if((last_signal_index >= SIGNAL_X) && (last_signal_index <= SIGNAL_E)){ // check if signal index within range
    if(signal_enable_flags[last_signal_index]){ // check if signal is enabled
      *signal_position_targets[last_signal_index] += dir; // increment or decrement based on direction
    }
  }
  // input_interrupt_cycles = ARM_DWT_CYCCNT - interrupt_entry_cycle_count;
}

void InputPort::enable_signal(uint8_t signal_index){
  signal_enable_flags[signal_index] = INPUT_ENABLED;
}

void InputPort::disable_signal(uint8_t signal_index){
  signal_enable_flags[signal_index] = INPUT_DISABLED;
}

void InputPort::enable_all_signals(){
  for(uint8_t signal_index = 0; signal_index < NUM_SIGNALS; signal_index++){
    // only enable signals that have a target channel
    if(signal_position_targets[signal_index] != nullptr){
      signal_enable_flags[signal_index] = INPUT_ENABLED;
    }else{
      signal_enable_flags[signal_index] = INPUT_DISABLED;
    }
  }
}

void InputPort::disable_all_signals(){
  for(uint8_t signal_index = 0; signal_index < NUM_SIGNALS; signal_index++){
    signal_enable_flags[signal_index] = INPUT_DISABLED;
  }
}