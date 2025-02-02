#include "imxrt.h"
#include "output_ports.hpp"

/*
Output Ports Module of the StepDance Control System

This module is responsible for outputting a pulse-width encoded stream of step and direction pulses on a
physical microcontroller pin.

[More Details to be Added]

A part of the Mixing Metaphors Project
(c) 2025 Ilan Moyer, Jennifer Jacobs, Devon Frost
*/

// ---- MCU-LEVEL OUTPUT PORT CONFIGURATION INFO ----
const struct output_port_info_struct OutputPort::output_port_info[] = {
  // This structure contains an indexed list of available output ports from 0-3.
  // Note that the specific pin and register assignments are chosen to maintain compatibility
  // for two outputs on a Teensy 4, and four outputs on a Teensy 4.1. This is because we intend to
  // use the Teensy 4.1 for connecting to stepper motor drivers using the same output ports.
  // For the Teensy 4, two output ports gives possibilities for splitting signals, and should be adequate for normal use.
  { .STEP_ARDUINO_PIN = 23, 
    .DIR_ARDUINO_PIN = 22,
    .STEP_FLEXIO_PIN = 9,
    .DIR_FLEXIO_PIN = 8,
    .STEP_PIN_IOMUXC_REGISTER = &IOMUXC_SW_MUX_CTL_PAD_GPIO_AD_B1_09,
    .DIR_PIN_IOMUXC_REGISTER = &IOMUXC_SW_MUX_CTL_PAD_GPIO_AD_B1_08, 
    .STEP_PIN_IOMUXC_VALUE = 9,
    .DIR_PIN_IOMUXC_VALUE = 9,
    .STEP_SHIFTCTRL_REGISTER = &FLEXIO3_SHIFTCTL0,
    .DIR_SHIFTCTRL_REGISTER = &FLEXIO3_SHIFTCTL1,  
    .STEP_SHIFTCFG_REGISTER = &FLEXIO3_SHIFTCFG0, 
    .DIR_SHIFTCFG_REGISTER = &FLEXIO3_SHIFTCFG1,
    .TIMER_ID = 0,
    .TIMCMP_REGISTER = &FLEXIO3_TIMCMP0,
    .TIMCTL_REGISTER = &FLEXIO3_TIMCTL0,
    .TIMCFG_REGISTER = &FLEXIO3_TIMCFG0
  },
  { .STEP_ARDUINO_PIN = 21, 
    .DIR_ARDUINO_PIN = 20,
    .STEP_FLEXIO_PIN = 11,
    .DIR_FLEXIO_PIN = 10,
    .STEP_PIN_IOMUXC_REGISTER = &IOMUXC_SW_MUX_CTL_PAD_GPIO_AD_B1_11,
    .DIR_PIN_IOMUXC_REGISTER = &IOMUXC_SW_MUX_CTL_PAD_GPIO_AD_B1_10, 
    .STEP_PIN_IOMUXC_VALUE = 9,
    .DIR_PIN_IOMUXC_VALUE = 9,
    .STEP_SHIFTCTRL_REGISTER = &FLEXIO3_SHIFTCTL2,
    .DIR_SHIFTCTRL_REGISTER = &FLEXIO3_SHIFTCTL3,  
    .STEP_SHIFTCFG_REGISTER = &FLEXIO3_SHIFTCFG2, 
    .DIR_SHIFTCFG_REGISTER = &FLEXIO3_SHIFTCFG3,
    .TIMER_ID = 1,
    .TIMCMP_REGISTER = &FLEXIO3_TIMCMP1,
    .TIMCTL_REGISTER = &FLEXIO3_TIMCTL1,
    .TIMCFG_REGISTER = &FLEXIO3_TIMCFG1
  },
  { .STEP_ARDUINO_PIN = 37, 
    .DIR_ARDUINO_PIN = 36,
    .STEP_FLEXIO_PIN = 19,
    .DIR_FLEXIO_PIN = 18,
    .STEP_PIN_IOMUXC_REGISTER = &IOMUXC_SW_MUX_CTL_PAD_GPIO_B1_03,
    .DIR_PIN_IOMUXC_REGISTER = &IOMUXC_SW_MUX_CTL_PAD_GPIO_B1_02,
    .STEP_PIN_IOMUXC_VALUE = 9,
    .DIR_PIN_IOMUXC_VALUE = 9,
    .STEP_SHIFTCTRL_REGISTER = &FLEXIO3_SHIFTCTL4,
    .DIR_SHIFTCTRL_REGISTER = &FLEXIO3_SHIFTCTL5,  
    .STEP_SHIFTCFG_REGISTER = &FLEXIO3_SHIFTCFG4, 
    .DIR_SHIFTCFG_REGISTER = &FLEXIO3_SHIFTCFG5,
    .TIMER_ID = 2,
    .TIMCMP_REGISTER = &FLEXIO3_TIMCMP2,
    .TIMCTL_REGISTER = &FLEXIO3_TIMCTL2,
    .TIMCFG_REGISTER = &FLEXIO3_TIMCFG2
  },
  { .STEP_ARDUINO_PIN = 35, 
    .DIR_ARDUINO_PIN = 34,
    .STEP_FLEXIO_PIN = 28,
    .DIR_FLEXIO_PIN = 29,
    .STEP_PIN_IOMUXC_REGISTER = &IOMUXC_SW_MUX_CTL_PAD_GPIO_B1_12,
    .DIR_PIN_IOMUXC_REGISTER = &IOMUXC_SW_MUX_CTL_PAD_GPIO_B1_13, 
    .STEP_PIN_IOMUXC_VALUE = 9,
    .DIR_PIN_IOMUXC_VALUE = 9,
    .STEP_SHIFTCTRL_REGISTER = &FLEXIO3_SHIFTCTL6,
    .DIR_SHIFTCTRL_REGISTER = &FLEXIO3_SHIFTCTL7,  
    .STEP_SHIFTCFG_REGISTER = &FLEXIO3_SHIFTCFG6, 
    .DIR_SHIFTCFG_REGISTER = &FLEXIO3_SHIFTCFG7,
    .TIMER_ID = 3,
    .TIMCMP_REGISTER = &FLEXIO3_TIMCMP3,
    .TIMCTL_REGISTER = &FLEXIO3_TIMCTL3,
    .TIMCFG_REGISTER = &FLEXIO3_TIMCFG3
  }
};