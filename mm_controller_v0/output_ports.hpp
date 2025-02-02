/*
Output Ports Module of the StepDance Control System

This module is responsible for outputting a pulse-width encoded stream of step and direction pulses on a
physical microcontroller pin.

[More Details to be Added]

A part of the Mixing Metaphors Project
(c) 2025 Ilan Moyer, Jennifer Jacobs, Devon Frost

*/

#ifndef output_ports_h //prevent importing twice
#define output_ports_h

struct output_port_info_struct{ //we use this structure to store hardware-specific information for each available port
  
  // Physical IO
  uint8_t STEP_ARDUINO_PIN; //Arduino Pin #s
  uint8_t DIR_ARDUINO_PIN;
  
  // Connecting FlexIO to Physical IO
  uint8_t STEP_FLEXIO_PIN; //FlexIO Module Pin #s (remember, IMXRT modules have pins, which attach to physical "pads" on the chip)
  uint8_t DIR_FLEXIO_PIN;
  volatile uint32_t *STEP_PIN_IOMUXC_REGISTER; //mux register pointer for the step pin
  volatile uint32_t *DIR_PIN_IOMUXC_REGISTER; //mux register pointer for the step pin
  uint8_t STEP_PIN_IOMUXC_VALUE; //the mux register value for the step pin
  uint8_t DIR_PIN_IOMUXC_VALUE; //the mux register value for the step pin

  // FlexIO Module Setup
  volatile uint32_t *STEP_SHIFTCTRL_REGISTER; //step shifter control register pointer
  volatile uint32_t *DIR_SHIFTCTRL_REGISTER; //step shifter control register pointer
  volatile uint32_t *STEP_SHIFTCFG_REGISTER; //step shifter config register pointer
  volatile uint32_t *DIR_SHIFTCFG_REGISTER; //step shifter config register pointer

  uint8_t TIMER_ID; //0-3 to identify the timer being used for this output port
  volatile uint32_t *TIMCMP_REGISTER; //timer compare register pointer, sets the output length and frequency
  volatile uint32_t *TIMCTL_REGISTER; //timer control register pointer
  volatile uint32_t *TIMCFG_REGISTER; //timer config register
};

class OutputPort{
  private:
    static const struct output_port_info_struct output_port_info[]; //stores setup information for all four output ports
};

#endif //output_ports_h