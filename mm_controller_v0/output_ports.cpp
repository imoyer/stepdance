#include <sys/_stdint.h>
#include "core_pins.h"
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

// ---- STATE VARIABLES ---

// Registration
output_port* registered_output_ports[NUM_AVAILABLE_OUTPUT_PORTS]; //stores all registered ports
uint8_t num_registered_output_ports = 0; //tracks the number of registered output ports 

// ---- MCU-LEVEL OUTPUT PORT CONFIGURATION INFO ----
const struct output_port_info_struct output_port::port_info[] = {
  // This structure contains an indexed list of available output ports from 0-3.
  // Note that the specific pin and register assignments are chosen to maintain compatibility
  // for two outputs on a Teensy 4, and four outputs on a Teensy 4.1. This is because we intend to
  // use the Teensy 4.1 for connecting to stepper motor drivers using the same output ports.
  // For the Teensy 4, two output ports gives possibilities for splitting signals, and should be adequate for normal use.
  { .STEP_TEENSY_PIN = 23, // OUTPUT PORT 0
    .DIR_TEENSY_PIN = 22,
    .STEP_FLEXIO_PIN = 9,
    .DIR_FLEXIO_PIN = 8,
    .STEP_PIN_IOMUXC_REGISTER = &IOMUXC_SW_MUX_CTL_PAD_GPIO_AD_B1_09,
    .DIR_PIN_IOMUXC_REGISTER = &IOMUXC_SW_MUX_CTL_PAD_GPIO_AD_B1_08, 
    .STEP_PIN_IOMUXC_VALUE = 9,
    .DIR_PIN_IOMUXC_VALUE = 9,
    .STEP_SHIFTCTRL_REGISTER = &FLEXIO3_SHIFTCTL0,
    .DIR_SHIFTCTRL_REGISTER = &FLEXIO3_SHIFTCTL4,  
    .STEP_SHIFTCFG_REGISTER = &FLEXIO3_SHIFTCFG0, 
    .DIR_SHIFTCFG_REGISTER = &FLEXIO3_SHIFTCFG4,
    .STEP_SHIFTBUF = &FLEXIO3_SHIFTBUF0,
    .DIR_SHIFTBUF = &FLEXIO3_SHIFTBUF4,
    .TIMER_ID = 0,
    .TIMCMP_REGISTER = &FLEXIO3_TIMCMP0,
    .TIMCTL_REGISTER = &FLEXIO3_TIMCTL0,
    .TIMCFG_REGISTER = &FLEXIO3_TIMCFG0
  },
  { .STEP_TEENSY_PIN = 21, // OUTPUT PORT 1
    .DIR_TEENSY_PIN = 20,
    .STEP_FLEXIO_PIN = 11,
    .DIR_FLEXIO_PIN = 10,
    .STEP_PIN_IOMUXC_REGISTER = &IOMUXC_SW_MUX_CTL_PAD_GPIO_AD_B1_11,
    .DIR_PIN_IOMUXC_REGISTER = &IOMUXC_SW_MUX_CTL_PAD_GPIO_AD_B1_10, 
    .STEP_PIN_IOMUXC_VALUE = 9,
    .DIR_PIN_IOMUXC_VALUE = 9,
    .STEP_SHIFTCTRL_REGISTER = &FLEXIO3_SHIFTCTL1,
    .DIR_SHIFTCTRL_REGISTER = &FLEXIO3_SHIFTCTL5,  
    .STEP_SHIFTCFG_REGISTER = &FLEXIO3_SHIFTCFG1, 
    .DIR_SHIFTCFG_REGISTER = &FLEXIO3_SHIFTCFG5,
    .STEP_SHIFTBUF = &FLEXIO3_SHIFTBUF1,
    .DIR_SHIFTBUF = &FLEXIO3_SHIFTBUF5,
    .TIMER_ID = 1,
    .TIMCMP_REGISTER = &FLEXIO3_TIMCMP1,
    .TIMCTL_REGISTER = &FLEXIO3_TIMCTL1,
    .TIMCFG_REGISTER = &FLEXIO3_TIMCFG1
  },
  { .STEP_TEENSY_PIN = 37, // OUTPUT PORT 2
    .DIR_TEENSY_PIN = 36,
    .STEP_FLEXIO_PIN = 19,
    .DIR_FLEXIO_PIN = 18,
    .STEP_PIN_IOMUXC_REGISTER = &IOMUXC_SW_MUX_CTL_PAD_GPIO_B1_03,
    .DIR_PIN_IOMUXC_REGISTER = &IOMUXC_SW_MUX_CTL_PAD_GPIO_B1_02,
    .STEP_PIN_IOMUXC_VALUE = 9,
    .DIR_PIN_IOMUXC_VALUE = 9,
    .STEP_SHIFTCTRL_REGISTER = &FLEXIO3_SHIFTCTL2,
    .DIR_SHIFTCTRL_REGISTER = &FLEXIO3_SHIFTCTL6,  
    .STEP_SHIFTCFG_REGISTER = &FLEXIO3_SHIFTCFG2, 
    .DIR_SHIFTCFG_REGISTER = &FLEXIO3_SHIFTCFG6,
    .STEP_SHIFTBUF = &FLEXIO3_SHIFTBUF2,
    .DIR_SHIFTBUF = &FLEXIO3_SHIFTBUF6,
    .TIMER_ID = 2,
    .TIMCMP_REGISTER = &FLEXIO3_TIMCMP2,
    .TIMCTL_REGISTER = &FLEXIO3_TIMCTL2,
    .TIMCFG_REGISTER = &FLEXIO3_TIMCFG2
  },
  { .STEP_TEENSY_PIN = 35, // OUTPUT PORT 3
    .DIR_TEENSY_PIN = 34,
    .STEP_FLEXIO_PIN = 28,
    .DIR_FLEXIO_PIN = 29,
    .STEP_PIN_IOMUXC_REGISTER = &IOMUXC_SW_MUX_CTL_PAD_GPIO_B1_12,
    .DIR_PIN_IOMUXC_REGISTER = &IOMUXC_SW_MUX_CTL_PAD_GPIO_B1_13, 
    .STEP_PIN_IOMUXC_VALUE = 9,
    .DIR_PIN_IOMUXC_VALUE = 9,
    .STEP_SHIFTCTRL_REGISTER = &FLEXIO3_SHIFTCTL3,
    .DIR_SHIFTCTRL_REGISTER = &FLEXIO3_SHIFTCTL7,  
    .STEP_SHIFTCFG_REGISTER = &FLEXIO3_SHIFTCFG3, 
    .DIR_SHIFTCFG_REGISTER = &FLEXIO3_SHIFTCFG7,
    .STEP_SHIFTBUF = &FLEXIO3_SHIFTBUF3,
    .DIR_SHIFTBUF = &FLEXIO3_SHIFTBUF7,
    .TIMER_ID = 3,
    .TIMCMP_REGISTER = &FLEXIO3_TIMCMP3,
    .TIMCTL_REGISTER = &FLEXIO3_TIMCTL3,
    .TIMCFG_REGISTER = &FLEXIO3_TIMCFG3
  }
};

// ---- OUTPUT_PORT CLASS FUNCTIONS ----

output_port::output_port(){};

void output_port::begin(uint8_t port_number){
  begin(port_number, OUTPUT_FORMAT_STEPDANCE);
}

void output_port::begin(uint8_t port_number, uint8_t output_format){
  // Configures the output port.
  //
  // port_number -- the port ID number, from 0-3.
  // mode -- specifies whether 
  
  // -- Store Parameters for Later --
  this->port_number = port_number;
  this->output_format = output_format;

  // -- Configure Teensy Output Pins --
  pinMode(port_info[port_number].STEP_TEENSY_PIN, OUTPUT);
  pinMode(port_info[port_number].DIR_TEENSY_PIN, OUTPUT);
  digitalWrite(port_info[port_number].STEP_TEENSY_PIN, LOW);
  digitalWrite(port_info[port_number].DIR_TEENSY_PIN, LOW);

  // --- CONFIG FLEXIO ---
  // I'm heavily referencing this post: https://forum.pjrc.com/index.php?threads/teensy-4-1-how-to-start-using-flexio.66201/
  // Note that we've chosen output pins that all reside on the FlexIO3 Submodule

  // -- Enable Clock to FLEXIO3 --
  // We do this throught the CCM Clock Gating Register 3 (See P1021 for assignment, P1087 for register definition)

  CCM_CCGR7 |= CCM_CCGR7_FLEXIO3(CCM_CCGR_ON);

  // -- Set Clock Speed --
  // Default is PLL3_SW_CLK (480MHz) / 16 = 30 MHz
  // According to table on 1032, FLEXIO2 can go up to 120 MHz
  // So, we'll set CS1CDR[FLEXIO2_CLK_PODF] to /2. In combination with the default /2 value of
  // CS1CDR[FLEXIO2_CLK_PRED], we'll end up with /4 and a clock speed of 480MHz/4 = 120MHz
  // Note: This should also set the clock dividers for FlexIO3
  CCM_CS1CDR &= ~(CCM_CS1CDR_FLEXIO2_CLK_PODF(7)); //clear all bits
  CCM_CS1CDR |= CCM_CS1CDR_FLEXIO2_CLK_PODF(1); //set to 1 (/2, see p1063)

  // -- Enable FlexIO --
  FLEXIO3_CTRL |= 1; // Bit0 of FLEXIO3_CTRL enables the module. (P2916)

  // -- MAP PADS TO FLEXIO MODULE PINS --
  *port_info[port_number].STEP_PIN_IOMUXC_REGISTER = port_info[port_number].STEP_PIN_IOMUXC_VALUE;
  *port_info[port_number].DIR_PIN_IOMUXC_REGISTER = port_info[port_number].DIR_PIN_IOMUXC_VALUE;

  // -- Configure Step Output Shifter --
  // Set Shifter Control Register (P2926)
  *port_info[port_number].STEP_SHIFTCTRL_REGISTER	= 
    FLEXIO_SHIFTCTL_TIMSEL(port_info[port_number].TIMER_ID) |	// select timerX as input. Multiple shifters can share a timer input.
		// FLEXIO_SHIFTCTL_TIMPOL	|			          // on positive edge
		FLEXIO_SHIFTCTL_PINCFG(3) |			          // pin output enabled
		FLEXIO_SHIFTCTL_PINSEL(port_info[port_number].STEP_FLEXIO_PIN)|	  // FLEXIO PIN
		// FLEXIO_SHIFTCTL_PINPOL	|			          // active high
		FLEXIO_SHIFTCTL_SMOD(2);					        // transmit mode

  // Set Shifter Config Register (P2927)
  *port_info[port_number].STEP_SHIFTCFG_REGISTER	= 
    FLEXIO_SHIFTCFG_PWIDTH(0)	|	// single bit
		// FLEXIO_SHIFTCFG_INSRC |	              // pin input
		FLEXIO_SHIFTCFG_SSTOP(0) |			        // stop bit disabled
		FLEXIO_SHIFTCFG_SSTART(0);					    // start bit disabled

  // -- Configure Dir Output Shifter --
  // Set Shifter Control Register (P2926)
  *port_info[port_number].DIR_SHIFTCTRL_REGISTER	= 
    FLEXIO_SHIFTCTL_TIMSEL(port_info[port_number].TIMER_ID) |	// select same timerX as input.
		// FLEXIO_SHIFTCTL_TIMPOL	|			          // on positive edge
		FLEXIO_SHIFTCTL_PINCFG(3) |			          // pin output enabled
		FLEXIO_SHIFTCTL_PINSEL(port_info[port_number].DIR_FLEXIO_PIN)| // FLEXIO PIN
		// FLEXIO_SHIFTCTL_PINPOL	|			          // active high
		FLEXIO_SHIFTCTL_SMOD(2);					        // transmit mode

  // Set Shifter Config Register (P2927)
  *port_info[port_number].DIR_SHIFTCFG_REGISTER	= 
    FLEXIO_SHIFTCFG_PWIDTH(0)	|	// single bit
		// FLEXIO_SHIFTCFG_INSRC |	              // pin input
		FLEXIO_SHIFTCFG_SSTOP(0) |			        // stop bit disabled
		FLEXIO_SHIFTCFG_SSTART(0);					    // start bit disabled
  
  // -- Configure Timer --

  // Set Timer Compare Register (p2938)
  *port_info[port_number].TIMCMP_REGISTER = (63 <<8) | //32 bits of output.
        ((120/2)-1); // We want an output every us, so we need CLK/120

  // Set Timer Control Register (P2933)
  *port_info[port_number].TIMCTL_REGISTER	= 
    FLEXIO_TIMCTL_TRGSEL((port_info[port_number].TIMER_ID *4) + 1)|			  // Trigger on Step Shifter Status Flag
		FLEXIO_TIMCTL_TRGPOL |			                // Invert the Trigger -- we want to trigger when the status flag is cleared,
                                                    // which happens when new data is loaded into Shifter 0's buffer
		FLEXIO_TIMCTL_TRGSRC |			                // internal trigger
		FLEXIO_TIMCTL_PINCFG(0) |			              // timer pin output disabled
		FLEXIO_TIMCTL_PINSEL(0)	|			              // Input pin select, not in use
		// FLEXIO_TIMCTL_PINPOL		|			            // input pin polarity, not in use
		FLEXIO_TIMCTL_TIMOD(1);					            // timer mode: 8 bit baud mode
  
  // Set Timer Config Register
  *port_info[port_number].TIMCFG_REGISTER	= 
    FLEXIO_TIMCFG_TIMOUT(0)	|		// timer output = high, not affcted by reset
		FLEXIO_TIMCFG_TIMDEC(0)	|			            // decrement on FlexIO clock
		FLEXIO_TIMCFG_TIMRST(0) |			            // dont reset timer 
		FLEXIO_TIMCFG_TIMDIS(2)	|			            // disable timer on timer compare
		FLEXIO_TIMCFG_TIMENA(2)	|			            // enable timer on trigger high
		FLEXIO_TIMCFG_TSTOP(0);			              // stop bit disabled
		//FLEXIO_TIMCFG_TSTART					          // start bit disabled  

  // register the output port
  register_output_port();
}

void output_port::transmit_frame(){
  encode();
  transmit();
  clear_all_signals();
}

void output_port::transmit(){
  // Transmits raw step and direction sequences over the output pins
  *port_info[port_number].DIR_SHIFTBUF = active_encoded_frame_dir;
  *port_info[port_number].STEP_SHIFTBUF = active_encoded_frame_step; //writing to the step shift buffer triggers transmission, so we do it last.
}

void output_port::clear_all_signals(){
  // Clears all active signal flags
  // We only bother clearing the step signal flags, because direction doesn't matter if the step flag is clear.

  for(uint8_t i = 0; i < NUM_SIGNALS; i++){
    active_signals[i] = 0;
  }
}

void output_port::add_signal(uint8_t signal_index, uint8_t signal_direction){
  // Adds a specific signal within the frame.
  // When the signal is added, a corresponding width pulse will be generated on transmit
  //
  // signal_index -- the index of the target signal within active_signals. We provide a bunch of defines to make it
  //                  easier to keep track of these... i.e. SIGNAL_X, SIGNAL_Y, etc...
  // signal_direction -- 0 for reverse, 1 for forward
  active_signals[signal_index] = 1;
  active_signal_directions[signal_index] = signal_direction;
}

void output_port::encode(){
  // Encodes the active_signals and _directions arrays into active_encoded_frame_step and _dir

  // First, initialize the active_encoded_frame registers
  active_encoded_frame_step = 0;
  active_encoded_frame_dir = 0;

  // Initialize framing state
  uint8_t step_pulse_bit_position = 2; // leading edge of first signal pulse is left-shifted two bits.
  uint8_t dir_pulse_bit_position = 1; // leading edge of first direction pulse is left-shifted one bit.

  uint32_t step_pulse;
  uint8_t step_pulse_length;
  uint32_t dir_pulse;
  uint8_t dir_pulse_length;

  for(uint8_t signal_index = 0; signal_index < NUM_SIGNALS; signal_index ++){
    if(active_signals[signal_index]){ //only bother to process active signals
      //synthesize step and direction pulses
      step_pulse_length = signal_index + SIGNAL_MIN_WIDTH_US;
      step_pulse = (1<<step_pulse_length) - 1;
      dir_pulse_length = step_pulse_length + SIGNAL_GAP_US;
      if(active_signal_directions[signal_index]){ //direction is forwards, need a pulse
        dir_pulse = (1<<dir_pulse_length) - 1;
      }else{
        dir_pulse = 0;
      }

      // overlay on encoded frame
      active_encoded_frame_step |= (step_pulse << step_pulse_bit_position);
      active_encoded_frame_dir |= (dir_pulse << dir_pulse_bit_position);

      // update bit positions
      step_pulse_bit_position += step_pulse_length + SIGNAL_GAP_US;
      dir_pulse_bit_position += dir_pulse_length;
    }
  }
}

void output_port::register_output_port(){
  if(num_registered_output_ports < NUM_AVAILABLE_OUTPUT_PORTS){
    registered_output_ports[num_registered_output_ports] = this;
    num_registered_output_ports ++;
  } // NOTE: should add a return value if it works
}

void transmit_frames_on_all_output_ports(){
  for(uint8_t output_port_index = 0; output_port_index < num_registered_output_ports; output_port_index++){
    registered_output_ports[output_port_index]->transmit_frame();
  }
}