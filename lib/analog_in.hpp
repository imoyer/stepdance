#include "arm_math.h"
#include <sys/_stdint.h>
#include <Arduino.h>
#include "core.hpp"

/*
Analog Input Module of the StepDance Control System

This module is responsible for reading analog input values in the background, and setting target parameters.

[More Details to be Added]

A part of the Mixing Metaphors Project
(c) 2025 Ilan Moyer, Jennifer Jacobs, Devon Frost

*/

#ifndef analog_in_h // prevent importing twice
#define analog_in_h

#define ADC_MODULE_1 0
#define ADC_MODULE_2 1

#define ADC_NONE 255 // used to flag modules and channels when pin does not map to an ADC

#define NUM_ADC_MODULES 2
#define MAX_NUM_ADC_INPUTS 16 // per module

// ---ADC SETTINGS---

// AVERAGING
#define ANALOG_AVERAGING_1 -1 // AVGE = 0
#define ANALOG_AVERAGING_4 0  // AVGE = 1, corresponds to settings of AVGS
#define ANALOG_AVERAGING_8 1
#define ANALOG_AVERAGING_16 2
#define ANALOG_AVERAGING_32 3

// SAMPLE RESOLUTION
#define ANALOG_RESOLUTION_8_BIT 0
#define ANALOG_RESOLUTION_10_BIT 1
#define ANALOG_RESOLUTION_12_BIT 2

// CLOCK SPEED
#define ANALOG_CLOCK_75MHZ_DIV_4 1
#define ANALOG_CLOCK_75MHZ_DIV_8 2
#define ANALOG_CLOCK_75MHZ_DIV_16 3

// ENABLED
#define ANALOG_INPUT_DISABLED 0
#define ANALOG_INPUT_ENABLED 1

// REFERENCE VOLTAGE
#define VREF_3V3 3.3

// DEADBAND STARTUP DELAY
#define ANALOG_DEADBAND_STARTUP_DELAY_MS 20

/** \cond */
 /**
   * This struct will be hidden from Doxygen documentation.
   */
struct analog_pin_info_struct
{ // hardware_specific
  // Physical IO
  uint8_t TEENSY_PIN;        // TEENSY Pin #s
  uint8_t ADC_MODULE;        // Which ADC module to use with this pin (some pins only support a single module)
  uint8_t ADC_INPUT_CHANNEL; // input number on the module
};
/** \endcond */
/**
 * @brief AnalogInput components read  values from physical user interface components and enable them to be mapped to Stepdance Components.
 * @ingroup inputs
 *
 *  AnalogInputs function similarly to the Arduino [analogIn](https://docs.arduino.cc/language-reference/en/functions/analog-io/analogRead/) function. They can be used to directly map analog readings from the Analog input terminals on the Machine Controller or Basic Module to Stepdance Component ControlParameters. Alternatively, you can read the value of the AnalogInput directly during the loop() function to implement custom control schemes. You can read AnalogInput values at either scaled values (based on user-defined floor and ceiling settings) or as raw Analog-to-Digital Converter (ADC) values.
 *
 * Here's an example of how to instantiate and configure two AnalogInputs to either map to control parameters or read a value directly during the loop() function:
 * @snippet snippets.cpp AnalogInput
 */

class AnalogInput : public Plugin
{
public:
  /**
   * @brief Default constructor for AnalogInput.
   * Initializes an AnalogInput instance with default, unconfigured state. This does not
   * configure hardware or register the input; call begin() to
   * initialize hardware-specific settings and register the input with the system.
   */
  AnalogInput();
  /**
   * @brief Initialize the AnalogInput with a pin reference corresponding to the target physical analog input pin on the Stepdance Board.
   * @param pin_reference Index of the physical analog input pin (e.g. IO_A1, IO_A2, IO_A3, IO_A4).
   */
  void begin(uint8_t pin_reference);
  /**
   * @brief Maps the AnalogInput to a ControlParameter, which will be updated each time a new analog value is read. ControlParameters are typically used to control Component parameters (e.g. 1DWaveGenerator Amplitude, VelocityGenerator Speed etc.).
   * @param target_parameter Pointer to the ControlParameter to map to.
   */
  void map(ControlParameter *target_parameter);
  /**
   * @brief Sets a callback function that will be called each time a new analog value is read.
   * @param callback_function Pointer to the callback function to be executed on new data.
   * code example:
   * @snippet snippets.cpp AnalogInputCallback
   */
  void set_callback(void (*callback_function)());

  /**
   * @brief Sets the scaled output value of the Analog Input at the floor (lower limit).
   * @param output_at_floor ControlParameter value corresponding to scaled output value at the lower limit.
   */
  void set_floor(ControlParameter output_at_floor); // sets the scaled output value at the floor
  /**
   * @brief  Sets the scaled output value of the Analog Input at the floor (lower limit). This defines the mapping between raw ADC values and the output ControlParameter values.
   * @param output_at_floor ControlParameter value corresponding to scaled output value at the lower limit.
   * @param adc_lower_limit Raw ADC value corresponding to the lower limit.
   */
  void set_floor(ControlParameter output_at_floor, uint16_t adc_lower_limit); // allows the lower limit to be adjusted. ADC values below this will output at the floor
  /**
   * @brief  Sets the scaled output value of the Analog Input at the ceiling (upper limit). This defines the mapping between raw ADC values and the output ControlParameter values.
   * @param output_at_ceiling ControlParameter value corresponding to scaled output value at the upper limit.
   */
  void set_ceiling(ControlParameter output_at_ceiling);
  /**
   * @brief  Sets the scaled output value of the Analog Input at the ceiling (upper limit). This defines the mapping between raw ADC values and the output ControlParameter values.
   * @param output_at_ceiling ControlParameter value corresponding to scaled output value at the upper limit.
   * @param adc_upper_limit Raw ADC value corresponding to the upper limit.
   */ 
  void set_ceiling(ControlParameter output_at_ceiling, uint16_t adc_upper_limit);
  /**
   * @brief Inverts the output of the Analog Input, such that higher raw ADC values correspond to lower ControlParameter values and vice versa. Useful for inverting potentiometer or other analog UI elements without rewiring.
   */
  void invert();               // inverts the output
  /**
   * @brief Reads the last converted and scaled value from the Analog Input.
   * @return The last read value, based on the internal conversion factor.
   */
  ControlParameter read();     // returns the last read value, based on the internal conversion factor
  /**
   * @brief Reads the last raw ADC value from the Analog Input.
   * @return The last read raw ADC value.
   */
  ControlParameter read_raw(); // returns the last read raw value.

  /** \cond */
  /**
   * These functions and methods will be hidden from Doxygen documentation.
   */
  void map(DecimalPosition *target_parameter);
  void calibrate();
  void set_averaging(int8_t averaging);
  void set_resolution(int8_t resolution);
  void set_clock(int8_t clock);
  volatile uint16_t last_value_raw = 0;
  void (*callback_function)() = nullptr;
  ControlParameter *target_control_param = nullptr;
  DecimalPosition *target_decimal_pos = nullptr;
  static AnalogInput *adc1_inputs[MAX_NUM_ADC_INPUTS]; // keeps pointers to all instantiated analog inputs on the ADC1 module
  static AnalogInput *adc2_inputs[MAX_NUM_ADC_INPUTS];
  static uint8_t module_num_inputs[NUM_ADC_MODULES];                                                // tracks the number of instantiated analog inputs on the ADC module; indexed by ADC module
  static volatile uint8_t module_current_input_index[NUM_ADC_MODULES];                              // current analog input
  void configure_adc();                                                                             // changes the ADC configuration to support this AnalogInput
  void begin_conversion();                                                                          // begins a conversion on the ADC module
  void set_deadband_here(ControlParameter output_at_deadband = 0, uint16_t adc_deadband_width = 4); // sets the deadband to the current value.
  void set_deadband(ControlParameter output_at_deadband, uint16_t adc_deadband_center, uint16_t adc_deadband_width = 4);
  float32_t full_scale_volts = VREF_3V3; // we'll default to this for now, until we support changing the reference voltage.

  float32_t conversion_slope_1 = 1;
  float32_t conversion_intercept_1 = 0;
  float32_t conversion_slope_2 = 1;
  float32_t conversion_intercept_2 = 0;
  uint16_t adc_deadband_location = 0;
  uint16_t adc_deadband_width = 0;
  uint16_t adc_deadband_lower = 0;
  uint16_t adc_deadband_upper = 0;

  void enroll(RPC *rpc, const String &instance_name);
  /** \endcond */

private:
  // Static Parameters and Methods
  static const uint16_t max_adc_values[3];                      // 8, 10, 12 bit
  static const struct analog_pin_info_struct analog_pin_info[]; // stores setup information for all analog inputs
  static volatile uint8_t module_calibrating[NUM_ADC_MODULES];  // flag to indicate that ADC is currently calibrating
  static void adc1_on_interrupt();
  static void adc2_on_interrupt();
  void set_slope_intercept(); // utility to set the slope and intercept values.

  // Instance Parameters and Methods
  void initialize_adc(); // initializes ADC modules
  // Default Values
  // SFCAdder = 3, AverageNum = 32, BCT = 21, LST = 21 --> Total ADCLK per input = 1347
  // At CLK/16, this gives a total sample rate of 3480 Hz, which would be the interrupt rate.
  // Spread across four analog inputs, this is ~ 870Hz. We'll turn off the VREF inputs when not actively reading them
  int8_t averaging = ANALOG_AVERAGING_32;
  int8_t resolution = ANALOG_RESOLUTION_10_BIT;
  int8_t clock = ANALOG_CLOCK_75MHZ_DIV_16;
  int8_t input_enabled = ANALOG_INPUT_ENABLED;
  uint8_t adc_module;
  uint8_t adc_input_channel;
  uint8_t teensy_pin;
  ControlParameter output_at_floor = 0;
  ControlParameter output_at_deadband = 512;
  ControlParameter output_at_ceiling = 1023;
  uint16_t adc_lower_limit = 0;
  uint16_t adc_upper_limit = 1023; // 10 bit
  bool deadband_enabled = false;

  int8_t inversion_multiplier = 1; // 1 for straight thru, -1 for inverted
};

#endif