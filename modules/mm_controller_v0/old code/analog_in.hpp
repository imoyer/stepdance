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

#ifndef analog_in_h //prevent importing twice
#define analog_in_h

#ifdef ARDUINO_TEENSY41
  #define INPUT_A1    0
  #define INPUT_A2    1
  #define INPUT_A3    2
  #define INPUT_A4    3
  #define MOTOR_A_VREF    4
  #define MOTOR_B_VREF    5
  #define MOTOR_C_VREF    6
  #define MOTOR_D_VREF    7
  #define INPUT_LEGACY_A  12
  #define INPUT_LEGACY_B  13
  #define INPUT_LEGACY_C  14
  #define INPUT_LEGACY_D  15
  #define INPUT_LEGACY_E  16

#else //TEENSY 4
  #define INPUT_A1    8
  #define INPUT_A2    9
  #define INPUT_A3    10
  #define INPUT_A4    11
#endif

#define ADC_MODULE_1    0
#define ADC_MODULE_2    1

#define NUM_ADC_MODULES     2
#define MAX_NUM_ADC_INPUTS  16  //per module

// ---ADC SETTINGS---

// AVERAGING
#define ANALOG_AVERAGING_1    -1 //AVGE = 0
#define ANALOG_AVERAGING_4    0 //AVGE = 1, corresponds to settings of AVGS
#define ANALOG_AVERAGING_8    1
#define ANALOG_AVERAGING_16   2
#define ANALOG_AVERAGING_32   3

// SAMPLE RESOLUTION
#define ANALOG_RESOLUTION_8_BIT   0
#define ANALOG_RESOLUTION_10_BIT  1
#define ANALOG_RESOLUTION_12_BIT  2

// CLOCK SPEED
#define ANALOG_CLOCK_75MHZ_DIV_4     1
#define ANALOG_CLOCK_75MHZ_DIV_8     2
#define ANALOG_CLOCK_75MHZ_DIV_16    3

// ENABLED
#define ANALOG_INPUT_DISABLED 0
#define ANALOG_INPUT_ENABLED  1


struct analog_pin_info_struct{ //hardware_specific 
    // Physical IO
  uint8_t TEENSY_PIN; //TEENSY Pin #s
  uint8_t ADC_MODULE; //Which ADC module to use with this pin (some pins only support a single module)
  uint8_t ADC_INPUT_CHANNEL; //input number on the module
};

class AnalogInput{
  public:
    AnalogInput();
    void begin(uint8_t pin_reference);
    void begin(uint8_t pin_reference, ControlParameter *target_parameter);
    void calibrate();
    void set_averaging(int8_t averaging);
    void set_resolution(int8_t resolution);
    void set_clock(int8_t clock);
    void map(ControlParameter *target_parameter);
    void set_callback(void (*callback_function)());
    volatile uint16_t last_value_raw = 0;
    void (*callback_function)() = nullptr;
    ControlParameter *target = nullptr;
    static AnalogInput *adc1_inputs[MAX_NUM_ADC_INPUTS]; //keeps pointers to all instantiated analog inputs on the ADC1 module
    static AnalogInput *adc2_inputs[MAX_NUM_ADC_INPUTS];
    static uint8_t module_num_inputs[NUM_ADC_MODULES]; //tracks the number of instantiated analog inputs on the ADC module; indexed by ADC module
    static volatile uint8_t module_current_input_index[NUM_ADC_MODULES]; //current analog input
    void configure_adc(); //changes the ADC configuration to support this AnalogInput
    void begin_conversion(); //begins a conversion on the ADC module

  private:
    // Static Parameters and Methods
    static const struct analog_pin_info_struct analog_pin_info[]; //stores setup information for all analog inputs
    static volatile uint8_t module_calibrating[NUM_ADC_MODULES]; //flag to indicate that ADC is currently calibrating
    static void adc1_on_interrupt();
    static void adc2_on_interrupt();
    
    // Instance Parameters and Methods
    void initialize_adc(); //initializes ADC modules
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
};


#endif