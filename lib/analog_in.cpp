#include "arm_math.h"
#include <sys/_stdint.h>
#include "pins_arduino.h"
#include "core_pins.h"
#include "imxrt.h"
#include "analog_in.hpp"

// Initialize static variables
uint8_t AnalogInput::module_num_inputs[NUM_ADC_MODULES] = {0, 0};
volatile uint8_t AnalogInput::module_current_input_index[NUM_ADC_MODULES] = {0, 0};
volatile uint8_t AnalogInput::module_calibrating[NUM_ADC_MODULES] = {0, 0}; //flag to indicate that ADC is currently calibrating
AnalogInput* AnalogInput::adc1_inputs[MAX_NUM_ADC_INPUTS] = {nullptr, nullptr}; //keeps pointers to all instantiated analog inputs on the ADC1 module
AnalogInput* AnalogInput::adc2_inputs[MAX_NUM_ADC_INPUTS] = {nullptr, nullptr};

const uint16_t max_adc_values[3] = {255, 1023, 4095}; //8, 10, 12 bit

const struct analog_pin_info_struct AnalogInput::analog_pin_info[] = {
  // We segregate A1 and A2 onto ADC1, and A3 and A4 onto ADC2. This will provide flexibility for running at very different speeds.
  // All of the VREF inputs are on ADC2

  { .TEENSY_PIN = 0,
    .ADC_MODULE = ADC_NONE,
    .ADC_INPUT_CHANNEL = ADC_NONE},
  { .TEENSY_PIN = 1,
    .ADC_MODULE = ADC_NONE,
    .ADC_INPUT_CHANNEL = ADC_NONE},
  { .TEENSY_PIN = 2,
    .ADC_MODULE = ADC_NONE,
    .ADC_INPUT_CHANNEL = ADC_NONE},
  { .TEENSY_PIN = 3,
    .ADC_MODULE = ADC_NONE,
    .ADC_INPUT_CHANNEL = ADC_NONE},
  { .TEENSY_PIN = 4,
    .ADC_MODULE = ADC_NONE,
    .ADC_INPUT_CHANNEL = ADC_NONE},
  { .TEENSY_PIN = 5,
    .ADC_MODULE = ADC_NONE,
    .ADC_INPUT_CHANNEL = ADC_NONE},
  { .TEENSY_PIN = 6,
    .ADC_MODULE = ADC_NONE,
    .ADC_INPUT_CHANNEL = ADC_NONE},
  { .TEENSY_PIN = 7,
    .ADC_MODULE = ADC_NONE,
    .ADC_INPUT_CHANNEL = ADC_NONE},
  { .TEENSY_PIN = 8,
    .ADC_MODULE = ADC_NONE,
    .ADC_INPUT_CHANNEL = ADC_NONE},
  { .TEENSY_PIN = 9,
    .ADC_MODULE = ADC_NONE,
    .ADC_INPUT_CHANNEL = ADC_NONE},
  { .TEENSY_PIN = 10,
    .ADC_MODULE = ADC_NONE,
    .ADC_INPUT_CHANNEL = ADC_NONE},
  { .TEENSY_PIN = 11,
    .ADC_MODULE = ADC_NONE,
    .ADC_INPUT_CHANNEL = ADC_NONE},
  { .TEENSY_PIN = 12,
    .ADC_MODULE = ADC_NONE,
    .ADC_INPUT_CHANNEL = ADC_NONE},
  { .TEENSY_PIN = 13,
    .ADC_MODULE = ADC_NONE,
    .ADC_INPUT_CHANNEL = ADC_NONE},
  { .TEENSY_PIN = 14, // BASIC MODULE - INPUT A1 - AD_B1_02
    .ADC_MODULE = ADC_MODULE_2,
    .ADC_INPUT_CHANNEL = 7},
  { .TEENSY_PIN = 15, // BASIC MODULE - INPUT A2, DRIVER MODULE - MOTOR A VREF - AD_B1_03
    .ADC_MODULE = ADC_MODULE_2,
    .ADC_INPUT_CHANNEL = 8},
  { .TEENSY_PIN = 16,
    .ADC_MODULE = ADC_MODULE_1,
    .ADC_INPUT_CHANNEL = 12},
  { .TEENSY_PIN = 17,
    .ADC_MODULE = ADC_MODULE_1,
    .ADC_INPUT_CHANNEL = 11},  
  { .TEENSY_PIN = 18,
    .ADC_MODULE = ADC_MODULE_1,
    .ADC_INPUT_CHANNEL = 6}, 
  { .TEENSY_PIN = 19,
    .ADC_MODULE = ADC_MODULE_2,
    .ADC_INPUT_CHANNEL = 5}, 
  { .TEENSY_PIN = 20,
    .ADC_MODULE = ADC_MODULE_1,
    .ADC_INPUT_CHANNEL = 15}, 
  { .TEENSY_PIN = 21,
    .ADC_MODULE = ADC_MODULE_2,
    .ADC_INPUT_CHANNEL = 0}, 
  { .TEENSY_PIN = 22,
    .ADC_MODULE = ADC_MODULE_1,
    .ADC_INPUT_CHANNEL = 13}, 
  { .TEENSY_PIN = 23,
    .ADC_MODULE = ADC_MODULE_2,
    .ADC_INPUT_CHANNEL = 14},
  { .TEENSY_PIN = 24, // TEENSY 4.1 - INPUT A1 - AD_B0_12
    .ADC_MODULE = ADC_MODULE_1,
    .ADC_INPUT_CHANNEL = 1},
  { .TEENSY_PIN = 25, // TEENSY 4.1 - INPUT A2 - AD_B0_13
    .ADC_MODULE = ADC_MODULE_1,
    .ADC_INPUT_CHANNEL = 2},
  { .TEENSY_PIN = 26, // TEENSY 4.1 - INPUT A3 - AD_B1_14
    .ADC_MODULE = ADC_MODULE_2,
    .ADC_INPUT_CHANNEL = 3},
  { .TEENSY_PIN = 27, // TEENSY 4.1 - INPUT A4 - AD_B1_15
    .ADC_MODULE = ADC_MODULE_2,
    .ADC_INPUT_CHANNEL = 4},
  { .TEENSY_PIN = 28,
    .ADC_MODULE = ADC_NONE,
    .ADC_INPUT_CHANNEL = ADC_NONE},
  { .TEENSY_PIN = 29, 
    .ADC_MODULE = ADC_NONE,
    .ADC_INPUT_CHANNEL = ADC_NONE},
  { .TEENSY_PIN = 30, 
    .ADC_MODULE = ADC_NONE,
    .ADC_INPUT_CHANNEL = ADC_NONE},
  { .TEENSY_PIN = 31, 
    .ADC_MODULE = ADC_NONE,
    .ADC_INPUT_CHANNEL = ADC_NONE},
  { .TEENSY_PIN = 32,
    .ADC_MODULE = ADC_NONE,
    .ADC_INPUT_CHANNEL = ADC_NONE},
  { .TEENSY_PIN = 33, 
    .ADC_MODULE = ADC_NONE,
    .ADC_INPUT_CHANNEL = ADC_NONE},
  { .TEENSY_PIN = 34,
    .ADC_MODULE = ADC_NONE,
    .ADC_INPUT_CHANNEL = ADC_NONE},
  { .TEENSY_PIN = 35,
    .ADC_MODULE = ADC_NONE,
    .ADC_INPUT_CHANNEL = ADC_NONE},
  { .TEENSY_PIN = 36,
    .ADC_MODULE = ADC_NONE,
    .ADC_INPUT_CHANNEL = ADC_NONE},
  { .TEENSY_PIN = 37,
    .ADC_MODULE = ADC_NONE,
    .ADC_INPUT_CHANNEL = ADC_NONE},
  { .TEENSY_PIN = 38, // TEENSY 4.1 - MOTOR D VREF - AD_B1_12
    .ADC_MODULE = ADC_MODULE_2,
    .ADC_INPUT_CHANNEL = 1},
  { .TEENSY_PIN = 39,
    .ADC_MODULE = ADC_MODULE_2,
    .ADC_INPUT_CHANNEL = 2},
  { .TEENSY_PIN = 40, // TEENSY 4.1 - MOTOR C VREF - AD_B1_04
    .ADC_MODULE = ADC_MODULE_1,
    .ADC_INPUT_CHANNEL = 9},
  { .TEENSY_PIN = 41, // TEENSY 4.1 - MOTOR B VREF - AD_B1_O5
    .ADC_MODULE = ADC_MODULE_1,
    .ADC_INPUT_CHANNEL = 10},
};

AnalogInput::AnalogInput(){};

void AnalogInput::initialize_adc(){
  // Initializes the ADC module.

  // Step 1: Turn on the ADC clocks
  if(adc_module == ADC_MODULE_1){
    CCM_CCGR1 |= CCM_CCGR1_ADC1(CCM_CCGR_ON); //ADC1
  }else{
    CCM_CCGR1 |= CCM_CCGR1_ADC2(CCM_CCGR_ON); //ADC2
  }

  // Step 2: Calibrate
  calibrate();

  // Step 3: Enable Interrupts
  if(adc_module == ADC_MODULE_1){
    attachInterruptVector(IRQ_ADC1, AnalogInput::adc1_on_interrupt);
    NVIC_SET_PRIORITY(IRQ_ADC1, 150); //lowest priority so far
    NVIC_ENABLE_IRQ(IRQ_ADC1);
  }else{
    attachInterruptVector(IRQ_ADC2, AnalogInput::adc2_on_interrupt);
    NVIC_SET_PRIORITY(IRQ_ADC2, 150); //lowest priority so far
    NVIC_ENABLE_IRQ(IRQ_ADC2);
  }
}

void AnalogInput::configure_adc(){
  uint32_t CFG_temp = 0; // temp Configuration register
  uint32_t GC_temp = 0; // temp General Control register

  //Config Register Basic Settings
  CFG_temp = ADC_CFG_OVWREN | ADC_CFG_ADICLK(1); //enable overwrites, input clock /2
  
  // Set Clock Divider
  CFG_temp |= ADC_CFG_ADIV(clock); //set clock divider

  // Set Averaging
  if(averaging >= 0){
    CFG_temp |= ADC_CFG_AVGS(averaging); //set averaging
    GC_temp |= ADC_GC_AVGE; //enable averaging
  }else{
    //we simply don't set anything to these registers
  }

  // Set Resolution
  switch(resolution){
    case ANALOG_RESOLUTION_8_BIT:
      CFG_temp |= ADC_CFG_MODE(resolution)|ADC_CFG_ADSTS(3); //short sample time, 9 ADCLK
      break;
    case ANALOG_RESOLUTION_10_BIT:
      CFG_temp |= ADC_CFG_MODE(resolution)|ADC_CFG_ADSTS(2)|ADC_CFG_ADLSMP; //long sample time, 21 ADCLK
      break;
    case ANALOG_RESOLUTION_12_BIT:
      CFG_temp |= ADC_CFG_MODE(resolution)|ADC_CFG_ADSTS(3)|ADC_CFG_ADLSMP; //long sample time, 21 ADCLK
      break;      
  }
  if(adc_module == ADC_MODULE_1){
    ADC1_CFG = CFG_temp;
    ADC1_GC = GC_temp;
  }else{
    ADC2_CFG = CFG_temp;
    ADC2_GC = GC_temp;    
  }
}

void AnalogInput::begin_conversion(){
  if(adc_module == ADC_MODULE_1){
    ADC1_HC0 = adc_input_channel | ADC_HC_AIEN; //sets input channel, enables interrupts locally. This triggers a conversion.
  }else{
    ADC2_HC0 = adc_input_channel | ADC_HC_AIEN;
  }
}

void AnalogInput::calibrate(){
  if(adc_module == ADC_MODULE_1){
    ADC1_CFG |= ADC_CFG_ADHSC; //set high speed mode
    ADC1_GC |= ADC_GC_CAL; //enable calibration
    module_calibrating[adc_module] = 1;
    while(ADC1_GC & ADC_GC_CAL){
    };
    module_calibrating[adc_module] = 0;
  }else{
    ADC2_CFG |= ADC_CFG_ADHSC; //set high speed mode
    ADC2_GC |= ADC_GC_CAL; //enable calibration
    module_calibrating[adc_module] = 1;
    while(ADC2_GC & ADC_GC_CAL){
    };
    module_calibrating[adc_module] = 0;    
  }
}


void AnalogInput::begin(uint8_t pin_reference){
  // initialize the module
  // pin_reference -- the StepDance AnalogInput pin ID, e.g. INPUT_A1, etc (defined in analog_in.hpp)
  // *target_parameter -- a pointer to a ControlParameter (i.e. float32_t) that should be updated automatically by this AnalogInput

  // Step 1: Load all AnalogInput parameters
  this->adc_module = analog_pin_info[pin_reference].ADC_MODULE;
  this->adc_input_channel = analog_pin_info[pin_reference].ADC_INPUT_CHANNEL;
  this->teensy_pin = analog_pin_info[pin_reference].TEENSY_PIN;

  // Step 2: Configure AnalogInput Pin
  pinMode(teensy_pin, INPUT); //set as input
  volatile uint32_t *pad = portControlRegister(teensy_pin); // disbale pin keeper
  uint32_t padval = *pad;
  if((padval & (IOMUXC_PAD_PUE | IOMUXC_PAD_PKE)) == IOMUXC_PAD_PKE){
    *pad = padval & ~IOMUXC_PAD_PKE;
  }
  
  // Step 3: add this AnalogInput to the module's list
  uint8_t num_inputs = module_num_inputs[adc_module];
  if(num_inputs < MAX_NUM_ADC_INPUTS){ //check that we don't exceed max number of ADC inputs
    if(adc_module == ADC_MODULE_1){ //add AnalogInput to ADC Module 1
      adc1_inputs[num_inputs] = this;
    }else{ //add AnalogInput to ADC Module 2
      adc2_inputs[num_inputs] = this;
    }
    module_num_inputs[adc_module] ++;
  }

  // Step 4: If this is the first AnalogInput assigned to the module, initialize and begin conversions
  if(num_inputs == 0){
    configure_adc();
    initialize_adc();
    begin_conversion();
  }
}

void AnalogInput::invert(){
  inversion_multiplier *= -1;
}

// Public Utility Methods
ControlParameter AnalogInput::read(){
  // Returns the last read value, converted into real-world units.
  
  ControlParameter converted_value;

  // convert
  if(deadband_enabled){
    if(last_value_raw < adc_deadband_lower){
      converted_value = static_cast<ControlParameter>(last_value_raw) * conversion_slope_1 + conversion_intercept_1;
    }else if(last_value_raw > adc_deadband_upper){
      converted_value = static_cast<ControlParameter>(last_value_raw) * conversion_slope_2 + conversion_intercept_2;
    }else{
      converted_value = output_at_deadband;
    }
  }else{ //deadband NOT enabled
    converted_value =  static_cast<ControlParameter>(last_value_raw) * conversion_slope_1 + conversion_intercept_1;
  }

  // latch value
  if(converted_value < output_at_floor){
    return output_at_floor * inversion_multiplier;
  }else if(converted_value > output_at_ceiling){
    return output_at_ceiling * inversion_multiplier;
  }else{
    return converted_value * inversion_multiplier;
  }
}

// Setup Methods
void AnalogInput::map(ControlParameter *target_parameter){
  this->target = target_parameter;
}

void AnalogInput::set_floor(ControlParameter output_at_floor){
  set_floor(output_at_floor, this->adc_lower_limit);
}

void AnalogInput::set_floor(ControlParameter output_at_floor, uint16_t adc_lower_limit){
  // sets the minimum scaled output value
  // output_at_floor -- the scaled minimum output value
  // adc_lower_limit -- the adc value at which this minimum applies.
  this->output_at_floor = output_at_floor;
  this->adc_lower_limit = adc_lower_limit;
  set_slope_intercept();
}

void AnalogInput::set_ceiling(ControlParameter output_at_ceiling){
  set_ceiling(output_at_ceiling, this->adc_upper_limit);
}

void AnalogInput::set_ceiling(ControlParameter output_at_ceiling, uint16_t adc_upper_limit){
  // sets the maximum scaled output value
  // output_at_floor -- the maximum scaled output value
  // adc_lower_limit -- the adc value at which this maximum applies.
  this->output_at_ceiling = output_at_ceiling;
  this->adc_upper_limit = adc_upper_limit;
  set_slope_intercept();
}

void AnalogInput::set_slope_intercept(){
  // sets the slope and intercept whenever floor, ceiling, or deadband are updated.
  // if there's no deadband, we just use slope_1 and intercept_1
  if(deadband_enabled){
    this->conversion_slope_1 = (this->output_at_deadband - this->output_at_floor) / static_cast<ControlParameter>(this->adc_deadband_lower - this->adc_lower_limit);
    this->conversion_intercept_1 = this->output_at_floor - this->conversion_slope_1 * static_cast<ControlParameter>(this->adc_lower_limit);
    this->conversion_slope_2 = (this->output_at_ceiling - this->output_at_deadband) / static_cast<ControlParameter>(this->adc_upper_limit - this->adc_deadband_upper);
    this->conversion_intercept_2 = this->output_at_deadband - this->conversion_slope_2 * static_cast<ControlParameter>(this->adc_deadband_upper);    
  }else{
    this->conversion_slope_1 = (this->output_at_ceiling - this->output_at_floor) / static_cast<ControlParameter>(this->adc_upper_limit - this->adc_lower_limit);
    this->conversion_intercept_1 = this->output_at_floor - this->conversion_slope_1 * static_cast<ControlParameter>(this->adc_lower_limit);
  }
}

void AnalogInput::set_deadband_here(ControlParameter output_at_deadband, uint16_t adc_deadband_width){
  if(adc_deadband_width == 1){ //let's make sure it doesn't end up at zero when /2.
    adc_deadband_width = 2;
  }
  this->deadband_enabled = true;
  this->output_at_deadband = output_at_deadband;
  this->adc_deadband_width = adc_deadband_width;
  delay(ANALOG_DEADBAND_STARTUP_DELAY_MS); // we need to wait for a first reading before setting the deadband location
  this->adc_deadband_location = last_value_raw;
  this->adc_deadband_lower = this->adc_deadband_location - (this->adc_deadband_width/2);
  this->adc_deadband_upper = this->adc_deadband_location + (this->adc_deadband_width/2);
  set_slope_intercept();
}

// Interrupt Routines
void AnalogInput::adc1_on_interrupt(){
  AnalogInput *this_module = AnalogInput::adc1_inputs[AnalogInput::module_current_input_index[ADC_MODULE_1]];

  // Read and Store ADC Value
  this_module->last_value_raw = ADC1_R0;
  if(this_module->target != nullptr){
    *(this_module->target) = this_module->read();
  }

  // Increment ADC input
  AnalogInput::module_current_input_index[ADC_MODULE_1]++;
  if(AnalogInput::module_current_input_index[ADC_MODULE_1] == AnalogInput::module_num_inputs[ADC_MODULE_1]){
    AnalogInput::module_current_input_index[ADC_MODULE_1] = 0;
  }
  AnalogInput *next_module = AnalogInput::adc1_inputs[AnalogInput::module_current_input_index[ADC_MODULE_1]];
  next_module->configure_adc();

  // Trigger new reading
  next_module->begin_conversion();

  // Callback
  if(this_module->callback_function != nullptr){
    this_module->callback_function();
  }
}

void AnalogInput::adc2_on_interrupt(){
  AnalogInput *this_module = AnalogInput::adc2_inputs[AnalogInput::module_current_input_index[ADC_MODULE_2]];

  // Read and Store ADC Value
  this_module->last_value_raw = ADC2_R0;
  if(this_module->target != nullptr){
    *(this_module->target) = this_module->read();
  }

  // Increment ADC input
  AnalogInput::module_current_input_index[ADC_MODULE_2]++;
  if(AnalogInput::module_current_input_index[ADC_MODULE_2] == AnalogInput::module_num_inputs[ADC_MODULE_2]){
    AnalogInput::module_current_input_index[ADC_MODULE_2] = 0;
  }
  AnalogInput *next_module = AnalogInput::adc2_inputs[AnalogInput::module_current_input_index[ADC_MODULE_2]];
  next_module->configure_adc();

  // Trigger new reading
  next_module->begin_conversion();

  // Callback
  if(this_module->callback_function != nullptr){
    this_module->callback_function();
  }
}

// float32_t AnalogInput::get_volts_per_raw_count(){
//   // Returns the volts per count of the encoder.
//   return this->full_scale_volts / static_cast<float32_t>((uint32_t)255 << (this->resolution * 2));
// }