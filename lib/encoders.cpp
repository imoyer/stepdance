#include "arm_math.h"
#include "core_pins.h"
#include "QuadEncoder.h"
#include <stdint.h>
#include "encoders.hpp"

// QuadEncoder quad_encoder_1(1, ENCODER_1A_PIN, ENCODER_1B_PIN, 1); //no pullups required
// QuadEncoder quad_encoder_2(2, ENCODER_2A_PIN, ENCODER_2B_PIN, 1);
QuadEncoder quad_encoder_1(1);
QuadEncoder quad_encoder_2(2);
QuadEncoder quad_encoder_3(3);

QuadEncoder * Encoder::all_encoders[MAX_NUM_ENCODERS] = {&quad_encoder_1, &quad_encoder_2, &quad_encoder_3};

const struct encoder_info_struct Encoder::encoder_info[] = {
  // This structure contains an indexed list of available encoder pins.

  { .CHANNEL_A_TEENSY_PIN = 0,
    .CHANNEL_B_TEENSY_PIN = 1,
  },
  { 
    .CHANNEL_A_TEENSY_PIN = 2,
    .CHANNEL_B_TEENSY_PIN = 3,
  },
  { 
    .CHANNEL_A_TEENSY_PIN = 4,
    .CHANNEL_B_TEENSY_PIN = 7,
  }
};

Encoder::Encoder(){};

void Encoder::begin(uint8_t encoder_index){
  quad_encoder = all_encoders[encoder_index];
  QuadEncoder_configure(encoder_index);
  quad_encoder->setInitConfig();
  quad_encoder->init();
  output.begin(&encoder_value); //will be interacting with encoder_value
  register_plugin();
}

void Encoder::QuadEncoder_configure(uint8_t encoder_index){
  // This function reproduces the work done by the constructor in the QuadEncoder library. (Lines 83-> in QuadEncoder.cpp)
  // We do this so that we don't need to configure the XBAR until we know an encoder channel is being used.
  // This keeps unused encoder pins free for other purposes.
  //
  // encoder_index -- an index into encoder_info, for configuring the encoder.
  uint8_t channel_a_pin = encoder_info[encoder_index].CHANNEL_A_TEENSY_PIN;
  uint8_t channel_b_pin = encoder_info[encoder_index].CHANNEL_B_TEENSY_PIN;
  quad_encoder->enc_xbara_mapping(channel_a_pin, PHASEA, 1); //no pullups
  quad_encoder->enc_xbara_mapping(channel_b_pin, PHASEB, 1); //no pullups
  quad_encoder->disableInterrupts(_positionROEnable);
  quad_encoder->disableInterrupts(_positionRUEnable);
}

void Encoder::invert(){
  invert_flag ^= 1;
}

int32_t Encoder::read(){
  return quad_encoder->read();
}

void Encoder::reset(){
  set(0);
}

void Encoder::set(DecimalPosition position){
  // Sets the encoder value, in world coordinates.
  if(invert_flag){
    position *= -1;
  }
  float64_t position_raw = output.convert_world_to_block_units(position);
  int32_t position_int = static_cast<int32_t>(position_raw);
  quad_encoder->write(position_int);
  output.reset(position_raw, true);
}

void Encoder::set_ratio(float output_units, float encoder_units){
  output.set_ratio(output_units, encoder_units);
}

void Encoder::set_latch(DecimalPosition value_world_units, uint8_t min_or_max){
  min_or_max ^= invert_flag; //we invert first, so that min/max reflects encoder units rather than world units.

  if(invert_flag){
    value_world_units *= -1;
  }
  int32_t latch_value_encoder_units = static_cast<int32_t>(output.convert_world_to_block_units(value_world_units)); //this has been inverted (if necessary) and converted into encoder units.

  if(min_or_max == MIN){
    min_latch_value = latch_value_encoder_units;
    min_latch_enabled = true;
  }else{ //MAX
    max_latch_value = latch_value_encoder_units;
    max_latch_enabled = true;
  }
}

void Encoder::run(){
  int32_t encoder_reading = quad_encoder->read(); //raw encoder reading

  float64_t encoder_reading_inverted = static_cast<float64_t>(encoder_reading);

  if(invert_flag){
    encoder_reading_inverted *= -1; //inverted if necessary to match world orientation
  }

  // update latch in encoder unit space
  if(min_latch_enabled){
    if(encoder_reading < min_latch_value){
      encoder_reading = min_latch_value;
      quad_encoder->write(encoder_reading);
      output.reset(encoder_reading_inverted, true); //raw value reset
    }
  }

  if(max_latch_enabled){
    if(encoder_reading > max_latch_value){
      encoder_reading = max_latch_value;
      quad_encoder->write(encoder_reading);
      output.reset(encoder_reading_inverted, true); //raw value reset
    }      
  }

  output.set(encoder_reading_inverted);
  output.push();
}