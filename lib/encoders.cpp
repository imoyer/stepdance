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
    .CHANNEL_B_TEENSY_PIN = 6,
  }
};

Encoder::Encoder(){};

void Encoder::begin(uint8_t encoder_index){
  quad_encoder = all_encoders[encoder_index];
  QuadEncoder_configure(encoder_index);
  quad_encoder->setInitConfig();
  quad_encoder->init();
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

void Encoder::map(DecimalPosition* target_position){
  this->target_position = target_position;
}

void Encoder::map(Transmission* target_transmission){
  this->target_transmission = target_transmission;
}

void Encoder::set_ratio(float input_units, float output_units){
  transfer_ratio = static_cast<float64_t>(output_units / input_units);
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

void Encoder::set(int32_t value){
  if(invert_flag){
    quad_encoder->write(-value);
  }else{
    quad_encoder->write(value);
  }
}

void Encoder::run(){
  int32_t encoder_value = quad_encoder->read();
  int32_t encoder_delta = encoder_value - last_encoder_value;
  float64_t output_value = transfer_ratio * static_cast<float64_t>(encoder_delta);
  if(invert_flag){
    output_value *= -1;
  }
  if(target_position != nullptr){
    *target_position += output_value;
  }
  if(target_transmission != nullptr){
    target_transmission->increment(output_value);
  }
  last_encoder_value = encoder_value;
}