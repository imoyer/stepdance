#include "encoders.hpp"

#ifndef NO_ENCODERS //we can define this to avoid using the encoder hardware interface
                    //unfortunately the quadencoder library configures the XBAR from within the constructor, so there's no way to incorporate this library without affecting pin mapping.
  QuadEncoder quad_encoder_1(1, ENCODER_1A_PIN, ENCODER_1B_PIN, 1); //no pullups required
  QuadEncoder quad_encoder_2(2, ENCODER_2A_PIN, ENCODER_2B_PIN, 1);
  QuadEncoder * Encoder::all_encoders[MAX_NUM_ENCODERS] = {&quad_encoder_1, &quad_encoder_2};
#else
  QuadEncoder * Encoder::all_encoders[MAX_NUM_ENCODERS] = {nullptr, nullptr};
#endif

Encoder::Encoder(){};

void Encoder::begin(uint8_t encoder_index){
  quad_encoder = all_encoders[encoder_index];
  quad_encoder->setInitConfig();
  quad_encoder->init();
  register_plugin();
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