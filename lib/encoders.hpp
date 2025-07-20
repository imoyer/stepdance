/*
Encoders Module of the StepDance Control System

This module interfaces with encoder inputs

[More Details to be Added]

A part of the Mixing Metaphors Project
(c) 2025 Ilan Moyer, Jennifer Jacobs, Devon Frost

*/
#include "core.hpp"
#include <QuadEncoder.h>

#ifndef encoders_h //prevent importing twice
#define encoders_h

#define MAX_NUM_ENCODERS 3 //this is the most we can get out of a stepdance board (ENC1, ENC2, and INPUT_A)
#define ENCODER_1 0
#define ENCODER_2 1
#define ENCODER_A 2 //the A stepdance input can double as a third encoder input

struct encoder_info_struct{
  uint8_t CHANNEL_A_TEENSY_PIN;
  uint8_t CHANNEL_B_TEENSY_PIN;
};

class Encoder : public Plugin{
  public:
    Encoder();
    void begin(uint8_t encoder_index);
    void map(DecimalPosition* target_position); //can target a position register directly
    void map(Transmission* target_transmission); //or can target a transmission
    void set_ratio(float input_units, float output_units); //sets the transmission ratio
    void invert();
    int32_t read(); //directly reads the instantaneous encoder value
    void reset(); //resets the encoder value to zero
    void set(int32_t value); //resets the encoder value to a provided value
    int32_t last_encoder_value = 0;

  private:
    static const struct encoder_info_struct encoder_info[]; //stores configuration for all available encoder ports
    static QuadEncoder* all_encoders[MAX_NUM_ENCODERS];
    void QuadEncoder_configure(uint8_t encoder_index); //applies the configuration stored in encoder_info[encoder_index] to the QuadEncoder object.
    QuadEncoder* quad_encoder; //pointer to the active quad encoder for this instance
    DecimalPosition* target_position = nullptr;
    Transmission* target_transmission = nullptr;
    float64_t transfer_ratio = 1.0;
    uint8_t invert_flag = 0;

  protected:
    void run();
};


#endif //encoders_h