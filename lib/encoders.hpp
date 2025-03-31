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

#define MAX_NUM_ENCODERS 2
#define ENCODER_1 0
#define ENCODER_2 1

#define ENCODER_1A_PIN  0
#define ENCODER_1B_PIN  1
#define ENCODER_2A_PIN  2
#define ENCODER_2B_PIN  3

class Encoder : public Plugin{
  public:
    Encoder();
    void begin(uint8_t encoder_index);
    void map(DecimalPosition* target_position); //can target a position register directly
    void map(Transmission* target_transmission); //or can target a transmission
    void set_ratio(float input_units, float output_units); //sets the transmission ratio
    void invert();

  private:
    static QuadEncoder* all_encoders[MAX_NUM_ENCODERS];
    QuadEncoder* quad_encoder; //pointer to the active quad encoder for this instance
    DecimalPosition* target_position = nullptr;
    Transmission* target_transmission = nullptr;
    float64_t transfer_ratio = 1.0;
    int32_t last_encoder_value = 0;
    uint8_t invert_flag = 0;

  protected:
    void run();
};


#endif //encoders_h