#include <stdint.h>
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
/** \cond */
 /**
   * This struct will be hidden from Doxygen documentation.
   */ 
struct encoder_info_struct{
  uint8_t CHANNEL_A_TEENSY_PIN;
  uint8_t CHANNEL_B_TEENSY_PIN;
};  
/** \endcond */
class Encoder : public Plugin{
  public:
    Encoder();
    void begin(uint8_t encoder_index);
    void invert();
    DecimalPosition read(); //returns the last read encoder value, in realworld units. This is a shortcut for output.read(ABSOLUTE).
    void reset(); //resets the encoder value to zero
    void set(DecimalPosition value); //resets the encoder value to a provided value, in world units
    void set_latch(DecimalPosition value_world_units, uint8_t min_or_max);
    void set_ratio(float output_units, float encoder_units = 1.0);

    // BlockPorts
    BlockPort output;

    bool min_latch_enabled = false; //enables latching in the negative direction (encoder units)
    bool max_latch_enabled = false; //enables latching in the positive direction
    int32_t min_latch_value = 0; //encoder units
    int32_t max_latch_value = 0;

    void enroll(RPC *rpc, const String& instance_name);

  private:
    static const struct encoder_info_struct encoder_info[]; //stores configuration for all available encoder ports
    static QuadEncoder* all_encoders[MAX_NUM_ENCODERS];
    void QuadEncoder_configure(uint8_t encoder_index); //applies the configuration stored in encoder_info[encoder_index] to the QuadEncoder object.
    QuadEncoder* quad_encoder; //pointer to the active quad encoder for this instance
    uint8_t invert_flag = 0;
    DecimalPosition encoder_value; //stores the encoder position as a DecimalPosition. This gets updated at the beginning of each call to run();

  protected:
    void run();
};


#endif //encoders_h