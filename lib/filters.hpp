/*
Filters Module of the StepDance Control System

This module contains an assortment of motion filters

[More Details to be Added]

A part of the Mixing Metaphors Project
(c) 2025 Ilan Moyer, Jennifer Jacobs, Devon Frost

*/
#include "core.hpp"

#ifndef filters_h //prevent importing twice
#define filters_h

class ScalingFilter1D : public Plugin{
  // Generates an output signal in proportion to one input signal.

  public:
    ScalingFilter1D();

    void begin();
    void set_ratio(ControlParameter ratio);
    inline void set_ratio(ControlParameter output_distance, ControlParameter input_distance){
      set_ratio(output_distance / input_distance);
    }

    ControlParameter ratio = 1.0; // output / input

    // BlockPorts
    BlockPort input;
    BlockPort output;

  private:
    DecimalPosition input_position;
    DecimalPosition output_position; 

  protected:
    void run();
};


#endif