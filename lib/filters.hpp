#include <stdint.h>
#include <sys/types.h>
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

    void begin(uint8_t mode = INCREMENTAL);
    void set_ratio(ControlParameter ratio);
    inline void set_ratio(ControlParameter output_distance, ControlParameter input_distance){
      set_ratio(output_distance / input_distance);
    }
    void enroll(RPC *rpc, const String& instance_name);

    ControlParameter ratio = 1.0; // output / input

    // BlockPorts
    BlockPort input;
    BlockPort output;

  private:
    DecimalPosition input_position;
    DecimalPosition output_position;
    uint8_t mode = INCREMENTAL;

  protected:
    void run();
};

class ScalingFilter2D : public Plugin{
  // Generates two output signals, each in proportion to an input signal.

  public:
    ScalingFilter2D();

    void begin(uint8_t mode = INCREMENTAL);
    void set_ratio(ControlParameter ratio);
    inline void set_ratio(ControlParameter output_distance, ControlParameter input_distance){
      set_ratio(output_distance / input_distance);
    }
    void enroll(RPC *rpc, const String& instance_name);
    
    ControlParameter ratio = 1.0; // output / input

    // BlockPorts
    BlockPort input_1;
    BlockPort input_2;
    BlockPort output_1;
    BlockPort output_2;

  private:
    DecimalPosition input_1_position;
    DecimalPosition input_2_position;
    DecimalPosition output_1_position;
    DecimalPosition output_2_position;
    uint8_t mode = INCREMENTAL;

  protected:
    void run();
};

#endif