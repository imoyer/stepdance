#include <stdint.h>
#include <sys/types.h>
#include "arm_math.h"
/*
Generators Module of the StepDance Control System

This module contains an assortment of motion stream generators

[More Details to be Added]

A part of the Mixing Metaphors Project
(c) 2025 Ilan Moyer, Jennifer Jacobs, Devon Frost

*/
#include "core.hpp"

#ifndef generators_h //prevent importing twice
#define generators_h

class CircleGenerator : public Plugin{
  public:
    CircleGenerator();
    volatile ControlParameter rotational_speed_rev_per_sec = 1; // circle generation speed
    volatile ControlParameter radius = 0; //radius of circle
    volatile ControlParameter max_radial_speed_mm_per_sec = 10; // maximum radial speed
    void begin();
    void map(Transmission* x_target_transmission, Transmission* y_target_transmission);

  private:
    Transmission* x_output_transmission = nullptr;
    Transmission* y_output_transmission = nullptr;
    volatile float32_t current_radius = 0;
    volatile float64_t current_angle_rad = 0;
  
  protected:
    void run();
};

class VelocityGenerator : public Plugin{
  public:
    VelocityGenerator();
    volatile ControlParameter speed_units_per_sec = 0; // generation velocity
    void begin();
    BlockPort output;
    DecimalPosition target_position = 0;
  
  protected:
    void run();
};

class PositionGenerator : public Plugin{
  // This position generator maintains its own internal state, and will incrementally drive an output transmission to achieve
  // a particular value of its internal position state under the constraints of a maximum velocity.

  public:
    PositionGenerator();

    void begin();
    void set_speed(ControlParameter speed);
    void go(float64_t distance_or_position, uint8_t mode);
    void go(float64_t distance_or_position, uint8_t mode, ControlParameter speed);

    volatile ControlParameter speed_units_per_sec = 0; // generation velocity. This will be used if not explicitly provided by the call to go()

    // BlockPorts
    BlockPort output;

  private:
    DecimalPosition target_position = 0;
    DecimalPosition current_position = 0;

  protected:
    void run();
};

class RatioGenerator1D : public Plugin{
  // Generates an output signal in proportion to one input signal.

  public:
    RatioGenerator1D();

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

class RatioGenerator2D : public Plugin{
  // Generates an output signal in proportion to the linear distance traversed by two inputs.

  public:
    RatioGenerator2D();

    void begin();
    void set_ratio(ControlParameter ratio);
    inline void set_ratio(ControlParameter output_distance, ControlParameter input_distance){
      set_ratio(output_distance / input_distance);
    }

    ControlParameter ratio = 1.0; // output / input

    // BlockPorts
    BlockPort input_1;
    BlockPort input_2;
    BlockPort output;

    DecimalPosition input_1_position;
    DecimalPosition input_2_position;
    DecimalPosition output_position;

  private:


  protected:
    void run();
};

#endif //generators_h