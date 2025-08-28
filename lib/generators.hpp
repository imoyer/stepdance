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

class ThresholdGenerator : public Plugin{
  public:
    ThresholdGenerator();
    volatile ControlParameter threshold = 0;
    BlockPort input_a;
    BlockPort input_b;
    BlockPort output;

    void begin();
    void debugPrint();
    void enable();
    void disable();

    private:
    DecimalPosition input_a_position; 
    DecimalPosition input_b_position; 
    DecimalPosition output_position;
    DecimalPosition current_value;

    protected:
      void run();


};

class WaveGenerator1D : public Plugin{
  public:
    WaveGenerator1D();
    volatile ControlParameter amplitude = 0;
    volatile ControlParameter phase = 1;
    volatile ControlParameter rotational_speed_rev_per_sec = 8;

    void begin();
    void setNoInput();
    void debugPrint();
    void enable();
    void disable();

    BlockPort input;
    BlockPort output;

    private:
    DecimalPosition input_position; 
    DecimalPosition output_position;
    bool no_input = false; //if set to true uses the frame value to update the output


    protected:
      volatile float64_t current_angle_rad = 0;
      volatile float64_t delta = 0;

      void run();
};

//removing because it's currently not needed with WaveGenerator1D
/*class WaveGenerator2D : public Plugin{
  public:
    WaveGenerator2D();
    volatile ControlParameter amplitude = 1.0;
    volatile ControlParameter phase = 0.0;
    volatile ControlParameter rotational_speed_rev_per_sec = 6;
    volatile bool no_input = false; //if set to true uses the frame value to update the output

    void begin();
    void setNoInput();

    void debugPrint();

    BlockPort input; 
    BlockPort output_x;
    BlockPort output_y;

    private:
    DecimalPosition input_position; 
    DecimalPosition output_x_position;
    DecimalPosition output_y_position;

    protected:
      volatile float64_t current_angle_rad = 0;
      volatile float64_t delta = 0;

      void run();
};*/

class CircleGenerator : public Plugin{
  public:
    CircleGenerator();
    volatile ControlParameter radius = 1.0;
    volatile ControlParameter rotational_speed_rev_per_sec = 6;

    void begin();
    void setNoInput();
    void debugPrint();

    BlockPort input; 
    BlockPort output_x;
    BlockPort output_y;

    private:
    DecimalPosition input_position; 
    DecimalPosition output_x_position;
    DecimalPosition output_y_position;
    bool no_input = false; //if set to true uses the frame value to update the output


    protected:
      volatile float64_t current_angle_rad = 0;
      volatile float64_t delta = 0;

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

class PathLengthGenerator2D : public Plugin{
  // Generates an output signal in proportion to the linear distance traversed by two inputs.

  public:
    PathLengthGenerator2D();

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

class PathLengthGenerator3D : public Plugin{
  // Generates an output signal in proportion to the linear distance traversed by three inputs.

  public:
    PathLengthGenerator3D();

    void begin();
    void set_ratio(ControlParameter ratio);
    inline void set_ratio(ControlParameter output_distance, ControlParameter input_distance){
      set_ratio(output_distance / input_distance);
    }

    ControlParameter ratio = 1.0; // output / input

    // BlockPorts
    BlockPort input_1;
    BlockPort input_2;
    BlockPort input_3;
    BlockPort output;

  private:
    DecimalPosition input_1_position;
    DecimalPosition input_2_position;
    DecimalPosition input_3_position;
    DecimalPosition output_position;

  protected:
    void run();
};

#endif //generators_h