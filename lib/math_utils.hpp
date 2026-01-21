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

#ifndef math_utils_h //prevent importing twice
#define math_utils_h


class Vector2DToAngle : public Plugin{
  public:
    Vector2DToAngle();

    void begin();
    void debugPrint();

    // BlockPort input; 
    BlockPort input_x;
    BlockPort input_y;
    BlockPort output_theta;

    private:
    // DecimalPosition input_position; 
    DecimalPosition input_x_position = 0.0;
    DecimalPosition input_y_position = 0.0;
    DecimalPosition output_theta_position = 0.0;

    protected:

      void run();
};

class MoveDurationToFrequency : public Plugin{
  public:
    MoveDurationToFrequency();

    volatile ControlParameter target_frequency = 1.0;

    void begin();
    void debugPrint();

    // BlockPort input; 
    BlockPort input_move_duration;
    BlockPort output_frequency;

    private:
    // DecimalPosition input_position; 
    DecimalPosition input_move_duration_value = 0.0;
    DecimalPosition output_frequency_value = 0.0;

    protected:

      void run();
};



#endif