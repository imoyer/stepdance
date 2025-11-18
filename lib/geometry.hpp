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

#ifndef geometry_h //prevent importing twice
#define geometry_h


class Vector2DToAngle : public Plugin{
  public:
    Vector2DToAngle();

    void begin();
    void debugPrint();

    // void enroll(RPC *rpc, const String& instance_name);

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



#endif