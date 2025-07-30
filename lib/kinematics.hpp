/*
Kinematics Module of the StepDance Control System

This module provides a variety of mechanism kinematics to go from one motion space (e.g. XY) to another (e.g. AB or RT) via e.g. an h-bot or polar mechanism, etc.

[More Details to be Added]

A part of the Mixing Metaphors Project
(c) 2025 Ilan Moyer, Jennifer Jacobs, Devon Frost
*/

#include "core.hpp"

#ifndef kinematics_h //prevent importing twice
#define kinematics_h

class KinematicsCoreXY : public Plugin{
  public:
    KinematicsCoreXY();
    void begin();

    // BlockPorts
    BlockPort input_x;
    BlockPort input_y;
    BlockPort output_a;
    BlockPort output_b;

  private:
    volatile DecimalPosition position_x = 0; //internal registers to store state positions
    volatile DecimalPosition position_y = 0;
    volatile DecimalPosition position_a = 0;
    volatile DecimalPosition position_b = 0;

    // DecimalPosition get_position_x(); // return current positions in input space, and override the input_transmission get() functions
    // DecimalPosition get_position_y();

  protected:
    void run();
};

class KinematicsPolarToCartesian : public Plugin{
  public:
    KinematicsPolarToCartesian();
    void begin();
    void reset(); //resets the internal state
    void solve_kinematics(); //solves the relationship between r-t and x-y.
                              // we do this outside run() so that it can be called during a state reset.
    
    BlockPort input_radius;
    BlockPort input_angle;
    BlockPort output_x;
    BlockPort output_y;

  private:
    DecimalPosition position_r = 0;
    DecimalPosition position_a = 0;
    DecimalPosition position_x = 0;
    DecimalPosition position_y = 0;

  protected:
    void run();
};

#endif //kinematics_h