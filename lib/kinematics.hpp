#include "arm_math.h"
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
    void reset(); //TODO: resets the internal state
    void solve_kinematics(); //TODO: solves the relationship between r-t and x-y.
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

class KinematicsFiveBarForward : public Plugin{
  // Converts encoder (or motor) angles to XY coordinates for a five-bar parallel kinematics mechanism (e.g. Parallel SCARA)
  // This was created for the digital pantograph, so we will generally refer to encoders rather than motors.
  // Because this module works with absolute inputs and therefor will not accumulate error, we will use float32_t precision
  // (rather than float64_t) to speed up calculations. Inputs and outputs remain float64_t.
  // For clarity of equations We will use capital letters for certain variables within this function. Externally we
  // maintain consistency with only using capital letters for constants.

  public:
    KinematicsFiveBarForward();
    void begin(float32_t s, float32_t l1, float32_t l2, float32_t l3, float32_t l4, float32_t l5, float32_t a);
    
    BlockPort input_r;
    BlockPort input_l;
    BlockPort output_x;
    BlockPort output_y;

  private:
    DecimalPosition position_r = 0; //right angle, in radians
    DecimalPosition position_l = 0; //left angle, in radians
    DecimalPosition position_x = 0;
    DecimalPosition position_y = 0;

    float32_t S; // encoder separation
    float32_t L1; // right encoder arm length
    float32_t L2; // left encoder arm length
    float32_t L3; // right pivot arm length
    float32_t L4; // left pivot arm length
    float32_t L5; // tool arm length
    float32_t A6; // tool arm angle in rad
    float32_t Xa; // X position of right encoder
    float32_t Ya; // Y position of right encoder
    float32_t Xb; // X position of left encoder
    float32_t Yb; // Y position of left encoder

  protected:
    void run();
};

#endif //kinematics_h