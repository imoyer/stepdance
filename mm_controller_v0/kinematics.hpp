/*
Kinematics Module of the StepDance Control System

This module provides a variety of mechanism kinematics to go from one motion space (e.g. XY) to another (e.g. AB or RT) via e.g. an h-bot or polar mechanism, etc.

[More Details to be Added]

A part of the Mixing Metaphors Project
(c) 2025 Ilan Moyer, Jennifer Jacobs, Devon Frost
*/

#include "core.hpp"

// Kinematics Mode
// If incremental, the kinematic module will:
//  1) read the input positions
//  2) convert them to output positions
//  3) INCREMENT the target transmissions by these output positions
//  4) clear the input positions
//
// If absolute, the kinematics module will:
//  1) read the input positions
//  2) convert them into output positions
//  3) SET the target transmissions to these output positions
#define KINEMATICS_MODE_INCREMENTAL   0
#define KINEMATICS_MODE_ABSOLUTE      1

class KinematicsHBot : public Plugin{
  public:
    KinematicsHBot();
    void begin(); //defaults to incremental mode
    void begin(uint8_t mode);
    void begin(Transmission *output_transmission_a, Transmission *output_transmission_b);
    void begin(uint8_t mode, Transmission *output_transmission_a, Transmission *output_transmission_b);
    void reset(); //resets the internal state
    Transmission input_transmission_x; //transmissions to work externally with internal x and y positions
    Transmission input_transmission_y;

  private:
    uint8_t mode;
    volatile DecimalPosition input_position_x; //internal registers to store target x and y positions
    volatile DecimalPosition input_position_y;

    Transmission *output_transmission_a; //pointers to the output transmissions
    Transmission *output_transmission_b;

  protected:
    void run();
};