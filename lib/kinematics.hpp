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

#define HBOT_NUM_OUTPUTS 2
#define HBOT_OUTPUT_A   0
#define HBOT_OUTPUT_B   1

#define POLAR_NUM_OUTPUTS 2
#define POLAR_OUTPUT_X  0
#define POLAR_OUTPUT_Y  1

class KinematicsHBot : public Plugin{
  public:
    KinematicsHBot();
    void begin(); //defaults to incremental mode
    void begin(uint8_t mode);
    void begin(Transmission *output_transmission_a, Transmission *output_transmission_b);
    void begin(uint8_t mode, Transmission *output_transmission_a, Transmission *output_transmission_b);
    void reset(); //resets the internal state
    void map(uint8_t output_index, Transmission* target_transmission); //maps an output to a target transmission
    Transmission input_transmission_x; //transmissions to work externally with internal x and y positions
    Transmission input_transmission_y;

  private:
    uint8_t mode;
    volatile DecimalPosition input_position_x; //internal registers to store target x and y positions
    volatile DecimalPosition input_position_y;
    DecimalPosition get_position_x(); // return current positions in input space, and override the input_transmission get() functions
    DecimalPosition get_position_y();

    Transmission* output_transmissions[HBOT_NUM_OUTPUTS] = {nullptr}; //pointers to the output transmissions

  protected:
    void run();
};

class KinematicsPolarToCartesian : public Plugin{
  public:
    KinematicsPolarToCartesian();
    void begin(); //defaults to incremental mode
    void begin(uint8_t mode);
    void begin(Transmission *output_transmission_x, Transmission *output_transmission_y);
    void begin(uint8_t mode, Transmission *output_transmission_x, Transmission *output_transmission_y);
    void reset(); //resets the internal state
    void map(uint8_t output_index, Transmission* target_transmission); //maps an output to a target transmission
    Transmission input_transmission_r; //transmissions to work externally with internal r and t positions
    Transmission input_transmission_t;

  private:
    uint8_t mode;
    volatile DecimalPosition input_position_r; //internal registers to store target x and y positions
    volatile DecimalPosition input_position_t;
    DecimalPosition get_position_r();
    DecimalPosition get_position_t();

    Transmission* output_transmissions[POLAR_NUM_OUTPUTS]; //pointers to the output transmissions

  protected:
    void run();
};