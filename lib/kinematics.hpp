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
    void begin(); //defaults to incremental mode
    void begin(uint8_t mode);

    // BlockPorts
    BlockPort input_x;
    BlockPort input_y;
    BlockPort output_a;
    BlockPort output_b;

  private:
    uint8_t mode;
    volatile DecimalPosition position_x = 0; //internal registers to store state positions
    volatile DecimalPosition position_y = 0;
    volatile DecimalPosition position_a = 0;
    volatile DecimalPosition position_b = 0;

    // DecimalPosition get_position_x(); // return current positions in input space, and override the input_transmission get() functions
    // DecimalPosition get_position_y();

  protected:
    void run();
};

#define POLAR_NUM_OUTPUTS 2
#define KINEMATICS_MODE_INCREMENTAL 0
#define KINEMATICS_MODE_ABSOLUTE 1
#define POLAR_OUTPUT_X 0
#define POLAR_OUTPUT_Y 1

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

#endif //kinematics_h