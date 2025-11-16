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
#include "digital_in.hpp"

#ifndef homing_h //prevent importing twice
#define homing_h

#define HOMING_DIR_FWD 1.0
#define HOMING_DIR_BWD -1.0

// --- HOMING STATE ---
#define HOMING_AXIS_STATE_BEGIN        0   // beginning homing on the axis
#define HOMING_AXIS_STATE_BACKING_OFF  1   // homing switch was active at start of axis homing.
#define HOMING_AXIS_STATE_SEEKING      2   // traveling in the direction of the homing switch.
#define HOMING_AXIS_STATE_CENTERING    3   // traveling towards the home position.
#define HOMING_STATE_FINISHED          4   // this gets set when all axes are homed.
#define HOMING_STATE_WAITING           5   // this gets set when homing routine is called for all axes.

class HomingAxis : public Plugin {
    public:
        HomingAxis();
        HomingAxis(
            uint8_t limit_switch_port,
            DecimalPosition value_at_limit,
            int direction,
            DecimalPosition velocity
        );

        void begin();

        // void run();

        uint8_t read_state();

        void start_homing_routine();
        void wait_for_homing();

        void set_axis_value();


        BlockPort output;

    protected:
        void run();

    private:
        uint8_t limit_switch_port_number;
        Button limit_switch_button;
        DecimalPosition value_at_limit;
        int homing_direction;
        DecimalPosition homing_velocity;

        uint8_t current_homing_state;

        DecimalPosition current_position = 0;

        void move_forward();
        void move_backward();

};

class Homing : public Plugin{

  public:
    Homing();
    
    void begin();

    void add_axis(
        uint8_t limit_switch_port, 
        DecimalPosition value_at_limit, 
        int direction, 
        DecimalPosition velocity,
        BlockPort *target);

    void start_homing_routine();

  private:
    HomingAxis axes[3];
    int nb_axes = 0;
    int axis_currently_homing = -1;
    DecimalPosition speed;

  protected:
    void loop();
};


#endif