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

#define MAX_NUM_AXES   5

#define HOMING_DIR_FWD 1.0
#define HOMING_DIR_BWD -1.0

// --- HOMING STATE ---
#define HOMING_AXIS_STATE_BEGIN        0   // beginning homing on the axis
#define HOMING_AXIS_STATE_BACKING_OFF  1   // homing switch was active at start of axis homing.
#define HOMING_AXIS_STATE_SEEKING      2   // traveling in the direction of the homing switch.
#define HOMING_AXIS_STATE_CENTERING    3   // traveling towards the home position.
#define HOMING_STATE_FINISHED          4   // this gets set when all axes are homed.
#define HOMING_STATE_WAITING           5   // this gets set when homing routine is called for all axes.


/**
 * \cond  
 *  These functions will be hidden from Doxygen documentation.  
 */
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
/** \endcond */

/**
 * @brief Used for running a homing routine on a machine
 * @details The Homing plugin lets the user define a number of axes they want to home
 * Here's an example of how to configure and run a homing routine :
 * @snippet snippets.cpp Homing
 */
class Homing : public Plugin{

  public:
    Homing();

    /**
     * @brief Initialize the Homing plugin. Must be called during setup().
     */
    void begin();

    /**
     * @brief Register an axis to home. The homing routine will go through the regisered axes
     * and home each one in the order in which they are registered.
     * @param limit_switch_port Stepdance board port for the limit switch.
     * @param value_at_limit Coordinate value we want to assign at the limit switch.
     * @param direction Direction in which the machine should jog (backward or forward?) to hit the switch.
     * @param velocity Speed at which the machine should jog to find the limit.
     * @param target Blockport corresponding to the axis to home.
     */
    void add_axis(
        uint8_t limit_switch_port, 
        DecimalPosition value_at_limit, 
        int direction, 
        DecimalPosition velocity,
        BlockPort *target);

    /**
     * @brief Launch the homing routine (machine will move until it hits the limit switch for each registered axis).
     */
    void start_homing_routine();

  private:
    HomingAxis axes[MAX_NUM_AXES];
    int nb_axes = 0;
    int axis_currently_homing = -1;
    DecimalPosition speed;

  protected:
    void loop();
};


#endif