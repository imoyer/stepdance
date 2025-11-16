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

/** 
 * @brief KinematicsCoreXY implements [coreXY](https://www.corexy.com/) kinematics transformation.
 * @ingroup kinematics
 * @details The KinematicsCoreXY class converts Cartesian X and Y motion inputs into CoreXY A and B motor outputs. This is useful for controlling CoreXY mechanisms commonly used in 3D printers, plotters and other CNC machines.
 * Here's an example of how to instantiate and configure a KinematicsCoreXY:
 * @snippet snippets.cpp KinematicsCoreXY
*/
class KinematicsCoreXY : public Plugin{
  public:
    KinematicsCoreXY();
    /**
     * @brief Initializes the KinematicsCoreXY. This must be called before using the kinematics.
     */
    void begin();
    /**
     * \cond
     * These definitions will be hidden from Doxygen documentation.
     */
    void enroll(RPC *rpc, const String& instance_name);
    /** \endcond */

    // BlockPorts
    /**
     * @brief Input BlockPort for X axis motion. Map upstream components to this port.
     */
    BlockPort input_x;
    /**
     * @brief Input BlockPort for Y axis motion. Map upstream components to this port.
     */
    BlockPort input_y;
    /**
     * @brief Output BlockPort for A motor motion. Map downstream components to this port.
     */
    BlockPort output_a;
    /**
     * @brief Output BlockPort for B motor motion. Map downstream components to this port.
     */ 
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

/**
 * @brief KinematicsPolarToCartesian converts polar coordinates (radius and angle) to Cartesian coordinates (X and Y).
 * @ingroup kinematics
 * @details The KinematicsPolarToCartesian class transforms polar coordinate inputs into Cartesian X and Y outputs. This is useful for simulating a system with a polar mechanism like a pottery wheel or a lathe.
 * Heres an example of how to instantiate and configure a KinematicsPolarToCartesian component and then use it to drive Cartesian motion from polar inputs:
 * @snippet snippets.cpp KinematicsPolarToCartesian
*/
class KinematicsPolarToCartesian : public Plugin{
  public:
    KinematicsPolarToCartesian();
    /**
     * @brief Initializes the KinematicsPolarToCartesian with an optional fixed radius. If a fixed radius is provided, the radius input BlockPort will be ignored.
     * @param fixed_radius Optional fixed radius value. Default is 0.
     */
    void begin(float64_t fixed_radius = 0); //optional radius parameter
    /**
     * \cond
     * These definitions will be hidden from Doxygen documentation.
     */
    void reset(); //TODO: resets the internal state
    void solve_kinematics(); //TODO: solves the relationship between r-t and x-y.
                              // we do this outside run() so that it can be called during a state reset.
    void enroll(RPC *rpc, const String& instance_name);
    /** \endcond */
    /**
     * @brief Input BlockPort for radius (r) in polar coordinates. Ignored if a fixed radius is set in begin(). Map upstream components to this port.
     */
    BlockPort input_radius;
    /**
     * @brief Input BlockPort for angle (theta) in polar coordinates. Map upstream components to this port.
     */
    BlockPort input_angle;
    /**
     * @brief Output BlockPort for X axis motion. Map downstream components to this port.
     */
    BlockPort output_x;
    /**
     * @brief Output BlockPort for Y axis motion. Map downstream components to this port.
     */
    BlockPort output_y;

  private:
    DecimalPosition position_r = 0;
    DecimalPosition position_a = 0;
    DecimalPosition position_x = 0;
    DecimalPosition position_y = 0;

  protected:
    void run();
};

/**
 * @brief KinematicsFiveBarForward implements forward kinematics for a five-bar parallel mechanism- currently we apply this to a custom built digital pantograph, though it may be useful for other things...
 * @ingroup kinematics
 * @details The KinematicsFiveBarForward class converts encoder angles into Cartesian X and Y coordinates for a five-bar parallel kinematics mechanism, such as a Parallel SCARA. This is useful for controlling mechanisms that utilize a five-bar linkage system.
 * To see an example of a basic module that uses KinematicsFiveBarForward, see the Pantograph example under
 * @ref examples_kinematics "Kinematics Examples".
*/
class KinematicsFiveBarForward : public Plugin{
  // Converts encoder (or motor) angles to XY coordinates for a five-bar parallel kinematics mechanism (e.g. Parallel SCARA)
  // This was created for the digital pantograph, so we will generally refer to encoders rather than motors.
  // Because this module works with absolute inputs and therefor will not accumulate error, we will use float32_t precision
  // (rather than float64_t) to speed up calculations. Inputs and outputs remain float64_t.
  // For clarity of equations We will use capital letters for certain variables within this function. Externally we
  // maintain consistency with only using capital letters for constants.

  public:
    KinematicsFiveBarForward();
    /**
     * @brief Initializes the KinematicsFiveBarForward with the specified mechanism parameters.
     * @param s Separation between the two encoders (distance between encoder pivots).
     * @param l1 Length of the right encoder arm.
     * @param l2 Length of the left encoder arm.
     * @param l3 Length of the right pivot arm.
     * @param l4 Length of the left pivot arm.
     * @param l5 Length of the tool arm.
     * @param a Tool arm angle in radians.
     */
    void begin(float32_t s, float32_t l1, float32_t l2, float32_t l3, float32_t l4, float32_t l5, float32_t a);
    /**
     * \cond
     * These definitions will be hidden from Doxygen documentation.
     */
    void enroll(RPC *rpc, const String& instance_name);
    
    /** \endcond */
    /**
     * @brief Input BlockPort for the right encoder angle in radians. Map upstream components to this port.
     */
    BlockPort input_r;
    /**
     * @brief Input BlockPort for the left encoder angle in radians. Map upstream components to this port.
     */
    BlockPort input_l;
    /**
     * @brief Output BlockPort for X axis motion. Map downstream components to this port.
     */
    BlockPort output_x;
    /**
     * @brief Output BlockPort for Y axis motion. Map downstream components to this port.
     */
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

using KinematicsLever = KinematicsPolarToCartesian;

#endif //kinematics_h