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

#ifndef filters_h //prevent importing twice
#define filters_h
/**
 * @brief Generates an output signal in proportion to one input signal.
 * @ingroup filters 
 * @details The ScalingFilter1D class scales an input signal by a specified ratio to produce an output signal. This is useful for scaling a motion trajectory.
 * Here's an example of how to instantiate and configure a ScalingFilter1D:
 * @snippet snippets.cpp ScalingFilter1D
 */
class ScalingFilter1D : public Plugin{
  // Generates an output signal in proportion to one input signal.

  public:
    ScalingFilter1D();
    /**
     * @brief Initializes the ScalingFilter1D with the specified mode.
     * @param mode Mode of operation: INCREMENTAL or ABSOLUTE. Default is INCREMENTAL.
       */
    void begin(uint8_t mode = INCREMENTAL);
    /**
     * @brief Sets the scaling ratio between output and input.
     * @param ratio Scaling ratio (output / input).
     */
    void set_ratio(ControlParameter ratio);
    /**
     * @brief Sets the scaling ratio using output and input distances.
     * @param output_distance Distance in world units for the output.
     * @param input_distance Distance in world units for the input.
     */
    inline void set_ratio(ControlParameter output_distance, ControlParameter input_distance){
      set_ratio(output_distance / input_distance);
    }
    /**
     * \cond
     * These definitions will be hidden from Doxygen documentation.
     */
    void enroll(RPC *rpc, const String& instance_name);

    ControlParameter ratio = 1.0; // output / input
    /**
     * \endcond
     */

    // BlockPorts
    /**
     * @brief Input BlockPort. Map upstream components to the input.
     */
    BlockPort input;
    /**
     * @brief Output BlockPort. Map downstream components to the output.
     */
    BlockPort output;

  private:
    DecimalPosition input_position;
    DecimalPosition output_position;
    uint8_t mode = INCREMENTAL;

  protected:
    void run();
};

/**
 * @brief Generates two output signals, each in proportion to an input signal.
 * @ingroup filters
 * @details The ScalingFilter2D class scales two input signals by a specified ratio to produce two output signals. This is useful for scaling a motion trajectory in two dimensions.
 */
class ScalingFilter2D : public Plugin{
  // Generates two output signals, each in proportion to an input signal.

  public:
    ScalingFilter2D();
/**
 * @brief Initializes the ScalingFilter2D with the specified mode. Must be called to set up the filter.
 * @param mode Mode of operation: INCREMENTAL or ABSOLUTE. Default is INCREMENTAL.
 */
    void begin(uint8_t mode = INCREMENTAL);
    /**
     * @brief Sets the scaling ratio between output and input.
     * @param ratio Scaling ratio (output / input).
     */
    void set_ratio(ControlParameter ratio);
    /**
     * @brief Sets the scaling ratio using output and input distances.
     * @param output_distance Distance in world units for the output.
     * @param input_distance Distance in world units for the input.
     */
    inline void set_ratio(ControlParameter output_distance, ControlParameter input_distance){
      set_ratio(output_distance / input_distance);
    }
    /**
     * \cond
     * These definitions will be hidden from Doxygen documentation.
     */
    void enroll(RPC *rpc, const String& instance_name);
    /** \endcond */
    /**
     * @brief ControlParameter scaling output relative to input (output/input). Can be set by calling set_ratio().
     */
    ControlParameter ratio = 1.0; // output / input

    // BlockPorts
    /**
     * @brief First input BlockPort. Map upstream components to this input.
     */
    BlockPort input_1;
    /**
     * @brief Second input BlockPort. Map upstream components to this input.
     */
    BlockPort input_2;
    /**
     * @brief First output BlockPort corresponding to the scaled input_1. Map downstream components to this output.
     */
    BlockPort output_1;
    /**
     * @brief Second output BlockPort corresponding to the scaled input_2. Map downstream components to this output.
     */
    BlockPort output_2;
    

  private:
    DecimalPosition input_1_position;
    DecimalPosition input_2_position;
    DecimalPosition output_1_position;
    DecimalPosition output_2_position;
    uint8_t mode = INCREMENTAL;

  protected:
    void run();
};

#endif