#include <stdint.h>
#include <sys/types.h>
#include "arm_math.h"
/*
Generators Module of the StepDance Control System

This module contains an assortment of motion stream generators

[More Details to be Added]

A part of the Mixing Metaphors Project
(c) 2025 Ilan Moyer, Jennifer Jacobs, Devon Frost

*/
#include "core.hpp"

#ifndef generators_h //prevent importing twice
#define generators_h


/**
 * @brief Used for generating output signals based on upper and lower thresholds.
 * @ingroup generators
 * 
 * @details The ThresholdGenerator monitors an input signal and triggers callbacks or clamps output when the input crosses specified upper or lower thresholds. It can be used  to restrict a motion signal within defined bounds and/or to trigger events when those bounds are exceeded.  
 * Here's an example of how to instantiate and configure a ThresholdGenerator:
 * @snippet snippets.cpp ThresholdGenerator
 */
class ThresholdGenerator : public Plugin{
  public:
  /**
   * @brief Default constructor for ThresholdGenerator. Does not initialize board hardware. Call begin() to set up the generator.
   **/
    ThresholdGenerator();
  /**
   * @brief Initialize the ThresholdGenerator.
   */
    void begin();
    /**
     * @brief Print debug information about the ThresholdGenerator to the Serial console.
     */ 
    void debugPrint();
    /**
     * @brief Enable the ThresholdGenerator.
     */
    void enable();
    /**
     * @brief Disable the ThresholdGenerator.
     */
    void disable();
    /**
     * @brief Set the callback function to be called when the input crosses the lower threshold.
     * @param callback_function Pointer to the callback function.
     */
    void setLowerCallback(void (*callback_function)());
    /**
     * @brief Set the callback function to be called when the input crosses the upper threshold.
     * @param callback_function Pointer to the callback function.
     */
    void setUpperCallback(void (*callback_function)());
    /**
     * @brief Set the upper threshold value and optionally enable clamping to the upper threshold. This must be called to activate the upper threshold.
     * @param upper_threshold The upper threshold value.
     * @param clamp_to_upper If true, output will be clamped to the upper threshold when exceeded.
     */
    void setUpperThreshold(float64_t upper_threshold, bool clamp_to_upper = false);
    /**
     * @brief Set the lower threshold value and optionally enable clamping to the lower threshold. This must be called to activate the lower threshold.
     * @param lower_threshold The lower threshold value.
     * @param clamp_to_lower If true, output will be clamped to the lower threshold when exceeded.
     */
    void setLowerThreshold(float64_t lower_threshold, bool clamp_to_lower = false);
    /**
     * @brief Clear the upper threshold, disabling its effect.
     */
    void clearUpperThreshold();
    /**
     * @brief Clear the lower threshold, disabling its effect.
     */
    void clearLowerThreshold();
  
    /**
     * @brief Input BlockPort for the ThresholdGenerator. This is what you map to with an upstream component.
     * @code
     * input_a.output_x.map(threshold_gen.input); // Map SIGNAL_X to ThresholdGenerator input
     * @endcode
     */
    BlockPort input;
    /**
     * @brief Output BlockPort for the ThresholdGenerator. This is what you map from to a downstream component. If clamping is enabled, the output will be clamped to the threshold values.
     * @code
     * threshold_gen.output.map(channelX.input_target_position); // Map ThresholdGenerator output to ChannelX target position
     * @endcode
     */
    BlockPort output;
    /** 
     * \cond
     * These functions will be hidden from Doxygen documentation.
     */
  void enroll(RPC *rpc, const String& instance_name);
/** \endcond */
    private:
    DecimalPosition input_position;
    DecimalPosition output_position;
    volatile ControlParameter _lower_threshold = 0;  
    volatile ControlParameter _upper_threshold = 0;
    void (*callback_on_lower_threshold)() = nullptr;
    void (*callback_on_upper_threshold)() = nullptr;
    bool clamp_lower = false;
    bool clamp_upper = false;
    bool upper_set = false;
    bool lower_set = false;



    protected:
      void run();


};

/**
 * @brief Used for generating a sinusoidal waveform signal.
 * @ingroup generators
 * 
 * A WaveGenerator1D produces a sinusoidal waveform output based on an input position or internal frame count. It allows control over amplitude, phase, and frequency, making it useful for generating or modifying motion signals with an oscillation.
 * Here's an example of how to instantiate and configure a WaveGenerator1D:
 * @snippet snippets.cpp WaveGenerator1D
 */
class WaveGenerator1D : public Plugin{
  public:
  /**
   * @brief Default constructor for WaveGenerator1D. Does not initialize board hardware. Call begin() to set up the generator.
   **/
    WaveGenerator1D();
  /**
   * @brief ControlParameter for the amplitude of the waveform- the distance from the center line of the wave form to the top or bottom of the wave crest or trough. You can set it directly or map an input to it (e.g. from an analog input).
   */
    volatile ControlParameter amplitude = 0;
    /**
     * @brief ControlParameter for the phase of the waveform in radians. Phase is the offset of the waveform from its starting point. You can set it directly or map an input to it.
     */
    volatile ControlParameter phase = 0.0;
    /**
     * @brief ControlParameter for the frequency of the waveform in revolutions per second (e.g. number of waves within a period). You can set it directly or map an input to it.
     */
    volatile ControlParameter frequency = 8;
    /**
     * @brief Initialize the WaveGenerator1D. Must be called to set up the generator. 
     */
    void begin();
    /**
     * @brief Set the generator to use internal frame count for waveform generation instead of an input signal. This means the generator will produce a continuous waveform based on the frame rate and does not need to be mapped to an input.
     */
    void setNoInput();
    /**
     * @brief Print debug information about the WaveGenerator1D to the Serial console.
     */
    void debugPrint();
    /**
     * @brief Enable the WaveGenerator1D.
     */
    void enable();
    /**
     * @brief Disable the WaveGenerator1D.
     */
    void disable();
    /**
     * @brief Input BlockPort for the WaveGenerator1D. This is what you map to with an upstream component. If setNoInput() is called, this input is ignored.
     * @code
     * input_a.output_x.map(wave_gen.input); // Map SIGNAL_X to WaveGenerator1D input
      * @endcode
     */
    BlockPort input;
    /**
     * @brief Output BlockPort for the WaveGenerator1D. This is what you map from to a downstream component.
     * @code
     * wave_gen.output.map(channelX.input_target_position); // Map WaveGenerator1D output to ChannelX target position
     * @endcode
     */
    BlockPort output;
    /** 
     * \cond
     * These functions will be hidden from Doxygen documentation.
     */   
    void enroll(RPC *rpc, const String& instance_name);
/** \endcond */

   private:

    private:
    DecimalPosition input_position; 
    DecimalPosition output_position;
    bool no_input = false; //if set to true uses the frame value to update the output


    protected:
      volatile float64_t current_angle_rad = 0;
      volatile float64_t delta = 0;

      void run();
};

/**
 * @brief Used for generating a 2D sinusoidal waveform signal along a given 2D direction.
 * @ingroup generators
 * 
 * A WaveGenerator2D produces a sinusoidal waveform output based on 
 * an input 2D direction (provided as an angle input_theta).
 * It allows control over amplitude, phase, and frequency, making it useful for generating or modifying motion signals with an oscillation.
 * Here's an example of how to instantiate and configure a WaveGenerator2D:
 * @snippet snippets.cpp WaveGenerator2D
 */
class WaveGenerator2D : public Plugin{
  public:
    WaveGenerator2D();
    /**
     * @brief Wave amplitude
     */
    volatile ControlParameter amplitude = 1.0;
    // volatile ControlParameter frequency = 10.0; // I set this up as a blockport below to be able to pass in variable values

    /**
     * @brief Initialize the time-based interpolator. This must be called to set up the interpolator.
     */
    void begin();

    /**
     * \cond
     * Hidden from Doxygen.
     */
    void debugPrint();
    void enroll(RPC *rpc, const String& instance_name);
    /** \endcond */

    /**
     * @brief Call to configure the wave generator to not need/user input_t and instead just base the wave parameter on system clock.
     */
    void setNoInput();


    // This exposes the frequency as a blockport,
    // can be useful to vary it depending on the length of the current segment
    // I also considered making it a constant. It seems like it'd be nice to have both options
    // maybe we need a "constant generator" to be able to plug in either a varying value or a constant
    /**
     * @brief Input BlockPort to map a variable wave frequency value.
     * @code
     * // Using the MoveDurationToFrequency utility plugin (durationToFreq), 
     * // compute the closest matching frequency to the target frequency (1.0)
     * // such that we obtain an integer number of half-periods over 
     * // the entire current move segment from the TimeBasedInterpolator (tbi)
     * durationToFreq.input_move_duration.map(&tbi.output_duration, ABSOLUTE);
     * durationToFreq.output_frequency.map(&wave2d_gen.input_frequency);
     * durationToFreq.target_frequency = 1.0;
     * @endcode
     */
    BlockPort input_frequency;

    /**
     * @brief Input BlockPort to map a parameter that will be used to compute the wave.
     * @code
     * // map the parameterization of a TimeBasedInterpolator motion 
     * // to the parameter used to compute the wave2D function
     * tbi.output_parameter.map(&wave2d_gen.input_t, ABSOLUTE);
     * @endcode
     */
    BlockPort input_t;

    /**
     * @brief Input BlockPort to map the desired 2D orientation of the wave.
     * This is given as an angle in radians, indicating the desired orientation of the horizontal axis of the wave on the 2D plane.
     * @code
     * // Convert the XY vector given by the TimeBasedInterpolator motion into an angle, 
     * // and pass it to the WaveGenerator2D to set the direction of the wave to match that of the TBI move.
     * vec2angle.input_x.map(&tbi.output_x);
     * vec2angle.input_y.map(&tbi.output_y);
     * vec2angle.output_theta.map(&wave2d_gen.input_theta, ABSOLUTE);
     * @endcode
     */
    BlockPort input_theta;

    /**
     * @brief Output BlockPort for the generated position signal on axis X.
     * @code
     * wave2d_gen.output_x.map(&kinematics.input_x); // Control the X axis of a machine
     * @endcode
     */
    BlockPort output_x;

    /**
     * @brief Output BlockPort for the generated position signal on axis Y.
     * @code
     * wave2d_gen.output_y.map(&kinematics.input_y); // Control the Y axis of a machine
     * @endcode
     */
    BlockPort output_y;

    private:
      DecimalPosition input_frequency_value;
      DecimalPosition input_t_value;

      DecimalPosition input_theta_value;
      DecimalPosition output_x_position;
      DecimalPosition output_y_position;

      bool no_input = false; //if set to true uses the frame value to update the output

    protected:
      volatile float64_t current_angle_rad = 0;

      void run();
};

/**
 * @brief Used for generating circular motion signals.
 * @ingroup generators
 * 
 * A CircleGenerator produces a motion signal that traverses a circle defined by a radius and a rotational speed. It can be driven
 * by an input signal (phase) or run autonomously using the internal frame count when setNoInput() is called.
 * The two outputs, output_x and output_y, provide the Cartesian components of the circle.
 * Here's an example of how to instantiate and configure a CircleGenerator:
 * @snippet snippets.cpp CircleGenerator
 */
class CircleGenerator : public Plugin{
  public:
    CircleGenerator();
    /**
     * @brief ControlParameter for the circle radius. You can set it directly or map an input to it.
     */
    volatile ControlParameter radius = 1.0;
    /**
     * @brief ControlParameter for the rotational speed, in revolutions per second.
     * You can set it directly or map an input to it.
     */
    volatile ControlParameter rotational_speed_rev_per_sec = 6; //starts off
    /**
     * @brief Initialize the CircleGenerator. Must be called to set up the generator.
     */
    void begin();
    /**
     * @brief Use internal frame count for phase instead of an input signal. This will allow the circle generator to update automatically based on frame rate. When enabled, input is ignored.
     */
    void setNoInput();
    /**
     * @brief Print debug information about the CircleGenerator to the Serial console.
     */
    void debugPrint();
    /**
     * \cond
     * Hidden from Doxygen: enrollment for RPC exposure.
     */
    void enroll(RPC *rpc, const String& instance_name);
    /** \endcond */
    /**
     * @brief Optional input BlockPort (phase). If setNoInput() is called, this input is ignored.
     * @code
     * input_a.output_x.map(circle.input); // Map a phase signal to CircleGenerator input
     * @endcode
     */
    BlockPort input; 
    /**
     * @brief Output BlockPort for X component of the circular path.
     * @code
     * circle.output_x.map(channelX.input_target_position); // Map X output to X-axis channel
     * @endcode
     */
    BlockPort output_x;
    /**
     * @brief Output BlockPort for Y component of the circular path.
     * @code
     * circle.output_y.map(channelY.input_target_position); // Map Y output to Y-axis channel
     * @endcode
     */
    BlockPort output_y;

    private:
    DecimalPosition input_position; 
    DecimalPosition output_x_position;
    DecimalPosition output_y_position;
    bool no_input = false; //if set to true uses the frame value to update the output


    protected:
      volatile float64_t current_angle_rad = 0;
      volatile float64_t delta = 0;

      void run();
};

/**
 * @brief Used for generating a position signal that advances at a constant speed.
 * @ingroup generators
 * 
 * A VelocityGenerator maintains an internal target position and increments it over time at a specified
 * speed (units per second). This can be used to move a downstream component at a specified rate.
 * Here's an example of how to instantiate and configure a VelocityGenerator:
 * @snippet snippets.cpp VelocityGenerator
 */
class VelocityGenerator : public Plugin{
  public:
    VelocityGenerator();
    /**
     * @brief ControlParameter specifying the generation speed in units per second.
     * You can set it directly or map an input to it.
     */
    volatile ControlParameter speed_units_per_sec = 0; // generation velocity
    /**
     * @brief Initialize the VelocityGenerator. Must be called to set up the generator.
     */
    void begin();
    /**
     * \cond
     * Hidden from Doxygen: enrollment for RPC exposure.
     */
    void enroll(RPC *rpc, const String& instance_name);
    /** \endcond */
    /**
     * @brief Output BlockPort for the generated position signal.
     * @code
     * velocity_gen.output.map(channelX.input_target_position); // Drive a channel at constant speed
     * @endcode
     */
    BlockPort output;
    /**
     * \cond
     * Internal state tracking the generator's target position. Will be hidden from Doxygen documentation.
     */
    DecimalPosition target_position = 0;
    /** \endcond */
  
  protected:
    void run();
};

/**
 * @brief Used for generating position signals based on a target position and maximum velocity.
 * @ingroup generators
 * 
 * A PositionGenerator maintains an internal position state and incrementally drives an output to reach a specified target position under the constraints of a maximum velocity. It is useful for generating smooth trajectories to an absolute or relative position. Position generators do not require an input signal, as they generate motion based on internal state and commands.
 * Here's an example of how to instantiate and configure a PositionGenerator:
 * @snippet snippets.cpp PositionGenerator
 */
class PositionGenerator : public Plugin{
  // This position generator maintains its own internal state, and will incrementally drive an output transmission to achieve
  // a particular value of its internal position state under the constraints of a maximum velocity.

  public:
  /**
   * @brief Default constructor for PositionGenerator. Does not initialize the Generator. Call begin() to set up the generator.
   */
    PositionGenerator();
/**
     * @brief Initialize the PositionGenerator. Must be called to set up the generator.
     */
    void begin();
    /**
     * @brief Set the speed (maximum velocity) for the PositionGenerator.
     * @param speed The speed in units per second.
     */
    void set_speed(ControlParameter speed);
    /**
     * @brief Command the PositionGenerator to move to a specified distance or position.
     * @param distance_or_position The target distance (incremental) or position (absolute).
     * @param mode The mode of operation: INCREMENTAL or ABSOLUTE.
     */
    void go(float64_t distance_or_position, uint8_t mode);
    /**
     * @brief Command the PositionGenerator to move to a specified distance or position with a specified speed.
     * @param distance_or_position The target distance (incremental) or position (absolute).
     * @param mode The mode of operation: INCREMENTAL, ABSOLUTE, or GLOBAL
     * @param speed The speed in units per second.
     */
    void go(float64_t distance_or_position, uint8_t mode, ControlParameter speed);
    /**
     * @brief ControlParameter that determines the speed of the PositionGenerator. Will not be used if speed is provided in the go() command.
     */
    volatile ControlParameter speed_units_per_sec = 0; // generation velocity. This will be used if not explicitly provided by the call to go()

    // BlockPorts
    /**
     * @brief Output BlockPort for the PositionGenerator. This is what you map from to a downstream component.
     * @code
     * position_gen.output.map(channelX.input_target_position); // Map PositionGenerator output to ChannelX target position
     * @endcode
     */
    BlockPort output;
/**
 * \cond  
 *  These functions will be hidden from Doxygen documentation.  
 */
    void enroll(RPC *rpc, const String& instance_name);
/** \endcond */

  private:
    DecimalPosition target_position = 0;
    DecimalPosition current_position = 0;

  protected:
    void run();
};

/**
 * @brief Used for generating an output proportional to the 2D path length traveled by two inputs.
 * @ingroup generators
 * 
 * PathLengthGenerator2D measures the incremental Euclidean distance traversed by two input signals
 * and produces an output that is proportional to that path length by a configurable ratio.
 * This is useful for applications like controlling extrusion rate of a 3D printer based on XY machine movement.
 * Here's an example of how to instantiate and configure a PathLengthGenerator2D:
 * @snippet snippets.cpp PathLengthGenerator2D
 */
class PathLengthGenerator2D : public Plugin{
  // Generates an output signal in proportion to the linear distance traversed by two inputs.

  public:
    PathLengthGenerator2D();
    /**
     * @brief Initialize the PathLengthGenerator2D. Must be called to set up the generator.
     */
    void begin();
    /**
     * @brief Set the ratio between output distance and input path length.
     * @param ratio The proportionality constant (output / input).
     */
    void set_ratio(ControlParameter ratio);
    inline void set_ratio(ControlParameter output_distance, ControlParameter input_distance){
      set_ratio(output_distance / input_distance);
    }
    /**
     * @brief Configure the generator for circular motion (e.g., from CircleGenerator).
     * Sets the ratio so that the output advances by the specified distance per complete revolution.
     * @param circle_radius The radius of the circle being traced by the inputs.
     * @param output_per_revolution Distance the output should advance per full circle (e.g., 1.0 for 1mm per revolution).
     * 
     * For example, to move Z-axis 1mm for each complete circle of radius 10mm:
     * @code
     * path_gen.set_ratio_for_circle(10.0, 1.0);
     * @endcode
     */
    void set_ratio_for_circle(ControlParameter circle_radius, ControlParameter output_per_revolution);
    /**
     * \cond
     * Hidden from Doxygen: enrollment for RPC exposure.
     */
    void enroll(RPC *rpc, const String& instance_name);
    /** \endcond */
    /**
     * @brief ControlParameter scaling output relative to input path length (output/input).
     */
    ControlParameter ratio = 1.0; // output / input
    // BlockPorts
    /**
     * @brief First input BlockPort (e.g., X position).
     */
    BlockPort input_1;
    /**
     * @brief Second input BlockPort (e.g., Y position).
     */
    BlockPort input_2;
    /**
     * @brief Output BlockPort proportional to 2D path length.
     * @code
     * xy_path.output.map(channelE.input_target_position); // Map to downstream target
     * @endcode
     */
    BlockPort output;

    DecimalPosition input_1_position;
    DecimalPosition input_2_position;
    DecimalPosition output_position;

  private:


  protected:
    void run();
};

/**
 * @brief Used for generating an output proportional to the 3D path length traveled by three inputs.
 * @ingroup generators
 * 
 * PathLengthGenerator3D measures the incremental Euclidean distance traversed by three input signals
 * and produces an output proportional to that path length by a configurable ratio.
 * Here's an example of how to instantiate and configure a PathLengthGenerator3D:
 * @snippet snippets.cpp PathLengthGenerator3D
 */
class PathLengthGenerator3D : public Plugin{
  // Generates an output signal in proportion to the linear distance traversed by three inputs.

  public:
    PathLengthGenerator3D();
    /**
     * @brief Initialize the PathLengthGenerator3D. Must be called to set up the generator.
     */
    void begin();
    /**
     * @brief Set the ratio between output distance and input path length.
     * @param ratio The proportionality constant (output / input).
     */
    void set_ratio(ControlParameter ratio);
    inline void set_ratio(ControlParameter output_distance, ControlParameter input_distance){
      set_ratio(output_distance / input_distance);
    }
    /**
     * \cond
     * Hidden from Doxygen: enrollment for RPC exposure.
     */
    void enroll(RPC *rpc, const String& instance_name);
    /** \endcond */
    /**
     * @brief ControlParameter scaling output relative to input path length (output/input).
     */
    ControlParameter ratio = 1.0; // output / input
    // BlockPorts
    /**
     * @brief First input BlockPort (e.g., X position).
     */
    BlockPort input_1;
    /**
     * @brief Second input BlockPort (e.g., Y position).
     */
    BlockPort input_2;
    /**
     * @brief Third input BlockPort (e.g., Z position).
     */
    BlockPort input_3;
    /**
     * @brief Output BlockPort proportional to 3D path length.
     */
    BlockPort output;

  private:
    DecimalPosition input_1_position;
    DecimalPosition input_2_position;
    DecimalPosition input_3_position;
    DecimalPosition output_position;

  protected:
    void run();
};




class SerialControlTrack {
  public:
    SerialControlTrack();
    void begin();
    void run();
    void loop(int input);

    BlockPort output;

    // TODO: adjusting these values is probably pretty important
    // It will affect how much control we can have over the velocity on the application side
    // static const int buffer_queue_size = 2;
    static constexpr float64_t MAX_STEPS_PER_LOOP = 25;
    // static constexpr float64_t MAX_RANGE = 50;

  private:
    DecimalPosition output_position;
    // DecimalPosition received_target_delta[buffer_queue_size];
    // int current_read_idx_in_queue = -1;
    // int current_write_idx_in_queue = -1;

    // DecimalPosition delta_to_current_target = 0.0;

    DecimalPosition current_target_velocity;
};

class SerialConnectionGenerator : public Plugin{
  // Generates output signals based on received values from a serial connection

  public:
    SerialConnectionGenerator();

    void begin();

    static const uint8_t NUM_CHANNELS = 3;
    SerialControlTrack controlled_tracks[NUM_CHANNELS];

    // BlockPorts (for now fixed to 2D XY)
    BlockPort& output_1 = controlled_tracks[0].output;
    BlockPort& output_2 = controlled_tracks[1].output;
    BlockPort& output_3 = controlled_tracks[2].output;


  protected:
    void loop();
    void run();

};

#endif //generators_h