#include <sys/_stdint.h>
#include "arm_math.h"
/*
Interpolator Module of the StepDance Control System

This module provides buffered motion interpretation.

[More Details to be Added]

A part of the Mixing Metaphors Project
(c) 2025 Ilan Moyer, Jennifer Jacobs, Devon Frost
*/

#include "core.hpp"

#ifndef interpolators_h //prevent importing twice
#define interpolators_h

// TBI stands for "TIME_BASED_INTERPRETER"
#define TBI_BLOCK_QUEUE_SIZE   100 //we'll start here. Each block currently requires 33 bytes of RAM
#define TBI_NUM_AXES  7 //number of axes to support in the interpolator

#define TBI_AXIS_INACTIVE 0
#define TBI_AXIS_ACTIVE 1

#define TBI_AXIS_X 0
#define TBI_AXIS_Y 1
#define TBI_AXIS_Z 2
#define TBI_AXIS_E 3
#define TBI_AXIS_R 4
#define TBI_AXIS_T 5
#define TBI_AXIS_V 6 //virtual axis. we use this for detecting the end of a move

class TimeBasedInterpolator : public Plugin{
  public:
    TimeBasedInterpolator();
    
    struct position{
      float64_t x_mm; // X
      float64_t y_mm; // Y
      float64_t z_mm; // Z
      float64_t e_mm; // Extruder
      float64_t r_mm; // Radial
      float64_t t_rad; // Theta
    };

    struct motion_block{
      uint8_t block_type; //specifies which type of block this is (e.g. delay, absolute, relative, set position, etc...). We don't currently use this.
      uint32_t block_id; //an ID # for the motion block
      float32_t block_time_s; //total time for the block, in seconds. We'll later convert this to frames, but keep it in seconds here for legibility.
      float32_t block_velocity_per_s;
      struct position block_position;
    };

    int16_t add_block(struct motion_block* block_to_add); //adds a block to the queue
    int16_t add_move(float32_t velocity_per_s, uint8_t mode, DecimalPosition x, DecimalPosition y, DecimalPosition z, DecimalPosition e, DecimalPosition r, DecimalPosition t);
    int16_t add_timed_move(float32_t time_s, uint8_t mode, DecimalPosition x, DecimalPosition y, DecimalPosition z, DecimalPosition e, DecimalPosition r, DecimalPosition t);
    volatile ControlParameter speed_overide = 1; //modifier for the interpolator speed.
    void begin();
    volatile uint16_t slots_remaining; //number of slots remaining in block queue
    bool is_idle(); //returns true if the interpolator is idle
    bool queue_is_full();
    void enroll(RPC *rpc, const String& instance_name);
    
    // BlockPorts
    BlockPort output_x;
    BlockPort output_y;
    BlockPort output_z;
    BlockPort output_e;
    BlockPort output_r;
    BlockPort output_t;

  private:
    // BlockPort State Variables
    DecimalPosition output_position_x;
    DecimalPosition output_position_y;
    DecimalPosition output_position_z;
    DecimalPosition output_position_e;
    DecimalPosition output_position_r;
    DecimalPosition output_position_t;

    struct motion_block block_queue[TBI_BLOCK_QUEUE_SIZE]; // stores all pending motion blocks
    volatile uint16_t next_write_index; //next write index in the block queue
    volatile uint16_t next_read_index; //next read index in the block queue 

    void advance_head(volatile uint16_t* target_head); //handles roll-overs etc
    void reset_block_queue();
    void pull_block(); //pulls a block from the queue and into the active buffer
    volatile uint8_t in_block = 0; //1 if actively reading a block
    volatile uint16_t active_block_id; //stores the current active block
    volatile uint8_t active_block_type; //we don't use this for now
    volatile uint8_t active_axes[TBI_NUM_AXES]; //indexed by axis #, 0 if axis inactive, 1 if active
    volatile float64_t active_axes_remaining_distance_mm[TBI_NUM_AXES];
    volatile float32_t active_axes_velocity_mm_per_frame[TBI_NUM_AXES];
    BlockPort* output_BlockPorts[TBI_NUM_AXES - 1] = {&output_x, &output_y, &output_z, &output_e, &output_r, &output_t};
    void run_frame_on_active_block(); //run a frame of the currently active block
    int16_t _add_move(float32_t move_time_s, uint8_t mode, float32_t velocity_per_s, DecimalPosition x, DecimalPosition y, DecimalPosition z, DecimalPosition e, DecimalPosition r, DecimalPosition t);
    
    enum{
      BLOCK_TYPE_INCREMENTAL,
      BLOCK_TYPE_ABSOLUTE,
      BLOCK_TYPE_GLOBAL
    };

  protected:
    void run();
};

#endif