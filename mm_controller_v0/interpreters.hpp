#include "core.hpp"

#ifndef interpreters_h //prevent importing twice
#define interpreters_h

#define TIME_BASED_INTERPRETER_BLOCK_QUEUE_SIZE   100 //we'll start here. Each block currently requires 33 bytes of RAM

class TimeBasedInterpreter : public Plugin{
  public:
    TimeBasedInterpreter();
    
    struct position{
      float x_mm; // X
      float y_mm; // Y
      float z_mm; // Z
      float e_mm; // Extruder
      float r_mm; // Radial
      float t_rad; // Theta
    };

    struct motion_block{
      uint8_t block_type; //specifies which type of block this is (e.g. delay, absolute, relative, set position, etc...)
      uint32_t block_id; //an ID # for the motion block
      float block_time_s; //total time for the block, in seconds. We'll later convert this to frames, but keep it in seconds here for legibility.
      struct position block_position;
    };

    uint16_t add_block(struct motion_block* block_to_add); //adds a block to the queue
    volatile struct motion_block active_block;
    volatile uint8_t in_block = 0; //1 if actively reading a block
    volatile uint16_t next_read_index; //next read index in the block queue 
    void begin();
    
  private:
    struct motion_block block_queue[TIME_BASED_INTERPRETER_BLOCK_QUEUE_SIZE]; // stores all pending motion blocks
    volatile uint16_t next_write_index; //next write index in the block queue
    volatile uint16_t slots_remaining; //number of slots remaining in block queue
    void advance_head(volatile uint16_t* target_head); //handles roll-overs etc
    void reset_block_queue();
  
  protected:
    void run();
};

#endif