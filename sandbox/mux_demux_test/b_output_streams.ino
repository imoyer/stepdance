// --- MIXING METAPHORS ---
//
// -- OUTPUT STREAMS --

struct output_stream{
  const int OUTPUT_STREAM_STEP_PIN;
  const int OUTPUT_STREAM_DIR_PIN; 

  volatile long channel_target_position = 0;
  volatile long channel_current_position = 0;
  volatile long channel_recorder_target_position = 0; //inbound position stream to the recorder
  volatile long channel_recorder_current_position = 0; //current position that's been recorded to memory
  volatile long channel_accumulator = 0;
  volatile long channel_accumulator_velocity = 0;
  volatile int channel_last_direction = 0;
  volatile int channel_step_flag = 0; //if 1, this axis will take a step 200ns after the step generator concludes. 
};

