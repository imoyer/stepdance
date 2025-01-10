// --- WHEELPRINT ---
// THE 3D PRINTING POTTERY WHEEL
//
// CHANNELS MODULE
//
// Generates output steps to each motor channel, based on target positions and established limits.

// --- CHANNELS ---
struct channel{
  const int CHANNEL_ID; //a numerical ID for the channel
  const int CHANNEL_STEP_PIN;
  const int CHANNEL_DIR_PIN; 
  const int CHANNEL_DIR_INVERT; //used to invert the direction signal of the channel.

  volatile long channel_target_position = 0;
  volatile long channel_current_position = 0;
  volatile long channel_recorder_target_position = 0; //inbound position stream to the recorder
  volatile long channel_recorder_current_position = 0; //current position that's been recorded to memory
  volatile long channel_accumulator = 0;
  volatile long channel_accumulator_velocity = 0;
  volatile int channel_last_direction = 0;
  volatile int channel_step_flag = 0; //if 1, this axis will take a step 200ns after the step generator concludes. 
};

struct channel channel_a = {.CHANNEL_ID = 1, .CHANNEL_STEP_PIN = 23, .CHANNEL_DIR_PIN = 22, .CHANNEL_DIR_INVERT = 0};
struct channel channel_b = {.CHANNEL_ID = 2, .CHANNEL_STEP_PIN = 21, .CHANNEL_DIR_PIN = 20, .CHANNEL_DIR_INVERT = 0};
struct channel channel_c = {.CHANNEL_ID = 3, .CHANNEL_STEP_PIN = 17, .CHANNEL_DIR_PIN = 10, .CHANNEL_DIR_INVERT = 0};
struct channel channel_d = {.CHANNEL_ID = 4, .CHANNEL_STEP_PIN = 11, .CHANNEL_DIR_PIN = 12, .CHANNEL_DIR_INVERT = 0};

const int CHANNEL_COUNT = 4;
struct channel *all_channels[CHANNEL_COUNT] = {&channel_a, &channel_b, &channel_c, &channel_d};

// --- STEP GENERATION TIMER ---
IntervalTimer step_gen_timer;
IntervalTimer step_pulse_timer;

// --- STEP GENERATION PARAMETERS ---
const long CHANNEL_ACCUMULATOR_THRESHOLD = 1000000;
const int STEP_GEN_TICK_PERIOD_US = 5; //microseconds. This yields a max output step rate of 200k steps/sec.
const long STEP_GEN_MAX_RATE = 1000000 / STEP_GEN_TICK_PERIOD_US;
const int STEP_GEN_DIR_HOLD_PERIOD_US = 1; //microseconds. 200ns is the direction setup time per Gecko (clearpath accepts 25nS)
const int STEP_GEN_STEP_HOLD_PERIOD_US = 1; //microseconds. 1us is the minimum hold time for a step pulse per gecko (clearpath accepts 750ns)

// --- STEP GENERATION STATE ---
volatile int channels_steps_pending_flag = 0; //if 1, steps are pending in at least one channel.

//--- CHANNEL INITIALIZATION ---
void channels_initialize(){
  // Initializes all channels
  int channel_index;
  for(channel_index = 0; channel_index < CHANNEL_COUNT; channel_index ++){
    channel_initialize(all_channels[channel_index]); 
  }

  step_gen_timer.priority(128);
  step_pulse_timer.priority(64);

  // Begin Step Generator
  step_gen_timer.begin(channels_step_generator, STEP_GEN_TICK_PERIOD_US);
}

void channel_initialize(struct channel *channel_target){
  channel_initialize_io(channel_target);
}

void channel_initialize_io(struct channel *channel_target){
  pinMode(channel_target->CHANNEL_STEP_PIN, OUTPUT);
  pinMode(channel_target->CHANNEL_DIR_PIN, OUTPUT);
}

// --- CHANNEL FUNCTIONS ---
void channel_velocity_set_max(float velocity_max_steps_per_sec, struct channel *channel_target){
  // sets the maximum velocity permissible on a channel.
  const float tick_time_seconds = (float) STEP_GEN_TICK_PERIOD_US / 1000000.0; //seconds per tick
  float steps_per_tick = velocity_max_steps_per_sec * tick_time_seconds; //steps per tick
  if(steps_per_tick>1.0){
    //cap the velocity at 1 step per tick
    steps_per_tick = 1.0;
  }
  channel_target->channel_accumulator_velocity = (long)((float)CHANNEL_ACCUMULATOR_THRESHOLD * steps_per_tick);
}

// --- STEP GENERATOR ---
void channels_step_generator(){
  // This is where the step generation magic happens.
  int channel_index;
  for(channel_index = 0; channel_index < CHANNEL_COUNT; channel_index ++){
    // reference the target channel
    struct channel *channel_target = all_channels[channel_index];
    
    // calculate distance to target position, in steps.
    long channel_delta_position = channel_target->channel_target_position - channel_target->channel_current_position;
    
    //determine direction of motion
    int channel_delta_direction;
    if(channel_delta_position > 0){
      channel_delta_direction = 1;
    }else{
      channel_delta_direction = 0;
    }

    //IF A STEP IS AVALIABLE...
    if(channel_delta_position != 0){
      //calculate the active accumulator threshold
      long channel_accumulator_active_threshold;
      if(channel_delta_direction ^ channel_target->channel_last_direction){
        //if direction has changed, we use an accumulator that is double the normal value.
        channel_accumulator_active_threshold = CHANNEL_ACCUMULATOR_THRESHOLD * 2;
      }else{
        channel_accumulator_active_threshold = CHANNEL_ACCUMULATOR_THRESHOLD;
      }

      if(channel_target->channel_accumulator >= channel_accumulator_active_threshold){
        // The accumulator is already maxed out. This happens when the accumulator has time to fill before a step is taken.
        // In this case, we simply take a step and empty the accumulator.
        channel_step(channel_target, channel_delta_direction); //take a step
        channel_target->channel_accumulator = 0;

      }else{
        channel_target->channel_accumulator += channel_target->channel_accumulator_velocity; //increment the accumulator by the velocity

        if(channel_target->channel_accumulator >= channel_accumulator_active_threshold){
          //we've _now_ exceeded the accumulator threshold, so take a step
          channel_step(channel_target, channel_delta_direction);
          channel_target->channel_accumulator -= channel_accumulator_active_threshold;
        }
      }
    } else { //we are not taking a step, but still want to increment the accumulator if it isn't already capped out.
      if(channel_target->channel_accumulator < 2*CHANNEL_ACCUMULATOR_THRESHOLD){ //2x threshold is the top of what is meaningful for us.
        channel_target->channel_accumulator += channel_target->channel_accumulator_velocity;
      }
    }
  }
  if(channels_steps_pending_flag){ //need to take a step
    step_pulse_timer.begin(channels_set_steps, STEP_GEN_DIR_HOLD_PERIOD_US); //start the pulse timer to hold the direction lines for a minimum setup period.
    channels_steps_pending_flag = 0; //clear the flag
  }
}

void channel_step(struct channel *channel_target, int step_direction){
  if(step_direction){ //forwards
    channel_target->channel_current_position ++;
    channel_target->channel_last_direction = 1;
    channel_target->channel_step_flag = 1; //step will be taken 200nS after this function concludes.
    digitalWrite(channel_target->CHANNEL_DIR_PIN, (1^channel_target->CHANNEL_DIR_INVERT)); //sets up the direction pin now
    channels_steps_pending_flag = 1; //flag that steps are waiting to be taken.
  }else{ //reverse
    //check that limits are off, or that within negative limits
    channel_target->channel_current_position --;
    channel_target->channel_last_direction = 0;
    channel_target->channel_step_flag = 1; //step will be taken 200nS after this function concludes.
    digitalWrite(channel_target->CHANNEL_DIR_PIN, (0^channel_target->CHANNEL_DIR_INVERT)); //sets up the direction pin now
    channels_steps_pending_flag = 1; //flag that steps are waiting to be taken.
  }
}

void channels_set_steps(){
  step_pulse_timer.end();
  int channel_index;
  for(channel_index = 0; channel_index < CHANNEL_COUNT; channel_index ++){
    // reference the target channel
    struct channel *channel_target = all_channels[channel_index];
    if(channel_target->channel_step_flag){
      digitalWrite(channel_target->CHANNEL_STEP_PIN, HIGH);
      channel_target->channel_step_flag = 0; //reset the step flag
    }
  }
  step_pulse_timer.begin(channels_clear_steps, STEP_GEN_STEP_HOLD_PERIOD_US);
}

void channels_clear_steps(){
  step_pulse_timer.end();
  int channel_index;
  for(channel_index = 0; channel_index < CHANNEL_COUNT; channel_index ++){
    // reference the target channel
    struct channel *channel_target = all_channels[channel_index];
    digitalWrite(channel_target->CHANNEL_STEP_PIN, LOW);
  }
}
