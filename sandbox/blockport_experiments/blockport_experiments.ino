// -- BlockPort Scraps --

// float64_t BlockPort::get(uint8_t mode){
//   // Internally reads the incremental or absolute buffers
//   // This is an alternative method that a block can have more fine-grained control over accessing values in the BlockPort

//   if(mode == INCREMENTAL){
//     incremental_buffer_is_read = true; //flags that the buffer has been internally read.
//     return incremental_buffer;
//   }else{
//     absolute_buffer_is_written = false; //flags that we've read the absolute buffer
//     return absolute_buffer;
//   }
// }

// void BlockPort::set(float64_t value, uint8_t mode){
//   // Internally updates the value of buffers
//   if(mode == INCREMENTAL){
//     incremental_buffer = value;
//   }else{
//     absolute_buffer = value;
//   }
// }




// -- Block Port Class --
// Block ports are used to provide inputs and outputs between component blocks.
class BlockInput;
class BlockOutput;

class BlockInput{
  public:
    BlockInput();
    void begin(); // initialized by the block that uses this input
    void begin(uint8_t mode); //initialize input with a mode

    // Mapping Functions (User Space)
    void set_pull(DecimalPosition *target); //default mode is ABSOLUTE for direct register access
    void set_pull(DecimalPosition *target, uint8_t mode);    
    void set_pull(ControlParameter *target); //default mode is ABSOLUTE for direct register access
    void set_pull(ControlParameter *target, uint8_t mode);  
    void set_pull(BlockOutput *target);
    void set_pull(BlockInput *target);

    // Setup Functions (User Space)
    void set_ratio(float input_units, float output_units); //sets the ratio between input and output, for automatic conversion. Default is 1.
  
    // Target Functions -- these get called by another BlockPort, when this BlockPort is the target
    void set(float64_t absolute_value);
    void increment(float64_t incremental_value);
    float64_t read(); //reads the active or last value, depending on when the read was executed. Note that this read will return a value according to THIS block's MODE.
                      // additionally, read does NOT apply the transfer ratio. It is intended to return its value in internal units. 

    // Block Functions -- called by the block that owns this BlockPort
    void pull(); //pulls in a value from the target
    float64_t get(); //returns the active_value and calls reset();
    void reset(); // transfers active_value to last_value, sets active_is_valid to false, and clears active_value

    // State Variables
    uint8_t mode = INCREMENTAL; //the primary mode of this BlockPort. It will push and pull using this mode.
    uint8_t direct_pull_mode = INCREMENTAL; //mode used for pulling directly from registers.

  private:
    // target pointers
    DecimalPosition* target_DecimalPosition = nullptr;
    ControlParameter* target_ControlParameter = nullptr;
    BlockOutput* target_BlockOutput = nullptr;
    BlockInput* target_BlockInput = nullptr;

    // value state
    volatile float64_t active_value = 0; //stores the active value of the input (post-transform ratio)
    volatile float64_t last_value = 0; //stores the last value of the input (post-transform ratio)

    // flags
    volatile bool active_is_valid = true; // if true, read() returns active_value. Otherwise last_value;

    // transfer ratio
    float64_t transfer_ratio = 1;
};

class BlockOutput{
  public:
    BlockOutput();
    void begin(); // initialized by the block that uses this input

    // Mapping Functions (User Space)
    void set_push(DecimalPosition *target);  //default mode is ABSOLUTE for direct register access
    void set_push(DecimalPosition *target, uint8_t mode);

    void set_push(ControlParameter *target);  //default mode is ABSOLUTE for direct register access
    void set_push(ControlParameter *target, uint8_t mode);

    void set_push(BlockInput *target); //mode is determined by the target

    // Setup Functions (User Space)
    void set_ratio(float input_units, float output_units); //sets the ratio between input and output, for automatic conversion. Default is 1.

    // Target Functions -- these get called by another BlockPort, when this BlockPort is the target
    float64_t read(uint8_t mode); //returns the last set incremental or absolute values

    // Block Functions -- called by the block that owns this BlockPort
    void push(); //pushes the absolute_value or incremental_value to the target
    void update_absolute(float64_t absolute_value);
    void update_incremental(float64_t incremental_value);
    void update_both(float64_t absolute_value); //updates absolute directly, and incremental as the difference with the last absolute value
    
  private:
    // target pointers
    DecimalPosition* target_DecimalPosition = nullptr;
    ControlParameter* target_ControlParameter = nullptr;
    BlockOutput* target_BlockOutput = nullptr;
    BlockInput* target_BlockInput = nullptr;

    volatile float64_t active_absolute_value; //(PRE-TRANSFORM RATIO)
    volatile float64_t active_incremental_value; //(PRE-TRANSFORM RATIO)

    float64_t transfer_ratio = 1;

    uint8_t push_mode = INCREMENTAL;

};

// -- BLOCK INPUT --
BlockInput::BlockInput(){};

void BlockInput::begin(){
  begin(INCREMENTAL);
}

void BlockInput::begin(uint8_t mode){
  this->mode = mode;
}

void BlockInput::set_pull(DecimalPosition *target){
  set_pull(target, ABSOLUTE);
}

void BlockInput::set_pull(DecimalPosition *target, uint8_t mode){
  target_DecimalPosition = target;
  direct_pull_mode = mode;
}

void BlockInput::set_pull(ControlParameter *target){
  set_pull(target, ABSOLUTE);
}

void BlockInput::set_pull(ControlParameter *target, uint8_t mode){
  target_ControlParameter = target;
  direct_pull_mode = mode;
}

void BlockInput::set_pull(BlockOutput *target){
  target_BlockOutput = target;
}

void BlockInput::set_pull(BlockInput *target){
  target_BlockInput = target;
}

void BlockInput::set_ratio(float input_units, float output_units){
  transfer_ratio = static_cast<float64_t>(output_units / input_units);  
}

void BlockInput::set(float64_t absolute_value){
  active_value = (absolute_value * transfer_ratio);
  active_is_valid = true;
}

void BlockInput::increment(float64_t incremental_value){
  active_value += (incremental_value * transfer_ratio);
  active_is_valid = true;
}

float64_t BlockInput::read(){
  if(active_is_valid){
    return active_value;
  }else{
    return last_value;
  }
}

float64_t BlockInput::get(){
  last_value = active_value;
  reset();
  return last_value;
}

void BlockInput::reset(){
  active_value = 0;
  active_is_valid = false;
}

void BlockInput::pull(){
  if(target_BlockInput != nullptr){ //pulling an input block
    if(mode == INCREMENTAL){
      increment(target_BlockInput->read());
    }else{
      set(target_BlockInput->read());
    }
  }
  if(target_BlockOutput != nullptr){
    if(mode == INCREMENTAL){
      increment(target_BlockOutput->read(INCREMENTAL));
    }else{
      set(target_BlockOutput->read(ABSOLUTE));
    }    
  }
  if(target_DecimalPosition != nullptr){
    if(direct_pull_mode == INCREMENTAL){
      active_value += (*target_DecimalPosition);
    }else{
      active_value = (*target_DecimalPosition);
    }
  }

  if(target_ControlParameter != nullptr){
    if(direct_pull_mode == INCREMENTAL){
      active_value += (*target_ControlParameter);
    }else{
      active_value = (*target_ControlParameter);
    }
  }
}

// -- BLOCK OUTPUT --
BlockOutput::BlockOutput(){};

void BlockOutput::begin(){}; //begin doesn't do anything for now, but should be called by the block for future compatibility.

void BlockOutput::set_push(DecimalPosition *target){
  set_push(target, ABSOLUTE);
}

void BlockOutput::set_push(DecimalPosition *target, uint8_t mode){
  target_DecimalPosition = target;
  push_mode = mode;
}

void BlockOutput::set_push(ControlParameter *target){
  set_push(target, ABSOLUTE);
}

void BlockOutput::set_push(ControlParameter *target, uint8_t mode){
  target_ControlParameter = target;
  push_mode = mode;
}

void BlockOutput::set_push(BlockInput *target){
  target_BlockInput = target;
  push_mode = target->mode;
}

void BlockOutput::set_ratio(float input_units, float output_units){
  transfer_ratio = static_cast<float64_t>(output_units / input_units); 
}

float64_t BlockOutput::read(uint8_t mode){
  if(mode == INCREMENTAL){
    return active_incremental_value * transfer_ratio;
  }else{
    return active_absolute_value * transfer_ratio;
  }
};

void BlockOutput::push(){
  if(target_BlockInput != nullptr){
    if(push_mode == INCREMENTAL){
      target_BlockInput->increment(active_incremental_value * transfer_ratio);
    }else{
      target_BlockInput->set(active_incremental_value * transfer_ratio);
    }
  }

  if(target_DecimalPosition != nullptr){
    if(push_mode == INCREMENTAL){
      (*target_DecimalPosition) += active_incremental_value;
    }else{
      (*target_DecimalPosition) = active_absolute_value;
    }
  }

  if(target_ControlParameter != nullptr){
    if(push_mode == INCREMENTAL){
      (*target_ControlParameter) += active_incremental_value;
    }else{
      (*target_ControlParameter) = active_absolute_value;
    }
  }
}

void BlockOutput::update_absolute(float64_t absolute_value){
  active_absolute_value = absolute_value;
}

void BlockOutput::update_incremental(float64_t incremental_value){
  active_incremental_value = incremental_value;
}

void BlockOutput::update_both(float64_t absolute_value){
  active_incremental_value = (absolute_value - active_absolute_value); //store delta as incremental value
  active_absolute_value = absolute_value; //overwrite absolute value
}
