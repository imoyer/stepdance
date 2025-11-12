#define module_driver
#include "stepdance.hpp"

InputPort inputA;
Channel channelX;
OutputPort outputA;
void setup() {
  // -- Configure and start the output port --
  outputA.begin(OUTPUT_A); // Initialize OutputPort A
    // Enable the output drivers
  enable_drivers();
    // -- Configure and start the channel --
  channelX.begin(&outputA, SIGNAL_X); // Connect Channel X to OutputPort A
  // -- Configure and start the input port --
  inputA.begin(INPUT_A); // Initialize InputPort A
  inputA.output_x.map(channelX.input_target_position); // Map SIGNAL_X to Channel X's input target position
}

void loop(){
dance_loop();
}
