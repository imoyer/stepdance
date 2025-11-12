#define module_driver
#include "stepdance.hpp"

Channel channel_x;
OutputPort output_x;    

void setup() {
  // -- Configure and start the output port --
  output_x.begin(OUTPUT_A); // "OUTPUT_A" specifies the physical port on the PCB for the output.
    // -- Configure and start the channels --
  channel_x.begin(&output_x, SIGNAL_X); // Connects the channel to the "X" signal on "output_x".
                                        // We choose the "X" signal because it results in a step pulse of 2us,
                                        // which is more than long enough for the driver IC
  channel_x.set_ratio(40, 3200); // Sets the input/output transmission ratio for the channel.
} 

void loop(){
dance_loop();
}
