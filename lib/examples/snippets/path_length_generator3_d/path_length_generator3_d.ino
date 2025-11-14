#define module_driver
#include "stepdance.hpp"

PathLengthGenerator3D path3_gen;
InputPort input_b;
Channel channel_p3;
OutputPort output_p3;

void setup(){
  // Initialize OutputPort and enable drivers
  output_p3.begin(OUTPUT_A);
  enable_drivers();

  // Initialize InputPort and map X/Y/Z to path length inputs
  input_b.begin(INPUT_A);
  input_b.output_x.map(path3_gen.input_1);
  input_b.output_y.map(path3_gen.input_2);
  input_b.output_z.map(path3_gen.input_3);

  // Configure and begin generator
  path3_gen.set_ratio(1.0);
  path3_gen.begin();

  // Map generator output to a channel
  path3_gen.output.map(channel_p3.input_target_position);
  channel_p3.begin(&output_p3, SIGNAL_E);

  dance_start();
}

void loop(){
  dance_loop();
}
