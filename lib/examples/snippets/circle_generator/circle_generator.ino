// Example of using CircleGenerator to create circular motion in X-Y plane (assuming an Cartesian X-Y mechanism)
#define module_driver
#include "stepdance.hpp"

CircleGenerator circle_gen;
Channel channel_x;
Channel channel_y;
OutputPort output_x;
OutputPort output_y;

void setup(){
  // Initialize OutputPort and enable drivers
  output_x.begin(OUTPUT_A);
  output_y.begin(OUTPUT_B);
  enable_drivers();

  // Configure circle generator to run without input
  circle_gen.setNoInput();
  circle_gen.radius = 10.0; // example radius in units
  circle_gen.rotational_speed_rev_per_sec = 0.5; // example frequency in rev/s
  circle_gen.begin();

  // Map generator outputs to two channels (X and Y)
  circle_gen.output_x.map(channel_x.input_target_position);
  circle_gen.output_y.map(channel_y.input_target_position);
  channel_x.begin(&output_x, SIGNAL_E);
  channel_y.begin(&output_y, SIGNAL_E);

  dance_start();
}

void loop(){
  dance_loop();
}
