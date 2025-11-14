#define module_driver
#include "stepdance.hpp"

CircleGenerator circle_gen;
PathLengthGenerator2D path_gen;
Channel channel_x;
Channel channel_y;
Channel channel_z;
OutputPort output_a;

void setup(){
  // Initialize OutputPort and enable drivers
  output_a.begin(OUTPUT_A);
  enable_drivers();

  // Configure CircleGenerator with radius 10mm, 1 revolution per second
  circle_gen.radius = 10.0;
  circle_gen.rotational_speed_rev_per_sec = 1.0;
  circle_gen.setNoInput(); // Use internal frame count
  circle_gen.begin();

  // Map circle outputs to X and Y channels
  circle_gen.output_x.map(channel_x.input_target_position);
  circle_gen.output_y.map(channel_y.input_target_position);
  channel_x.begin(&output_a, SIGNAL_X);
  channel_y.begin(&output_a, SIGNAL_Y);

  // Configure PathLengthGenerator2D to move Z 1mm per complete circle
  // The circle has radius 10mm, so we want 1mm output per 2π*10mm = 62.83mm of XY path
  path_gen.set_ratio_for_circle(10.0, 1.0); // 1mm per revolution of a 10mm radius circle
  path_gen.begin();

  // Map circle outputs to path length inputs
  circle_gen.output_x.map(path_gen.input_1);
  circle_gen.output_y.map(path_gen.input_2);

  // Map path length output to Z channel
  path_gen.output.map(channel_z.input_target_position);
  channel_z.begin(&output_a, SIGNAL_Z);

  dance_start();
}

void loop(){
  dance_loop();
}
