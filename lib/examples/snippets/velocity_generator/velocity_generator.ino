//will drive a channel at a constant speed. WARNING, if no limits are implemented, the axis will drive until it crashes.
#define module_driver
#include "stepdance.hpp"

VelocityGenerator vel_gen;
Channel channel_x;
OutputPort output_a;

void setup() {
  // Initialize OutputPort and enable drivers
  output_a.begin(OUTPUT_A);
  enable_drivers();

  // Map VelocityGenerator output to channel target position
  vel_gen.output.map(channel_x.input_target_position);
  vel_gen.begin();
  vel_gen.speed_units_per_sec = 20.0; // example speed

  // Initialize channel on an output signal
  channel_x.begin(&output_a , SIGNAL_E);

  dance_start();
}

void loop(){
  dance_loop();
}
