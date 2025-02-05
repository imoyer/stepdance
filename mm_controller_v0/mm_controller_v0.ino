#include "output_ports.hpp"
#include "channels.hpp"
#include "core.hpp"

output_port output_x;
output_port output_y;
output_port output_z;
output_port output_e;

channel channel_x;
channel channel_y;
channel channel_z;
channel channel_e;

void setup() {

  // put your setup code here, to run once:
  output_x.begin(OUTPUT_PORT_0);
  output_y.begin(OUTPUT_PORT_1);
  output_z.begin(OUTPUT_PORT_2);
  output_e.begin(OUTPUT_PORT_3);
  channel_x.begin(&output_x, SIGNAL_X);
  channel_y.begin(&output_y, SIGNAL_Y);
  channel_z.begin(&output_z, SIGNAL_Z);
  channel_e.begin(&output_e, SIGNAL_E);
  activate_channels();
  stepdance_start();
}

void loop() {
  channel_x.target_position += 100;
  // channel_y.target_position += -100;
  // channel_z.target_position += 100;
  // channel_e.target_position -= 100;
  delay(500);
}