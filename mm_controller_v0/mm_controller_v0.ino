#include "output_ports.hpp"
#include "channels.hpp"

output_port output;
channel channel_x;
channel channel_y;
channel channel_z;

void setup() {
  // put your setup code here, to run once:
  output.begin(OUTPUT_PORT_0);
  channel_x.begin(&output, SIGNAL_X);
  channel_y.begin(&output, SIGNAL_Y);
  channel_z.begin(&output, SIGNAL_Z);
}

void loop() {
  delayMicroseconds(40);
}
