#include "output_ports.hpp"

output_port output_0;
output_port output_1;
output_port output_2;
output_port output_3;

void setup() {
  // put your setup code here, to run once:
  output_0.begin(0);
  output_1.begin(1);
  output_2.begin(2);
  output_3.begin(3);
}

void loop() {
  output_0.add_signal(SIGNAL_X, DIRECTION_FORWARD);
  output_0.add_signal(SIGNAL_Y, DIRECTION_REVERSE);
  output_0.add_signal(SIGNAL_Z, DIRECTION_FORWARD);
  output_0.add_signal(SIGNAL_E, DIRECTION_REVERSE);
  output_0.transmit_frame();
  delayMicroseconds(40);
}
