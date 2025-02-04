#include "output_ports.hpp"

output_port output_0;

void setup() {
  // put your setup code here, to run once:
  output_0.begin(0);
}

void loop() {
  delayMicroseconds(40);
}
