#include "output_ports.hpp"

output_port output_0;

void setup() {
  // put your setup code here, to run once:
  output_0.begin(0, OUTPUT_FORMAT_STEPDANCE);
}

void loop() {
  // put your main code here, to run repeatedly:
  output_0.transmit(0b00000011111110011111100111001100, 0b00000000000000111111110000011110);
  delayMicroseconds(40);
}
