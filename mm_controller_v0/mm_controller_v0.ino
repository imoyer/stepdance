#include "input_ports.hpp"
#include "output_ports.hpp"
#include "channels.hpp"
#include "core.hpp"

input_port input_b;
output_port output_a;

channel channel_x;
channel channel_y;
channel channel_z;
channel channel_e;

void setup() {

  // put your setup code here, to run once:
  output_a.begin(OUTPUT_A);

  channel_x.begin(&output_a, SIGNAL_X);
  channel_y.begin(&output_a, SIGNAL_Y);
  channel_z.begin(&output_a, SIGNAL_Z);
  channel_e.begin(&output_a, SIGNAL_E);

  input_b.begin(INPUT_B_LEGACY, &channel_y.target_position, nullptr, nullptr, nullptr, nullptr, nullptr);

  activate_channels();
  stepdance_start();

  Serial.begin(115200);
}

void loop() {
  while(SerialUSB1.available() > 0){
    uint8_t byte_in = SerialUSB1.read();
    Serial.write(byte_in);
  }
  while(Serial.available() > 0){
    uint8_t byte_in2 = Serial.read();
    SerialUSB1.write(byte_in2);
  }
  // channel_x.target_position -= 100;
  // // channel_y.target_position += -100;
  // // channel_z.target_position += 100;
  // // channel_e.target_position -= 1000;
  // delay(500);
  // // Serial.print("INTERRUPT CYCLES");
  // // Serial.println(input_b.input_interrupt_cycles);
  // Serial.print("Y POSITION: ");
  // Serial.println(channel_y.current_position);
}