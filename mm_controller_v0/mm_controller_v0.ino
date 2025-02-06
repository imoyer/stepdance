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

  input_b.begin(INPUT_B_LEGACY, &channel_x, &channel_y, nullptr, nullptr, &channel_z, &channel_e);

  activate_channels();
  stepdance_start();

  Serial.begin(115200);
}

void loop() {
  // channel_x.target_position += 100;
  // channel_y.target_position += -100;
  // channel_z.target_position += 100;
  channel_e.target_position -= 1000;
  delay(500);
  Serial.print("PULSE LENGTH:");
  Serial.println(input_b.last_pulse_width_us);
  Serial.print("PULSES:");
  Serial.println(input_b.fire_count);
  input_b.fire_count = 0;
}