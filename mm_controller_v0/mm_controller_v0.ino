#include "input_ports.hpp"
#include "output_ports.hpp"
#include "channels.hpp"
#include "core.hpp"
#include "interface_ebb.hpp"
#include "interpreters.hpp"

InputPort input_b;
OutputPort output_a;

Channel channel_x;
Channel channel_y;
Channel channel_z;
Channel channel_e;

Eibotboard axidraw;
TimeBasedInterpreter interpreter;

void setup() {

  // put your setup code here, to run once:
  output_a.begin(OUTPUT_A);

  channel_x.begin(&output_a, SIGNAL_X);
  channel_y.begin(&output_a, SIGNAL_Y);
  channel_z.begin(&output_a, SIGNAL_Z);
  channel_e.begin(&output_a, SIGNAL_E);

  input_b.begin(INPUT_B_LEGACY, &channel_y.target_position, nullptr, nullptr, nullptr, nullptr, nullptr);

  axidraw.begin(&interpreter);
  interpreter.begin();
  activate_channels();
  stepdance_start();
}

uint16_t test_block_id = 0;
void loop() {
  axidraw.loop();
  // struct TimeBasedInterpreter::motion_block this_block = {.block_id = test_block_id, .block_position = {.x_mm = 100, .y_mm = 200, .z_mm = 300}};
  // interpreter.add_block(&this_block);
  // delay(500);
  // Serial.print("READ HEAD: ");
  // Serial.println(interpreter.next_read_index);
  // Serial.print("ACTIVE ID: ");
  // Serial.println(interpreter.active_block.block_id);
  // Serial.print("X POS: ");
  // Serial.println(interpreter.active_block.block_position.x_mm);
  // Serial.print("CPU USAGE: ");
  // Serial.println(stepdance_get_cpu_usage());
  // test_block_id ++;
  // interpreter.in_block = 0;
  // channel_x.target_position -= 100;
  // channel_y.target_position += -100;
  // channel_z.target_position += 100;
  // channel_e.target_position -= 1000;
  // delay(500);
  // Serial.print("CPU USAGE ");
  // Serial.println(stepdance_get_cpu_usage());
  // Serial.print("Y POSITION: ");
  // Serial.println(channel_y.current_position);
}