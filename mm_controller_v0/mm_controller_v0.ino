#include "input_ports.hpp"
#include "output_ports.hpp"
#include "channels.hpp"
#include "core.hpp"
#include "interface_ebb.hpp"
#include "interpreters.hpp"
#include "kinematics.hpp"

InputPort input_b;
OutputPort output_a;
OutputPort output_b;

Channel channel_x;
Channel channel_y;


Eibotboard axidraw;
TimeBasedInterpreter interpreter;
KinematicsHBot axidraw_kinematics;

void setup() {
  // put your setup code here, to run once:
  output_a.begin(OUTPUT_A);
  output_b.begin(OUTPUT_B);

  channel_x.begin(&output_a, SIGNAL_E);
  channel_x.set_transmission_ratio(25.4, 2874); //axidraw: 1" == 2874 steps
  channel_x.invert_output();

  channel_y.begin(&output_b, SIGNAL_X);
  channel_y.set_transmission_ratio(25.4, 2874); //axidraw: 1" == 2874 steps
  channel_y.invert_output();

  input_b.begin(INPUT_B_LEGACY);
  input_b.map(SIGNAL_X, &channel_x.target_position);
  // input_b.map(SIGNAL_Y, &channel_z.target_position);
  // input_b.map(SIGNAL_Z, &channel_e.target_position);


  axidraw.begin(&interpreter);
  axidraw.set_steps_to_mm(2874, 25.4);

  interpreter.begin();
  interpreter.map(TBI_AXIS_X, &axidraw_kinematics.input_transmission_x);
  interpreter.map(TBI_AXIS_Y, &axidraw_kinematics.input_transmission_y);

  axidraw_kinematics.begin();
  axidraw_kinematics.map(HBOT_OUTPUT_A, &channel_x.target_position_transmission);
  axidraw_kinematics.map(HBOT_OUTPUT_B, &channel_y.target_position_transmission);

  stepdance_start();
}

uint64_t reporting_count = 0;
void loop() {
  axidraw.loop();
  // reporting_count++;
  // if(reporting_count%5000000 == 0){
  //   SerialUSB1.print("X POSITION: ");
  //   SerialUSB1.println(channel_x.current_position); 
  //   SerialUSB1.print("Y POSITION: ");
  //   SerialUSB1.println(channel_y.current_position);
  //   SerialUSB1.print("SLOTS AVAILABLE: ");
  //   SerialUSB1.println(interpreter.slots_remaining);    
  // }
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
  // Serial.println(stepdance_get_cpu_usage()*100);
  // Serial.print("Y POSITION: ");
  // Serial.println(channel_y.current_position);
}