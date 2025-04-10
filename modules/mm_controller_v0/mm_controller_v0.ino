#include "stepdance.hpp"

// First protoype, using legacy hardware

InputPort input_b;

OutputPort output_a;
OutputPort output_b;
OutputPort output_d;

Channel channel_x;
Channel channel_y;
Channel channel_z;

Eibotboard axidraw;
TimeBasedInterpolator interpolator;
KinematicsHBot axidraw_kinematics;

AnalogInput analog_a;
AnalogInput analog_b;
AnalogInput analog_d;
AnalogInput analog_e;

VelocityGenerator velocity_gen;

void setup() {
  // put your setup code here, to run once:
  output_a.begin(OUTPUT_A_LEGACY);
  output_b.begin(OUTPUT_B_LEGACY);
  output_d.begin(OUTPUT_C);

  channel_x.begin(&output_a, SIGNAL_Z);
  channel_x.set_transmission_ratio(25.4, 2874); //axidraw: 1" == 2874 steps
  channel_x.invert_output();

  channel_y.begin(&output_b, SIGNAL_Z);
  channel_y.set_transmission_ratio(25.4, 2874); //axidraw: 1" == 2874 steps
  channel_y.invert_output();

  channel_z.begin(&output_d, SIGNAL_E); //servo motor, so we use a long pulse width
  channel_z.set_transmission_ratio(1, 1); //straight step pass-thru.

  input_b.begin(INPUT_B_LEGACY);
  input_b.map(SIGNAL_X, &channel_x.target_position);
  // input_b.map(SIGNAL_Y, &channel_z.target_position);
  // input_b.map(SIGNAL_Z, &channel_e.target_position);


  axidraw.begin(&interpolator);
  axidraw.set_steps_to_mm(2874, 25.4);

  interpolator.begin();
  interpolator.map(TBI_AXIS_X, &axidraw_kinematics.input_transmission_x);
  interpolator.map(TBI_AXIS_Y, &axidraw_kinematics.input_transmission_y);
  interpolator.map(TBI_AXIS_Z, &channel_z.target_position_transmission); //pass this straight thru to the channel

  axidraw_kinematics.begin();
  axidraw_kinematics.map(HBOT_OUTPUT_A, &channel_x.target_position_transmission);
  axidraw_kinematics.map(HBOT_OUTPUT_B, &channel_y.target_position_transmission);

  velocity_gen.begin();
  velocity_gen.map(&channel_x.target_position_transmission);

  analog_a.set_floor(-20, 10);
  analog_a.set_ceiling(20, 1020);
  analog_a.map(&velocity_gen.speed_units_per_sec);
  analog_a.begin(IO_LEGACY_A);
  analog_b.begin(IO_LEGACY_B);
  analog_d.begin(IO_LEGACY_D);
  analog_e.begin(IO_LEGACY_E);

  dance_start();
}

LoopDelay say_hi;

void loop() {
  axidraw.loop();
  say_hi.periodic_call(&say_hello, 1000);
  dance_loop();
}

void say_hello(){
  Serial.print("A: ");
  Serial.println(analog_a.last_value_raw);
  Serial.print("B: ");
  Serial.println(analog_b.last_value_raw);
  Serial.print("D: ");
  Serial.println(analog_d.last_value_raw);
  Serial.print("E: ");
  Serial.println(analog_e.last_value_raw);
  Serial.println(ADC1_GC);
}