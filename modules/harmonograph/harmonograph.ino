#include "stepdance.hpp"
#include <QuadEncoder.h>

// First protoype, using legacy hardware

InputPort input_a;

OutputPort output_a;
OutputPort output_b;
OutputPort output_c;

Channel channel_x;
Channel channel_y;
Channel channel_z;

Eibotboard axidraw;
TimeBasedInterpolator interpolator;
KinematicsHBot axidraw_kinematics;

CircleGenerator tiny_circles;

AnalogInput analog_a1;
AnalogInput analog_a2;
AnalogInput analog_a3;
AnalogInput analog_a4;

Encoder encoder_1;
Encoder encoder_2;

VelocityGenerator velocity_gen;

void setup() {
  // put your setup code here, to run once:
  pinMode(14, OUTPUT); //motor A enable
  pinMode(13, OUTPUT); //motor B enable
  digitalWrite(14, LOW); //enable motors
  digitalWrite(13, LOW);

  output_a.begin(OUTPUT_A);
  output_b.begin(OUTPUT_B);
  output_c.begin(OUTPUT_C);

  channel_x.begin(&output_a, SIGNAL_E);
  channel_x.set_transmission_ratio(25.4, 2874); //axidraw: 1" == 2874 steps
  channel_x.invert_output();

  channel_y.begin(&output_b, SIGNAL_E);
  channel_y.set_transmission_ratio(25.4, 2874); //axidraw: 1" == 2874 steps
  channel_y.invert_output();

  channel_z.begin(&output_c, SIGNAL_E); //servo motor, so we use a long pulse width
  channel_z.set_transmission_ratio(1, 1); //straight step pass-thru.

  input_a.begin(INPUT_A);
  input_a.map(SIGNAL_X, &channel_x.target_position);
  // input_b.map(SIGNAL_Y, &channel_z.target_position);
  // input_b.map(SIGNAL_Z, &channel_e.target_position);

  axidraw.begin(&interpolator);
  axidraw.set_steps_to_mm(2874, 25.4);

  interpolator.begin();
  interpolator.map(TBI_AXIS_X, &axidraw_kinematics.input_transmission_x);
  interpolator.map(TBI_AXIS_Y, &axidraw_kinematics.input_transmission_y);
  interpolator.map(TBI_AXIS_Z, &channel_z.target_position_transmission); //pass this straight thru to the channel

  tiny_circles.begin();
  tiny_circles.map(&axidraw_kinematics.input_transmission_x, &axidraw_kinematics.input_transmission_y);

  velocity_gen.begin();
  velocity_gen.map(&axidraw_kinematics.input_transmission_x);

  encoder_1.begin(ENCODER_1);
  encoder_1.set_ratio(2400, 150); //24mm per revolution
  encoder_1.map(&axidraw_kinematics.input_transmission_x);
  encoder_1.invert();

  encoder_2.begin(ENCODER_2);
  encoder_2.set_ratio(2400, 150); //24mm per revolution
  encoder_2.map(&axidraw_kinematics.input_transmission_y);
  encoder_2.invert();

  axidraw_kinematics.begin();
  axidraw_kinematics.map(HBOT_OUTPUT_A, &channel_x.target_position_transmission);
  axidraw_kinematics.map(HBOT_OUTPUT_B, &channel_y.target_position_transmission);

  // analog_a1.set_floor(0, 10);
  // analog_a1.set_ceiling(2.0, 1020);
  // analog_a1.map(&interpolator.speed_overide);
  // analog_a1.set_floor(-10, 10);
  // analog_a1.set_ceiling(10, 1020);
  // analog_a1.map(&velocity_gen.speed_units_per_sec);
  // analog_a1.set_floor(0, 10);
  // analog_a1.set_ceiling(10, 1020);
  // analog_a1.map(&velocity_gen.speed_units_per_sec);
  // analog_a1.begin(IO_A1);

  // analog_a2.set_floor(0, 10);
  // analog_a2.set_ceiling(5.0, 1020);
  // analog_a2.map(&tiny_circles.radius);
  // analog_a2.begin(IO_A2);

  // analog_a3.set_floor(0, 10);
  // analog_a3.set_ceiling(10.0, 1020);
  // analog_a3.map(&tiny_circles.rotational_speed_rev_per_sec);
  // analog_a3.begin(IO_A3);
  // analog_a4.begin(IO_A4);
  dance_start();
}

LoopDelay say_hi;

void loop() {
  axidraw.loop();
  say_hi.periodic_call(&say_hello, 500);
  dance_loop();
}

void say_hello(){
  Serial.print("X POS: ");
  Serial.println(channel_x.target_position);
  Serial.print("ENC1 POS: ");
  Serial.println(encoder_1.last_encoder_value);
  Serial.print("ENC2 POS: ");
  Serial.println(encoder_2.last_encoder_value);
}