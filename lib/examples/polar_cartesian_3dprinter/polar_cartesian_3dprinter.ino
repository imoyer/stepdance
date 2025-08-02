#define module_driver

#include "stepdance.hpp"

OutputPort output_a;
OutputPort output_b;
OutputPort output_c;
OutputPort output_d;

Channel channel_a;
Channel channel_b;
Channel channel_z;
Channel channel_e;

KinematicsPolarToCartesian polar_kinematics;
KinematicsCoreXY axidraw_kinematics;

AnalogInput analog_a1; //foot pedal

Encoder encoder_1; //hand lever

VelocityGenerator velocity_gen;

ScalingFilter1D z_gen;

PathLengthGenerator2D e_gen; //generates extruder signal

void setup() {
  output_a.begin(OUTPUT_A);
  output_b.begin(OUTPUT_B);
  output_c.begin(OUTPUT_C);
  output_d.begin(OUTPUT_D);

  enable_drivers();

  channel_a.begin(&output_a, SIGNAL_E);
  channel_a.set_ratio(25.4, 2874); //axidraw: 1" == 2874 steps
  channel_a.invert_output();

  channel_b.begin(&output_b, SIGNAL_E);
  channel_b.set_ratio(25.4, 2874); //axidraw: 1" == 2874 steps
  channel_b.invert_output();

  channel_z.begin(&output_c, SIGNAL_E);
  channel_z.set_ratio(8, 3200); // testing with an 8mm lead leadscrew

  channel_e.begin(&output_d, SIGNAL_E);
  channel_e.set_ratio(8, 3200); // testing with an 8mm lead leadscrew

  velocity_gen.begin();
  velocity_gen.output.map(&polar_kinematics.input_angle);

  encoder_1.begin(ENCODER_1);
  encoder_1.set_ratio(24, 2400); //25mm per revolution
  encoder_1.output.map(&polar_kinematics.input_radius);
  encoder_1.invert();

  polar_kinematics.output_x.map(&axidraw_kinematics.input_x);
  polar_kinematics.output_y.map(&axidraw_kinematics.input_y);
  polar_kinematics.begin();

  axidraw_kinematics.output_a.map(&channel_a.input_target_position);
  axidraw_kinematics.output_b.map(&channel_b.input_target_position);
  axidraw_kinematics.begin();

  z_gen.begin();
  z_gen.set_ratio(1, TWO_PI); //1mm per rev
  z_gen.input.map(&polar_kinematics.input_angle);
  z_gen.output.map(&channel_z.input_target_position);

  e_gen.begin();
  e_gen.set_ratio(1, TWO_PI); //1mm per rev
  e_gen.input_1.map(&channel_a.input_target_position);
  e_gen.input_2.map(&channel_b.input_target_position);
  e_gen.output.map(&channel_e.input_target_position);

  analog_a1.set_floor(0, 25);
  analog_a1.set_ceiling(6.28, 1020); //radians per second
  analog_a1.map(&velocity_gen.speed_units_per_sec);
  analog_a1.begin(IO_A1);

  dance_start();
}

LoopDelay overhead_delay;

void loop() {
  overhead_delay.periodic_call(&report_overhead, 500);
  dance_loop();
}

void report_overhead(){
  Serial.println(stepdance_get_cpu_usage(), 4);
  Serial.println(e_gen.input_1_position, 4);
  Serial.println(e_gen.input_2_position, 4);
}