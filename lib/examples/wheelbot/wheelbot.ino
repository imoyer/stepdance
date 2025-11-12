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

AnalogInput analog_a1; //foot pedal
AnalogInput analog_a2; //rotary pot
AnalogInput analog_a3; //rotary pot

Encoder encoder_1; //hand lever
Encoder encoder_2; //z babystep

VelocityGenerator velocity_gen;

ScalingFilter1D z_gen; //generates z signal
ScalingFilter1D x_stretch;

PathLengthGenerator2D e_gen; //generates extruder signal

float64_t layerHeight = 2.0;
float64_t nozzleDiameter = 4.0;

void setup() {

  output_a.begin(OUTPUT_A);
  output_b.begin(OUTPUT_B);
  output_c.begin(OUTPUT_C);
  output_d.begin(OUTPUT_D);

  enable_drivers();

  channel_a.begin(&output_a, SIGNAL_E);
  channel_a.set_ratio(1, 40);
  channel_a.enable_filtering(200);

  channel_b.begin(&output_b, SIGNAL_E);
  channel_b.set_ratio(1, 40);
  channel_b.enable_filtering(200);

  channel_z.begin(&output_c, SIGNAL_E);
  channel_z.set_ratio(1, 1201); // testing with an 8mm lead leadscrew
  channel_z.invert_output();

  channel_e.begin(&output_d, SIGNAL_E);
  channel_e.set_ratio(1, 70); // testing with an 8mm lead leadscrew
  channel_e.invert_output();
  channel_e.enable_filtering(1000);

  velocity_gen.begin();
  velocity_gen.output.map(&polar_kinematics.input_angle);

  encoder_1.begin(ENCODER_1);
  encoder_1.set_ratio(1, 2400); //1mm per revolution
  encoder_1.output.map(&polar_kinematics.input_radius);

  encoder_2.begin(ENCODER_2);
  encoder_2.set_ratio(1, 2400); //1mm per revolution
  encoder_2.output.map(&channel_z.input_target_position);

  polar_kinematics.output_x.map(&channel_a.input_target_position);
  polar_kinematics.output_x.map(&x_stretch.input);

  polar_kinematics.output_y.map(&channel_b.input_target_position);
  polar_kinematics.begin();

  z_gen.begin();
  z_gen.set_ratio(layerHeight, TWO_PI); //raise a distance of a single layer for 1 revolution
  z_gen.input.map(&polar_kinematics.input_angle);
  z_gen.output.map(&channel_z.input_target_position);

  x_stretch.begin(ABSOLUTE);
  x_stretch.output.map(&channel_a.input_target_position);

  e_gen.begin();
  e_gen.input_1.map(&channel_a.input_target_position);
  e_gen.input_2.map(&channel_b.input_target_position);
  e_gen.output.map(&channel_e.input_target_position);

  //pedal
  analog_a1.set_floor(0, 25);
  analog_a1.set_ceiling(2.75, 1020);
  analog_a1.map(&velocity_gen.speed_units_per_sec);
  analog_a1.begin(IO_A1);

  //extrusion multiplier knob
  analog_a2.set_floor(0, 25);
  analog_a2.set_ceiling(10, 1020);
  analog_a2.begin(IO_A2);

  //x-stretch knob
  analog_a3.set_floor(0, 25);
  analog_a3.set_ceiling(100, 1020);
  analog_a3.map(&velocity_gen.speed_units_per_sec);
  analog_a3.begin(IO_A3);

  dance_start();
}


void loop() {

  float extrusionMultiplier = analog_a2.read();
  float extrusionRate = (4*layerHeight * extrusionMultiplier * nozzleDiameter) / (PI*nozzleDiameter*nozzleDiameter);
  e_gen.set_ratio(extrusionRate);

  x_stretch.set_ratio(analog_a3.read());

  dance_loop();
}

