/*
Digital Pantograph

Example project for the Stepdance control system.

A part of the Mixing Metaphors Project

// (c) 2025 Ilan Moyer, Jennifer Jacobs, Devon Frost
*/

#define module_basic   // tells compiler we're using the Stepdance Basic Module PCB

#include "stepdance.hpp"  // Import the stepdance library

// -- Define Output Ports --
// We'll run everything out over a single output port

OutputPort output_a;  // Basic Module output port

// -- Define Motion Channels --
Channel channel_x;
Channel channel_y;
Channel channel_z;

// -- Define Kinematics --
KinematicsFiveBarForward pantograph_kinematics;

// -- Define Encoders --
Encoder encoder_r; //ENC_1 -- right encoder
Encoder encoder_l; //ENC_2 -- left encoder
Encoder encoder_t; //ENC_A -- tilt encoder

void setup() {
  // -- Configure and start the output ports --
  output_a.begin(OUTPUT_A);

  // -- Configure and start the channels --
  channel_x.begin(&output_a, SIGNAL_X);
  channel_x.set_ratio(0.01, 1); //basic resolution of 1 output step = 0.01mm
  
  channel_y.begin(&output_a, SIGNAL_Y);
  channel_y.set_ratio(0.01, 1);

  channel_z.begin(&output_a, SIGNAL_Z);
  channel_y.set_ratio(0.01, 1);

  // -- Configure and start the encoders --
  const DecimalPosition encoder_r_home_rad = -(60.0/360.0)*TWO_PI;
  encoder_r.begin(ENCODER_1); // RIGHT ENCODER
  encoder_r.set_ratio(TWO_PI, 40000); // 1 REV = 40000 COUNTS. (10K Cycles/Rev)
  encoder_r.invert(); //invert the encoder direction
  encoder_r.set(encoder_r_home_rad); //home position
  encoder_r.set_latch(encoder_r_home_rad, MIN); // as we rotate against the hardstop, the home position will update.
  encoder_r.output.map(&pantograph_kinematics.input_r, ABSOLUTE);

  const DecimalPosition encoder_l_home_rad = (240.0/360.0)*TWO_PI;
  encoder_l.begin(ENCODER_2); // LEFT ENCODER
  encoder_l.set_ratio(TWO_PI, 40000);
  encoder_l.invert();
  encoder_l.set(encoder_l_home_rad);
  encoder_l.set_latch(encoder_l_home_rad, MAX);
  encoder_l.output.map(&pantograph_kinematics.input_l, ABSOLUTE); // map the right encoder to the y axis input of the kinematics

  // -- Configure and start the kinematics modules --
  pantograph_kinematics.begin(60, 145.75, 134, 169, 178.3, 28.82, 2.9019);
  pantograph_kinematics.output_x.map(&channel_x.input_target_position);
  pantograph_kinematics.output_y.map(&channel_y.input_target_position);

  // -- Start the stepdance library --
  // This activates the system.
  dance_start();
}

LoopDelay overhead_delay;

void loop() {
  overhead_delay.periodic_call(&report_overhead, 500);

  dance_loop(); // Stepdance loop provides convenience functions, and should be called at the end of the main loop
}

void report_overhead(){
  Serial.println(stepdance_get_cpu_usage(), 4);
}