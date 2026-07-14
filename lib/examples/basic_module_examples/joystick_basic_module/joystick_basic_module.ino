/*
Joystick Basic Module

Example project for the Stepdance control system.

A part of the Mixing Metaphors Project

// (c) 2025 Ilan Moyer, Jennifer Jacobs, Devon Frost
*/

#define module_basic   // tells compiler we're using the Stepdance Driver Module PCB
                        // This configures pin assignments for the Teensy 4.1

#include "stepdance.hpp"  // Import the stepdance library

// -- Define Input Ports --
// We'll use these for pass-thru of any signals, so you can chain modules.
InputPort input_a; //input pass-thru

// -- Define Output Ports --
OutputPort output_a;  // Joystick Output

// -- Define Motion Channels --
Channel channel_x;
Channel channel_y;
Channel channel_z;

// -- Define Analog Inputs --
AnalogInput left_right_a1;
AnalogInput up_down_a2;
AnalogInput twist_a4;

// -- Define Velocity Generators --
VelocityGenerator x_gen;
VelocityGenerator y_gen;
VelocityGenerator z_gen;

const float32_t top_speed_mm_per_sec = 50;

void setup() {
  // -- Configure and start the output ports --
  output_a.begin(OUTPUT_A);

  // -- Configure and start the channels --
  channel_x.begin(&output_a, SIGNAL_X);
  channel_x.set_ratio(STANDARD_RATIO_MM);

  channel_y.begin(&output_a, SIGNAL_Y);
  channel_y.set_ratio(STANDARD_RATIO_MM);

  channel_z.begin(&output_a, SIGNAL_Z);
  channel_z.set_ratio(STANDARD_RATIO_MM);

  // -- Configure and start the input port --
  input_a.begin(INPUT_A);
  input_a.set_ratio(STANDARD_RATIO_MM); //sets the ratio for all signals.

  input_a.output_x.map(&channel_x.input_target_position);
  input_a.output_y.map(&channel_y.input_target_position);
  input_a.output_z.map(&channel_z.input_target_position);

  // -- Configure and start the knob --
  left_right_a1.begin(IO_A1);
  left_right_a1.set_floor(-top_speed_mm_per_sec);
  left_right_a1.set_ceiling(top_speed_mm_per_sec);
  left_right_a1.set_deadband_here(0, 50);
  left_right_a1.invert();
  left_right_a1.map(&x_gen.speed_units_per_sec);

  up_down_a2.begin(IO_A2);
  up_down_a2.set_floor(-top_speed_mm_per_sec);
  up_down_a2.set_ceiling(top_speed_mm_per_sec);
  up_down_a2.set_deadband_here(0, 50);
  up_down_a2.invert();
  up_down_a2.map(&y_gen.speed_units_per_sec);

  twist_a4.begin(IO_A4);
  twist_a4.set_floor(-top_speed_mm_per_sec/2.0);
  twist_a4.set_ceiling(top_speed_mm_per_sec/2.0);
  twist_a4.set_deadband_here(0, 50);
  // twist_a4.invert();
  twist_a4.map(&z_gen.speed_units_per_sec);

  // -- Configure and start the velocity generators --
  x_gen.begin();
  x_gen.output.map(&channel_x.input_target_position);
  
  y_gen.begin();
  y_gen.output.map(&channel_y.input_target_position);

  z_gen.begin();
  z_gen.output.map(&channel_z.input_target_position);

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
  Serial.print("CONVERTED: ");
  Serial.println(up_down_a2.read());
  Serial.print("RAW: ");
  Serial.println(up_down_a2.last_value_raw);
  Serial.print("slope 1: ");
  Serial.println(up_down_a2.conversion_slope_1);
  Serial.print("intercept 1: ");
  Serial.println(up_down_a2.conversion_intercept_1);
  Serial.print("slope 2: ");
  Serial.println(up_down_a2.conversion_slope_2);
  Serial.print("intercept 2: ");
  Serial.println(up_down_a2.conversion_intercept_2);
  Serial.print("deadband_upper: ");
  Serial.println(up_down_a2.adc_deadband_upper);
  Serial.print("deadband_lower: ");
  Serial.println(up_down_a2.adc_deadband_lower);  

}