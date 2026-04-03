/*
Harmonograph Basic Module

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
OutputPort output_a;  // Harmonograph Output

// -- Define Motion Channels --

Channel channel_x; //ENC1 output channel
Channel channel_y; //ENC2 output channel
Channel channel_z; //for pass-thru
//eventually let's pass thru all channels.

// -- Define Encoders --
Encoder encoder_1;  // left knob, controls horizontal
Encoder encoder_2;  // right knob, controls vertical

// -- Define Analog Inputs --
AnalogInput ratio_a1;

const float32_t encoder_ratio = 50.0/2400.0; // 50mm per 2400 step pulses

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

  // -- Configure and start the encoders --
  encoder_1.begin(ENCODER_1); // "ENCODER_1" specifies the physical port on the PCB
  encoder_1.set_ratio(encoder_ratio);
  encoder_1.invert(); //invert the encoder direction
  encoder_1.output.map(&channel_x.input_target_position);


  encoder_2.begin(ENCODER_2);
  encoder_2.set_ratio(encoder_ratio);
  encoder_2.invert();
  encoder_2.output.map(&channel_y.input_target_position);

  // -- Configure and start the knob --
  ratio_a1.begin(IO_A1);
  ratio_a1.set_floor(encoder_ratio/3);
  ratio_a1.set_ceiling(encoder_ratio*3);

  // -- Start the stepdance library --
  // This activates the system.
  dance_start();
}

LoopDelay overhead_delay;

void loop() {
  overhead_delay.periodic_call(&report_overhead, 500);

  dance_loop(); // Stepdance loop provides convenience functions, and should be called at the end of the main loop
  encoder_1.set_ratio(ratio_a1.read());
  encoder_2.set_ratio(ratio_a1.read());
}

void report_overhead(){
  Serial.println(stepdance_get_cpu_usage(), 4);
}