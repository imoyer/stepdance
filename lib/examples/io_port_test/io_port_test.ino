/*
IO Port Test

This example sketch outputs multiple signals on an output port, and then
reads back in the received signals to check that they match.

This can be used as a self-test to ensure that the input and outputs work properly,
e.g. that the level converter chips have been properly soldered.

Example project for the Stepdance control system.

A part of the Mixing Metaphors Project

// (c) 2025 Ilan Moyer, Jennifer Jacobs, Devon Frost
*/

#define module_driver   // tells compiler we're using the Stepdance Driver Module PCB
                        // This configures pin assignments for the Teensy 4.1

#include "stepdance.hpp"  // Import the stepdance library

// -- Define Output Ports --
// Output ports generate step and direction electrical signals

OutputPort output_port;

// -- Define Motion Channels --
// Channels track target positions and interface with output ports

Channel channel_x;
Channel channel_y;
Channel channel_r;
Channel channel_t;
Channel channel_z;
Channel channel_e; 

// -- Button to trigger test --
Button button_d1;

// -- Input Variables --
// We'll use these to track whatever has been read in on the input port.
DecimalPosition in_x = 0;
DecimalPosition in_y = 0;
DecimalPosition in_r = 0;
DecimalPosition in_t = 0;
DecimalPosition in_z = 0;
DecimalPosition in_e = 0;

// -- Input Port --
InputPort input_port;

void setup() {
  // -- Configure and start the output port --
  output_port.begin(OUTPUT_D, OUTPUT_FRAME_32US, OUTPUT_TRANSMIT_ON_FRAME); // We'll use "D" because it has both a driver socket and an output jack

  // -- Configure and start the channels --
  channel_x.begin(&output_port, SIGNAL_X);
  channel_y.begin(&output_port, SIGNAL_Y);
  channel_r.begin(&output_port, SIGNAL_R);
  channel_t.begin(&output_port, SIGNAL_T);
  channel_z.begin(&output_port, SIGNAL_Z);
  channel_e.begin(&output_port, SIGNAL_E);

  // -- Configure and start the input port --
  input_port.begin(INPUT_A);
  input_port.map(SIGNAL_X, &in_x);
  input_port.map(SIGNAL_Y, &in_y);
  input_port.map(SIGNAL_R, &in_r);
  input_port.map(SIGNAL_T, &in_t);
  input_port.map(SIGNAL_Z, &in_z);
  input_port.map(SIGNAL_E, &in_e);

  button_d1.begin(IO_D1, INPUT_PULLDOWN);
  button_d1.set_mode(BUTTON_MODE_STANDARD);
  button_d1.set_callback_on_press(&button_press);

  // -- Start Serial Port --
  Serial.begin(115200);

  // -- Start the stepdance library --
  // This activates the system.
  dance_start();
}

LoopDelay report;

void loop() {
  dance_loop(); // Stepdance loop
  report.periodic_call(&report_input_values, 500);
}

void button_press(){
  channel_x.input_target_position.write(10000, INCREMENTAL);
  channel_y.input_target_position.write(-10000, INCREMENTAL);
  // channel_r.target_position += 10000;
  // channel_t.target_position -= 10000;
  channel_z.input_target_position.write(10000, INCREMENTAL);
  channel_e.input_target_position.write(-10000, INCREMENTAL);
}

void report_input_values(){
  Serial.print("X: ");
  Serial.print(in_x);
  Serial.print(", Y: ");
  Serial.print(in_y); 
  // Serial.print(", R: ");
  // Serial.print(in_r);
  // Serial.print(", T: ");
  // Serial.print(in_t);
  Serial.print(", Z: ");
  Serial.print(in_z);
  Serial.print(", E: ");
  Serial.println(in_e);
}