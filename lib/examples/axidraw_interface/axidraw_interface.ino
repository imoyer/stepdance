/*
AxiDraw Interface

This extends the Step-A-Sketch demo with an InkScape interface that simulates the AxiDraw's
EBB Control Board.

Example project for the Stepdance control system.

A part of the Mixing Metaphors Project

// (c) 2025 Ilan Moyer, Jennifer Jacobs, Devon Frost
*/

#define module_driver   // tells compiler we're using the Stepdance Driver Module PCB
                        // This configures pin assignments for the Teensy 4.1

// Machine Selection
// Choose one of the two machines below
#define axidraw 
// #define pocket_plotter

#include "stepdance.hpp"  // Import the stepdance library
// -- Define Input Ports --
InputPort input_a;

// -- Define Output Ports --
// Output ports generate step and direction electrical signals
// Here, we control two stepper drivers and a servo driver
// We choose names that match the labels on the PCB

OutputPort output_a;  // Axidraw left motor
OutputPort output_b;  // Axidraw right motor
OutputPort output_c;  // Z axis, a servo driver for the AxiDraw

// -- Define Motion Channels --
// Channels track target positions and interface with output ports
// Generally, we need a channel for each output port
// We choose names that match the axes of the AxiDraw's motors

Channel channel_a;  //AxiDraw "A" axis --> left motor motion
Channel channel_b;  // AxiDraw "B" axis --> right motor motion
Channel channel_z;  // AxiDraw "Z" axis --> pen up/down

// -- AxiDraw Interface
Eibotboard ebb_interface;

// -- Define Kinematics --
// Kinematics convert between two coordinate spaces.
// We think in XY, but the axidraw moves in AB according to "CoreXY" (also "HBot") kinematics
KinematicsCoreXY axidraw_kinematics;

// -- Define Encoders --
// Encoders read quadrature input signals and can drive kinematics or other elements
// We use rotary optical encoders for the two etch-a-sketch knobs
Encoder encoder_1;  // left knob, controls horizontal
Encoder encoder_2;  // right knob, controls vertical

// -- Define Input Button --
Button button_d1;
Button button_d2;

// -- Position Generator for Pen Up/Down --
PositionGenerator position_gen;

void setup() {
  // -- Configure and start the output ports --
  output_a.begin(OUTPUT_A); // "OUTPUT_A" specifies the physical port on the PCB for the output.
  output_b.begin(OUTPUT_B);
  output_c.begin(OUTPUT_C);

  // Enable the output drivers
  enable_drivers();

  // -- Configure and start the channels --
  channel_a.begin(&output_a, SIGNAL_E); // Connects the channel to the "E" signal on "output_a".
                                        // We choose the "E" signal because it results in a step pulse of 7us,
                                        // which is more than long enough for the driver IC
  channel_a.invert_output();  // CALL THIS TO INVERT THE MOTOR DIRECTION IF NEEDED
  channel_b.begin(&output_b, SIGNAL_E);
  channel_b.invert_output();

// #ifdef axidraw
  // channel_a.set_ratio(25.4, 2874);
  // channel_b.set_ratio(25.4, 2874);
// #endif
// #ifdef pocket_plotter
  channel_a.set_ratio(40, 3200); // Sets the input/output transmission ratio for the channel.
  channel_b.set_ratio(40, 3200);
// #endif
                                                // This provides a convenience of converting between input units and motor (micro)steps
                                                // For the pocket plotter, 40mm == 3200 steps (1/16 microstepping)

  channel_z.begin(&output_c, SIGNAL_E); //servo motor, so we use a long pulse width
  channel_z.set_ratio(1, 50); //straight step pass-thru.

  // -- Configure and start the input port --
  input_a.begin(INPUT_A);
  input_a.output_x.set_ratio(0.01, 1); //1 step is 0.01mm
  input_a.output_x.map(&axidraw_kinematics.input_x);

  input_a.output_y.set_ratio(0.01, 1); //1 step is 0.01mm
  input_a.output_y.map(&axidraw_kinematics.input_y);

  input_a.output_z.set_ratio(0.01, 1); //1 step is 0.01mm
  input_a.output_z.map(&channel_z.input_target_position);

  // -- Configure and start the encoders --
  encoder_1.begin(ENCODER_1); // "ENCODER_1" specifies the physical port on the PCB
  encoder_1.set_ratio(24, 2400);  // 24mm per revolution, where 1 rev == 2400 encoder pulses
                                  //We're using a 600CPR encoder, which generates 4 edge transitions per cycle.
  encoder_1.invert(); //invert the encoder direction
  encoder_1.output.map(&axidraw_kinematics.input_x);


  encoder_2.begin(ENCODER_2);
  encoder_2.set_ratio(24, 2400);
  encoder_2.invert();
  encoder_2.output.map(&axidraw_kinematics.input_y); // map the right encoder to the y axis input of the kinematics

  // -- Configure and start EBB Interface --
  ebb_interface.begin();
  ebb_interface.output_x.map(&axidraw_kinematics.input_x);
  ebb_interface.output_y.map(&axidraw_kinematics.input_y);
  ebb_interface.output_z.map(&channel_z.input_target_position);

  // -- Configure and start the kinematics module --
  axidraw_kinematics.begin();
  axidraw_kinematics.output_a.map(&channel_a.input_target_position);
  axidraw_kinematics.output_b.map(&channel_b.input_target_position);


  // -- Configure Button --
  button_d1.begin(IO_D1, INPUT_PULLDOWN);
  button_d1.set_mode(BUTTON_MODE_TOGGLE);
  button_d1.set_callback_on_press(&pen_up);
  button_d1.set_callback_on_release(&pen_down);

  button_d2.begin(IO_D2, INPUT_PULLDOWN);
  button_d2.set_mode(BUTTON_MODE_TOGGLE);
  button_d2.set_callback_on_press(&motors_enable);
  button_d2.set_callback_on_release(&motors_disable);

  // -- Configure Position Generator --
  position_gen.output.map(&channel_z.input_target_position);
  position_gen.begin();

  // -- Start the stepdance library --
  // This activates the system.
  dance_start();
}

LoopDelay overhead_delay;

void loop() {
  // overhead_delay.periodic_call(&report_overhead, 500);

  dance_loop(); // Stepdance loop provides convenience functions, and should be called at the end of the main loop
}

void pen_down(){
  position_gen.go(-4, ABSOLUTE, 100);
}

void pen_up(){
  position_gen.go(4, ABSOLUTE, 100);
}

void motors_enable(){
  enable_drivers();
}

void motors_disable(){
  disable_drivers();
}

void report_overhead(){
  Serial.println(channel_z.target_position, 4);
  Serial.println(stepdance_get_cpu_usage(), 4);
}