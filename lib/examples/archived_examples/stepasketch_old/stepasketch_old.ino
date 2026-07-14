/*
Step-A-Sketch: A digital etch-a-sketch

Example project for the Stepdance control system.

A part of the Mixing Metaphors Project

// (c) 2025 Ilan Moyer, Jennifer Jacobs, Devon Frost
*/

#define module_driver   // tells compiler we're using the Stepdance Driver Module PCB
                        // This configures pin assignments for the Teensy 4.1

#include "stepdance.hpp"  // Import the stepdance library

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
                                        // which is more than long enough for the driver IC.
  channel_a.set_transmission_ratio(25.4, 2874); // Sets the input/output transmission ratio for the channel.
                                                // This provides a convenience of converting between input units and motor (micro)steps
                                                // For the axidraw, 25.4mm == 2874 steps
  channel_a.invert_output();  // CALL THIS TO INVERT THE MOTOR DIRECTION IF NEEDED

  channel_b.begin(&output_b, SIGNAL_E);
  channel_b.set_transmission_ratio(25.4, 2874);
  channel_b.invert_output();

  channel_z.begin(&output_c, SIGNAL_E); //servo motor, so we use a long pulse width
  channel_z.set_transmission_ratio(1, 1); //straight step pass-thru.

  // -- Configure and start the encoders --
  encoder_1.begin(ENCODER_1); // "ENCODER_1" specifies the physical port on the PCB
  encoder_1.set_ratio(2400, 24);  // 24mm per revolution, where 1 rev == 2400 encoder pulses
                                  // We're using a 600CPR encoder, which generates 4 edge transitions per cycle.
  encoder_1.map(&axidraw_kinematics.input_transmission_x);  // map the left encoder to the X axis input of the kinematics
  encoder_1.invert(); //invert the encoder direction

  encoder_2.begin(ENCODER_2);
  encoder_2.set_ratio(2400, 24);
  encoder_2.map(&axidraw_kinematics.input_transmission_y); // map the right encoder to the y axis input of the kinematics
  // encoder_2.invert();

  // -- Configure and start the kinematics module --
  axidraw_kinematics.begin();
  axidraw_kinematics.map(COREXY_OUTPUT_A, &channel_a.target_position_transmission); //connect the kinematics to the motion channels
  axidraw_kinematics.map(COREXY_OUTPUT_B, &channel_b.target_position_transmission);

  // -- Configure Button --
  button_d1.begin(IO_D1, INPUT_PULLDOWN);
  button_d1.set_mode(BUTTON_MODE_TOGGLE);
  button_d1.set_callback_on_press(&pen_down);
  button_d1.set_callback_on_release(&pen_up);

  // -- Configure Position Generator --
  position_gen.map(&channel_z.target_position_transmission);
  position_gen.begin();

  // -- Start the stepdance library --
  // This activates the system.
  dance_start();
}

LoopDelay overhead_delay;

void loop() {
  // your custom code here
  overhead_delay.periodic_call(&report_overhead, 500);

  dance_loop(); // Stepdance loop provides convenience functions, and should be called at the end of the main loop
}

void pen_down(){
  position_gen.go_absolute(-200, 2000);
}

void pen_up(){
  position_gen.go_absolute(200, 2000);
}

void report_overhead(){
  Serial.println(stepdance_get_cpu_usage(), 4);
}