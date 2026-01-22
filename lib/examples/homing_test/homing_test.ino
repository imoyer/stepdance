/*
Step-A-Sketch: A digital etch-a-sketch

Example project for the Stepdance control system.

A part of the Mixing Metaphors Project

// (c) 2025 Ilan Moyer, Jennifer Jacobs, Devon Frost
*/

#define module_driver   // tells compiler we're using the Stepdance Driver Module PCB
                        // This configures pin assignments for the Teensy 4.1

// Machine Selection
// Choose one of the two machines below
// #define axidraw 
#define pocket_plotter

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

// -- Define Kinematics --
// Kinematics convert between two coordinate spaces.
// We think in XY, but the axidraw moves in AB according to "CoreXY" (also "HBot") kinematics
KinematicsCoreXY axidraw_kinematics;


// -- Position Generator for Pen Up/Down --
PositionGenerator position_gen;

// -- Time based interpolator (used for testing the coordinate system) --
TimeBasedInterpolator tbi;

// -- Homing --
Homing homing;

// -- RPC Interface --
RPC rpc;

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

#ifdef axidraw
  channel_a.set_ratio(25.4, 2874);
  channel_b.set_ratio(25.4, 2874);
#endif
#ifdef pocket_plotter
  channel_a.set_ratio(40, 3200); // Sets the input/output transmission ratio for the channel.
  channel_b.set_ratio(40, 3200);
#endif
                                                // This provides a convenience of converting between input units and motor (micro)steps
                                                // For the pocket plotter, 40mm == 3200 steps (1/16 microstepping)

  channel_z.begin(&output_c, SIGNAL_E); //servo motor, so we use a long pulse width
  channel_z.set_ratio(1, 50); //straight step pass-thru.


  // -- Configure and start the kinematics module --
  axidraw_kinematics.begin();
  axidraw_kinematics.output_a.map(&channel_a.input_target_position);
  axidraw_kinematics.output_b.map(&channel_b.input_target_position);

  // -- Configure Position Generator --
  position_gen.output.map(&channel_z.input_target_position);
  position_gen.begin();

  // -- Configure Homing --
  init_homing();

  // TBI (can be used to test that the homing works properly)
  tbi.begin();
  tbi.output_x.map(&axidraw_kinematics.input_x);
  tbi.output_y.map(&axidraw_kinematics.input_y);

  rpc.begin();

  // Start the homing routine: axes will move until the limit switch button is hit
  // See init_homing() function to configure the homing routine and axes
  // {"name": "home_axes"}
  rpc.enroll("home_axes", home_axes);

  // {"name": "go_to_xy", "args": [6, 5, 10]}
  // args are: absolute X, absolute Y, speed (mm/s)
  rpc.enroll("go_to_xy", go_to_xy);

  // -- Start the stepdance library --
  // This activates the system.
  dance_start();
}

LoopDelay overhead_delay;

void loop() {
  overhead_delay.periodic_call(&report_overhead, 500);
  dance_loop(); // Stepdance loop provides convenience functions, and should be called at the end of the main loop
}

void pen_down(){
  position_gen.go(-4, ABSOLUTE, 100);
}

void pen_up(){
  position_gen.go(4, ABSOLUTE, 100);
}

// This function registers the two axes we want to home in (X and Y)
// and provides specific info about the physical machine setup (homing button pins, home coordinates).
// The order in which the axes are added specifies the order in which they will be homed (eg first X then Y here).
// Note that the axes to home are workspace axes (XY) and not stepper motor axes (AB).
// Generally, we want to home in the axes that we think of as "design" axes, those in which we want to be able to specify absolute coordinates in.
void init_homing() {
  Serial.println("Initialized homing");

  homing.add_axis(
  IO_D1,                         // Stepdance board port for the limit switch
  0,                             // Coordinate value we want to assign at the limit switch
  HOMING_DIR_BWD,                // Direction in which the machine should jog (backward or forward?) to hit the switch
  5,                             // Speed at which the machine should jog to find the limit 
  &axidraw_kinematics.input_x    // Blockport corresponding to the axis to home
  );

  homing.add_axis(
  IO_D2,                         // Stepdance board port for the limit switch
  0,                             // Coordinate value we want to assign at the limit switch
  HOMING_DIR_BWD,                // Direction in which the machine should jog (backward or forward?) to hit the switch
  5,                             // Speed at which the machine should jog to find the limit 
  &axidraw_kinematics.input_y    // Blockport corresponding to the axis to home
  );

  homing.begin();
}

void home_axes() {
  // Calling this method launches the homing routine (machine will move until it hits its limit switches)
  homing.start_homing_routine();
  Serial.println("start homing");
    
}

void go_to_xy(float x, float y, float v) {

  tbi.add_move(GLOBAL, v, x, y, 0, 0, 0, 0); // mode, vel, x, y, 0, 0, 0, 0

}

void report_overhead(){

  Serial.println("button values (LIMIT_A, LIMIT_B, LIMIT_C, IO_D1, IO_D2):");
  Serial.print(digitalReadFast(LIMIT_A));
  Serial.print(",");
  Serial.print(digitalReadFast(LIMIT_B));
  Serial.print(",");
  Serial.print(digitalReadFast(LIMIT_C));
  Serial.print(",");
  Serial.print(digitalReadFast(IO_D1));
  Serial.print(",");
  Serial.print(digitalReadFast(IO_D2));
  Serial.print("\n");

  Serial.print("channel A: ");
  Serial.print(channel_a.input_target_position.read(ABSOLUTE), 4);
  Serial.print(" channel B: ");
  Serial.print(channel_b.input_target_position.read(ABSOLUTE), 4);
  Serial.print(" axidraw X: ");
  Serial.print(axidraw_kinematics.input_x.read(ABSOLUTE), 4);
    Serial.print(" axidraw Y: ");
  Serial.print(axidraw_kinematics.input_y.read(ABSOLUTE), 4);
  Serial.print("\n");
  // Serial.println(stepdance_get_cpu_usage(), 4);
}