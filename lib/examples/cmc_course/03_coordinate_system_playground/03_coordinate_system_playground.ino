/*
  UCSB MAT Creative Motion Control -- Generators example
*/

#define module_driver   // tells compiler we're using the Stepdance Driver Module PCB
                        // This configures pin assignments for the Teensy 4.1

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

// -- Define Input Button --
// We use button to engage/disengage motors in this example
Button button_d1;

// -- RPC Interface --
RPC rpc;

// -- Position Generators for Pen XY --
PositionGenerator position_gen_x;
PositionGenerator position_gen_y;

// -- Time Based Interpolators for Pen XY --
TimeBasedInterpolator time_based_interpolator;


void setup() {
  // -- Configure and start the output ports --
  output_a.begin(OUTPUT_A); // "OUTPUT_A" specifies the physical port on the PCB for the output.
  output_b.begin(OUTPUT_B);
  output_c.begin(OUTPUT_C);

  // Enable the output drivers
  enable_drivers();

  // -- Configure and start the channels --
  channel_a.begin(&output_a, SIGNAL_E);
  channel_b.begin(&output_b, SIGNAL_E);

  // These ratios are for the Axidraw V3: 2032 steps correspond to 1 inch (25.4mm)
  channel_a.set_ratio(25.4, 2032);
  channel_a.invert_output(); // We do that so that the X axis points from motor A to motor B (left to right)
  channel_b.set_ratio(25.4, 2032);
  channel_b.invert_output(); // We do that so that the Y axis points down (away from the long axis)

  channel_z.begin(&output_c, SIGNAL_E);
  channel_z.set_ratio(1, 50); //straight step pass-thru.


  // -- Configure and start the kinematics module --
  axidraw_kinematics.begin();
  axidraw_kinematics.output_a.map(&channel_a.input_target_position);
  axidraw_kinematics.output_b.map(&channel_b.input_target_position);


  // -- Configure Button --
  button_d1.begin(IO_D1, INPUT_PULLDOWN);
  button_d1.set_mode(BUTTON_MODE_TOGGLE);
  button_d1.set_callback_on_press(&motors_enable);
  button_d1.set_callback_on_release(&motors_disable);


  // -- Configure Position Generators --
  position_gen_x.output.map(&axidraw_kinematics.input_x);
  position_gen_x.begin();

  position_gen_y.output.map(&axidraw_kinematics.input_y);
  position_gen_y.begin();

  // TBI (can be used to queue motions)
  time_based_interpolator.begin();
  time_based_interpolator.output_x.map(&axidraw_kinematics.input_x);
  time_based_interpolator.output_y.map(&axidraw_kinematics.input_y);
  time_based_interpolator.output_z.map(&channel_z.input_target_position);


  // -- Control interface (RPC) --
  rpc.begin(); 

  // Call example: {"name": "hello"}
  // expected result: serial monitor prints "hello!{"result":"ok"}"
  rpc.enroll("hello", hello_serial);

  // Call example: {"name": "reset_origin"}
  // expected result: this rests the origin of the coordinate system to whatever position we are in right now
  rpc.enroll("reset_origin", reset_origin);

  // Call example: {"name": "motors_enable"}
  // expected result: this enables the motors
  rpc.enroll("motors_enable", motors_enable);

  // Call example: {"name": "motors_disable"}
  // expected result: this disables the motors (so that you can manually move the machine without motors being engaged)
  rpc.enroll("motors_disable", motors_disable);

  // {"name": "go_to_x", "args": [10]}
  // args are: absolute X
  rpc.enroll("go_to_x", go_to_x);

  // {"name": "go_to_y", "args": [10]}
  // args are: absolute Y
  rpc.enroll("go_to_y", go_to_y);

  // {"name": "queue_xy_target", "args": [6, 5]}
  // args are: absolute X, absolute Y
  rpc.enroll("queue_xy_target", queue_xy_target);

  // -- Start the stepdance library --
  // This activates the system.
  dance_start();
}

LoopDelay overhead_delay;

void loop() {
  overhead_delay.periodic_call(&report_overhead, 500);

  dance_loop(); // Stepdance loop provides convenience functions, and should be called at the end of the main loop
}

void motors_enable(){
  enable_drivers();
}

void motors_disable(){
  disable_drivers();
}

void reset_origin() {
  axidraw_kinematics.input_x.reset_deep(0);
  axidraw_kinematics.input_x.reset_deep(0);
}

void go_to_x(float x) {
  // Arguments are: value, mode (GLOBAL ensures we work in absolute coordinates), speed
  position_gen_x.go(x, GLOBAL, 15.0);
}

void go_to_y(float y) {
  // Arguments are: value, mode (GLOBAL ensures we work in absolute coordinates), speed
  position_gen_y.go(y, GLOBAL, 15.0);
}

void queue_xy_target(float x, float y) {

  time_based_interpolator.add_move(GLOBAL, 15.0, x, y, 0, 0, 0, 0); // mode, vel, x, y, 0, 0, 0, 0

}


void hello_serial(){
  Serial.print("hello!");
}

void report_overhead(){
  Serial.println("Positions (X, Y):");
  Serial.print(axidraw_kinematics.input_x.read(ABSOLUTE));
  Serial.print(",");
  Serial.print(axidraw_kinematics.input_y.read(ABSOLUTE));
  Serial.print("\n");
}