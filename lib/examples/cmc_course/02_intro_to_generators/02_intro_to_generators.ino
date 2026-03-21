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

// -- Define Encoders --
// Encoders read quadrature input signals and we can map the signal to a channel or other elements
Encoder encoder_1;
Encoder encoder_2;


// -- Define Input Button --
// Just like the step-a-sketch example, we use two buttons for pen up / pen down
Button button_d1;
Button button_d2;

// -- Position Generator for Pen Up/Down --
PositionGenerator position_gen;

// -- RPC Interface --
RPC rpc;

// -- Velocity Generator --
// TODO: declare velocity generator

// -- Wave Generator --
// TODO: declare WaveGenerator1D

// -- Circle Generator --
// TODO: declare CircleGenerator


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

  // -- Configure and start the encoders --
  encoder_1.begin(ENCODER_1); // "ENCODER_1" specifies the physical port on the PCB
  encoder_1.set_ratio(24, 2400);  // 24mm per revolution, where 1 rev == 2400 encoder pulses
  encoder_1.output.map(&axidraw_kinematics.input_x);

  encoder_2.begin(ENCODER_2); // "ENCODER_2" specifies the physical port on the PCB
  encoder_2.set_ratio(24, 2400);  // 24mm per revolution, where 1 rev == 2400 encoder pulses
  encoder_2.output.map(&axidraw_kinematics.input_y);

  // -- Configure the velocity generator --
  // TODO

  // -- Configure the wave generator --
  // TODO

  // -- Configure the circle generator --
  // TODO
  // Configure CircleGenerator with radius 10mm, 1 revolution per second
  circle_gen.radius = 1.0;
  circle_gen.rotational_speed_rev_per_sec = 10.0;
  circle_gen.setNoInput(); // Use internal frame count

  // Map circle outputs to X and Y channels
  circle_gen.output_x.map(&axidraw_kinematics.input_x);
  circle_gen.output_y.map(&axidraw_kinematics.input_y);
  circle_gen.begin();


  // -- Configure and start the kinematics module --
  axidraw_kinematics.begin();
  axidraw_kinematics.output_a.map(&channel_a.input_target_position);
  axidraw_kinematics.output_b.map(&channel_b.input_target_position);


  // -- Configure Button --
  button_d1.begin(IO_D1, INPUT_PULLDOWN);
  button_d1.set_mode(BUTTON_MODE_TOGGLE);
  button_d1.set_callback_on_press(&pen_up);
  button_d1.set_callback_on_release(&pen_down);

  // -- Control interface (RPC) --
  rpc.begin(); 

  // Call example: {"name": "hello"}
  // expected result: serial monitor prints "hello!{"result":"ok"}"
  rpc.enroll("hello", hello_serial);

  // Call example: {"name": "set_speed_y", "args": [5]}
  rpc.enroll("set_speed_y", set_speed_y);

  // Call example: {"name": "set_z_amplitude", "args": [1]}
  rpc.enroll("set_z_amplitude", set_z_amplitude);

  // Call example: {"name": "set_circle_radius", "args": [1]}
  rpc.enroll("set_circle_radius", set_circle_radius);

  // -- Configure Position Generator --
  position_gen.output.map(&channel_z.input_target_position);
  position_gen.begin();

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

void hello_serial(){
  Serial.print("hello!");
}

void set_speed_y(float32_t speed){
  // TODO: modify to set the vertical velocity generator speed
}

void set_z_amplitude(float32_t amplitude){
  // TODO: modify to set the wave generator amplitude
}

void set_circle_radius(float32_t radius){
  // TODO: modify to set the circle generator radius parameter
}

void report_overhead(){

}