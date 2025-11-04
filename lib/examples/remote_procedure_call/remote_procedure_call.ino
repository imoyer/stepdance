/*
Remote Procedure Call Example

Based on the Step-A-Sketch example code for the Stepdance control system.

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

// -- Define Analog Input --
AnalogInput analog_a1;

// -- Position Generator for Pen Up/Down --
PositionGenerator position_gen;

// -- TimeBasedInterpolator

TimeBasedInterpolator interpolator;

// -- Circle Generator

CircleGenerator circles;

// -- Scaling Filter
ScalingFilter2D scaler;

// -- Recording and Playback

FourTrackRecorder recorder;
FourTrackPlayer player;

// -- RPC Interface --
RPC rpc;
DecimalPosition testValue = 1.234;

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

  // -- Configure and start the kinematics module --
  axidraw_kinematics.begin();
  axidraw_kinematics.output_a.map(&channel_a.input_target_position);
  axidraw_kinematics.output_b.map(&channel_b.input_target_position);

  // -- Scaler --
  scaler.begin();
  scaler.output_1.map(&axidraw_kinematics.input_x);
  scaler.output_2.map(&axidraw_kinematics.input_y);

  // -- Configure Circle Generator --
  circles.begin();
  circles.output_x.map(&scaler.input_1);
  circles.output_y.map(&scaler.input_2);

  // -- Configure Button --
  button_d1.begin(IO_D1, INPUT_PULLDOWN);
  button_d1.set_mode(BUTTON_MODE_TOGGLE);
  button_d1.set_callback_on_press(&pen_up);
  button_d1.set_callback_on_release(&pen_down);

  button_d2.begin(IO_D2, INPUT_PULLDOWN);
  button_d2.set_mode(BUTTON_MODE_TOGGLE);
  button_d2.set_callback_on_press(&motors_enable);
  button_d2.set_callback_on_release(&motors_disable);

  // -- Configure Analog Input --
  analog_a1.begin(IO_A1);

  // -- Time Based Interpolator
  interpolator.begin();
  interpolator.output_x.map(&axidraw_kinematics.input_x);
  interpolator.output_y.map(&axidraw_kinematics.input_y);

  // -- Configure Position Generator --
  position_gen.output.map(&channel_z.input_target_position);
  position_gen.begin();

  // -- Configure Recorder
  recorder.begin();
  recorder.input_1.map(&channel_a.input_target_position);
  recorder.input_2.map(&channel_b.input_target_position);
  recorder.input_3.map(&channel_z.input_target_position);

  player.begin();
  player.output_1.map(&channel_a.input_target_position);
  player.output_2.map(&channel_b.input_target_position);
  player.output_3.map(&channel_z.input_target_position);

  // -- RPC Configuration
  rpc.begin(); // defaults to Serial
  rpc.enroll("hello", say_hello);
  rpc.enroll("hello2", say_hello_2);
  rpc.enroll("add", add);
  rpc.enroll("testValue", testValue);
  rpc.enroll("cpu", stepdance_get_cpu_usage);
  rpc.enroll("encoder_1", encoder_1);
  rpc.enroll("analog_a1", analog_a1);
  rpc.enroll("channel_a", channel_a);
  rpc.enroll("button_d1", button_d1);
  rpc.enroll("button_d2", button_d2);
  rpc.enroll("input_a", input_a);
  rpc.enroll("interpolator", interpolator);
  rpc.enroll("axidraw_kinematics", axidraw_kinematics);
  rpc.enroll("output_a", output_a);
  rpc.enroll("position_gen", position_gen);
  rpc.enroll("circles", circles);
  rpc.enroll("scaler", scaler);
  rpc.enroll("recorder", recorder);
  rpc.enroll("player", player);

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

void say_hello(){
  Serial.print("hello!");
}

void say_hello_2(uint8_t num_times){
  Serial.println(num_times);
  for(uint8_t i = 0; i<num_times; i++){
    Serial.println("Hello");
  }
}

DecimalPosition add(DecimalPosition X, DecimalPosition Y){
  return X + Y;
}