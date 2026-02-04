#include <SD.h>

#define module_driver

// Machine Selection
// Choose one of the two machines below
// #define axidraw 
#define pocket_plotter
#include "stepdance.hpp"

// Pantograph XY input plugged in input_a
// InputPort input_a;

// Axidraw XYZ plugged into output ports
OutputPort output_a;
OutputPort output_b;
OutputPort output_c;

// Corresponding channels for the Axidraw
Channel channel_a;
Channel channel_b;
Channel channel_z;

// -- Define Kinematics --
// Kinematics convert between two coordinate spaces.
// We think in XY, but the axidraw moves in AB according to "CoreXY" (also "HBot") kinematics
KinematicsCoreXY axidraw_kinematics;

// -- Position Generator for Pen Up/Down --
PositionGenerator position_gen;

// -- Time based interpolator (used for sending XY motion commands from the p5 sketch) --
TimeBasedInterpolator tbi;

// -- Homing --
Homing homing;

// -- Wave 2D Generator and utility functions --
WaveGenerator2D wave2d_gen;
Vector2DToAngle vec2angle;
MoveDurationToFrequency durationToFreq;

// -- Remote Procedure Call --
// This is the object that will listen for messages over Serial, and trigger callback functions.
// In setup, we will map callback functions to specific Serial messages.
// The Python server will be responsible for sending 
RPC rpc;

char incomingByte = 0;


void setup() {
  // Make sure the baudrate matches between this number and the one in p5 sketch: serial.open(serialPort, { baudrate: 115200});
  // NB: in Stepdance RPC code, 115200 is the baudrate used, so we stick to that throughout.
  Serial.begin(115200);

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

  // -- Configure wave 2D generator --

  vec2angle.input_x.map(&tbi.output_x);
  vec2angle.input_y.map(&tbi.output_y);
  vec2angle.output_theta.map(&wave2d_gen.input_theta, ABSOLUTE);
  vec2angle.begin();

  wave2d_gen.output_x.map(&axidraw_kinematics.input_x);
  wave2d_gen.output_y.map(&axidraw_kinematics.input_y);
  wave2d_gen.input_t.map(&tbi.output_parameter, ABSOLUTE);
  wave2d_gen.begin();
  wave2d_gen.amplitude = 0.0;

  durationToFreq.input_move_duration.map(&tbi.output_duration, ABSOLUTE);
  durationToFreq.output_frequency.map(&wave2d_gen.input_frequency);
  durationToFreq.target_frequency = 1.0;
  durationToFreq.begin();


  // -- RPC Configuration
  rpc.begin();

  // Call example: {"name": "hello"}
  // expected result: serial monitor prints "hello!{"result":"ok"}"
  rpc.enroll("hello", say_hello);

  // Start the homing routine: axes will move until the limit switch button is hit
  // See init_homing() function to configure the homing routine and axes
  // {"name": "home_axes"}
  rpc.enroll("home_axes", home_axes);

  rpc.enroll("pen_down", pen_down);
  rpc.enroll("pen_up", pen_up);


  // {"name": "go_to_xy", "args": [6, 5, 10]}
  // args are: absolute X, absolute Y, speed (mm/s)
  rpc.enroll("go_to_xy", go_to_xy);

  // {"name": "set_noise_freq", "args": [5]}
  rpc.enroll("set_noise_freq", set_noise_freq);
  // {"name": "set_noise_amp", "args": [1]}
  rpc.enroll("set_noise_amp", set_noise_amp);


  dance_start();

  Serial.println("started!");
}

LoopDelay overhead_delay;

float64_t incoming_offsets[3]; // Fixed at 3 values here for now (TODO: this should be defined based on the input port, within the plugin code)
String incoming_str[3];

void loop() {


  dance_loop();
  overhead_delay.periodic_call(&report_overhead, 100);
}

void pen_down(){
  position_gen.go(-4, ABSOLUTE, 100);
}

void pen_up(){
  position_gen.go(4, ABSOLUTE, 100);
}

void say_hello(){
  Serial.println("hello!");
}

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

void set_noise_amp(float amp) {
  wave2d_gen.amplitude = amp;
  Serial.println(wave2d_gen.amplitude);
}

void set_noise_freq(float freq) {
  durationToFreq.target_frequency = freq;
  Serial.println(durationToFreq.target_frequency);
}

void report_overhead(){

  // wave2d_gen.debugPrint();
}