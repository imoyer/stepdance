#include <SD.h>

#define module_driver

// Machine Selection
// Choose one of the two machines below
#define axidraw 
// #define pocket_plotter
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
// KinematicsCoreXY axidraw_kinematics;

// Knobs input
Encoder encoder_1; 
Encoder encoder_2; 

// Slider input
AnalogInput analog_a1; //extrusion rate controller

// -- Define Input Button --
Button button_d1; 
Button button_d2;

// -- Serial connection --
SerialConnectionGenerator connection_generator;

// -- Position Generator for Pen Up/Down --
// PositionGenerator position_gen;


char incomingByte = 0;


void setup() {

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
  channel_a.set_ratio(25.4, 2032);
  channel_b.set_ratio(25.4, 2032);
#endif
#ifdef pocket_plotter
  channel_a.set_ratio(40, 3200); // Sets the input/output transmission ratio for the channel.
  channel_b.set_ratio(40, 3200);
#endif
                                                // This provides a convenience of converting between input units and motor (micro)steps
                                                // For the pocket plotter, 40mm == 3200 steps (1/16 microstepping)

  channel_a.set_ratio(1, 1);
  channel_b.set_ratio(1, 1);

  channel_z.begin(&output_c, SIGNAL_E); //servo motor, so we use a long pulse width
  // channel_z.set_ratio(1, 50); //straight step pass-thru.
  channel_z.set_ratio(1, 1); //straight step pass-thru.

  // -- Configure Serial generator --
  connection_generator.output_1.map(&channel_a.input_target_position);
  connection_generator.output_2.map(&channel_b.input_target_position);
  connection_generator.output_3.map(&channel_z.input_target_position);
  connection_generator.begin();


  // knob or pedal
  analog_a1.set_floor(0, 25);
  analog_a1.set_ceiling(2, 1020);
  analog_a1.begin(IO_A1);


  dance_start();

  Serial.println("started!");
}

LoopDelay overhead_delay;

void loop() {


  dance_loop();
  overhead_delay.periodic_call(&report_overhead, 100);
  
  // Reset serial buffer
  // incomingByte = 0;
}


void report_overhead(){

  // int knob_value = encoder_2.read();

  // Serial.printf("%d\n", knob_value);

  // Serial.printf("%d\n", incomingByte);


  // Serial.print("running\n");

}
