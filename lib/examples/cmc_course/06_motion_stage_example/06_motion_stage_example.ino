/*
Single Motor Demonstration

Example project for the Stepdance control system.

A part of the Mixing Metaphors Project

// (c) 2026 Ilan Moyer, Jennifer Jacobs, Emilie Yu, Alejandro Aponte
*/

#define module_driver   // tells compiler we're using the Stepdance Driver Module PCB
                        // This configures pin assignments for the Teensy 4.1

#include "stepdance.hpp"  // Import the stepdance library

// -- Define Output Ports --


OutputPort output_a;  // motor for motion stage


// -- Define Motion Channels --

Channel channel_a;  //motion channel for motor


// -- Define Encoders --
Encoder encoder_1;  // left knob, controls horizontal

// -- Define Generators --
VelocityGenerator velocityGen;

// -- RPC Interface --
RPC rpc;

void setup() {
  // -- Configure and start the output ports --
  output_a.begin(OUTPUT_A); // "OUTPUT_A" specifies the physical port on the PCB for the output.


  // Enable the output drivers
  enable_drivers();

  // -- Configure and start the channels --
  channel_a.begin(&output_a, SIGNAL_E); // Connects the channel to the "E" signal on "output_a".
                                        // We choose the "E" signal because it results in a step pulse of 7us,
                                        // which is more than long enough for the driver IC.

  // Ratio calculation Linear distance = number of rotations x Lead
  // we are using a 2 lead screw = 2 mm per revolution
  // stepper  does 3200 steps per rev for 1/16 microsteping.            
  channel_a.set_ratio(2, 3200); // Sets the input/output transmission ratio for the channel.
                                                // This provides a convenience of converting between input units and motor (micro)steps
                                                // For the axidraw, 25.4mm == 2874 steps
  channel_a.invert_output();  // CALL THIS TO INVERT THE MOTOR DIRECTION IF NEEDED

  // -- Configure and start the encoders --
  encoder_1.begin(ENCODER_1); // "ENCODER_1" specifies the physical port on the PCB
  encoder_1.set_ratio(1, 2400);  // 24mm per revolution, where 1 rev == 2400 encoder pulses
                                  //We're using a 600CPR encoder, which generates 4 edge transitions per cycle.
  encoder_1.invert(); //invert the encoder direction
  encoder_1.output.map(&channel_a.input_target_position);

  velocityGen.begin();
  velocityGen.output.map(&channel_a.input_target_position);

  // -- Control interface (RPC) --
  rpc.begin(); 

  
  // Call example: {"name": "set_speed_y", "args": [5]}
  rpc.enroll("set_speed", set_speed);



  // -- Start the stepdance library --
  // This activates the system.
  dance_start();
}

LoopDelay overhead_delay;

void loop() {
  overhead_delay.periodic_call(&report_overhead, 500);

  dance_loop(); // Stepdance loop provides convenience functions, and should be called at the end of the main loop
}


void report_overhead(){
  Serial.println(stepdance_get_cpu_usage(), 4);
}

void set_speed(float32_t speed){
  // TODO: modify to set the vertical velocity generator speed
   velocityGen.speed_units_per_sec = speed;

}