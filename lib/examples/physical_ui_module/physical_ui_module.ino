/*
Digital In Test

Displays current digital values, on A1->A4, D1->D2

A part of the Mixing Metaphors Project

// (c) 2025 Ilan Moyer, Jennifer Jacobs, Devon Frost
*/

// #define module_driver   // tells compiler we're using the Stepdance Driver Module PCB
#define module_basic   // tells compiler we're using the Stepdance Basic Module PCB
                        // This configures pin assignments for the Teensy 4.1

#include "stepdance.hpp"  // Import the stepdance library

OutputPort output_a;

Channel channel_amp; // amplitude channel
Channel channel_freq; // frequency channel
Channel channel_phase; // phase channel
Channel channel_ext; // extruder channel

AnalogInput analog_a1; //rotary pot
AnalogInput analog_a2; //linear pot
AnalogInput analog_a3; //rotary pot
AnalogInput analog_a4; //rotary pot

float64_t layerHeight = 2.0;
float64_t nozzleDiameter = 4.0;
volatile float64_t extrusionMultiplier = 1.0;
volatile float64_t extrusionRate = 0.0;


void setup() {
  output_a.begin(OUTPUT_A);
 

  channel_amp.begin(&output_a, SIGNAL_X);
  channel_freq.begin(&output_a, SIGNAL_Y);
  channel_phase.begin(&output_a, SIGNAL_Z);
  channel_ext.begin(&output_a, SIGNAL_E);

  channel_amp.enable();
  channel_freq.enable();
  channel_phase.enable();

   //extrusion rate
  analog_a1.set_floor(0, 1020);
  analog_a1.set_ceiling(10, 25); // extrusion multiplier 
  analog_a1.begin(IO_A1);


  //amplitude
  analog_a2.set_floor(-4, 1020);
  analog_a2.set_ceiling(4, 25); 
  analog_a2.begin(IO_A2);
  analog_a2.map(&channel_amp.input_target_position.absolute_buffer);

  //frequency
  analog_a3.set_floor(1, 1020);
  analog_a3.set_ceiling(10, 25); 
  analog_a3.begin(IO_A3);
  analog_a3.map(&channel_freq.input_target_position.absolute_buffer);

  //phase
  analog_a4.set_floor(1, 1020);
  analog_a4.set_ceiling(10, 25);
  analog_a4.begin(IO_A4);
  analog_a4.map(&channel_phase.input_target_position.absolute_buffer);

  dance_start();
}

LoopDelay overhead_delay;

void loop() {
  
  extrusionMultiplier = analog_a1.read();
  float64_t segmentLength = 1.0;
  extrusionRate = (4*layerHeight * extrusionMultiplier * nozzleDiameter * segmentLength) / (PI*nozzleDiameter*nozzleDiameter);
  channel_ext.input_target_position.write(extrusionRate, ABSOLUTE);

  dance_loop();
  report_digital_values();
}


void report_digital_values(){
  Serial.print("amplitude:");
  Serial.print(channel_amp.target_position);
    Serial.print(" , freq:");
  Serial.print(channel_freq.target_position);
    Serial.print(" , phase:");
  Serial.print(channel_phase.target_position);
    Serial.print(" , extrusion:");
  Serial.println(channel_ext.target_position);

}