#define module_driver
#include "stepdance.hpp"

ThresholdGenerator threshold_gen;
InputPort input_a;
Channel  channel_a;
OutputPort output_a;

void setup() {

   // Initialize OutputPort A
  output_a.begin(OUTPUT_A);
    // Enable the output drivers
  enable_drivers();

  // Initialize InputPort A
  input_a.begin(INPUT_A);
  input_a.output_x.map(threshold_gen.input); // Map SIGNAL_X to ThresholdGenerator input

  // Configure ThresholdGenerator
  threshold_gen.setLowerThreshold(10.0, true); // Set lower threshold at 10.0 with clamping
  threshold_gen.setUpperThreshold(90.0, true); // Set upper threshold at 90.0 with clamping
  threshold_gen.setLowerCallback(&onLowerThresholdCrossed); // Set callback for lower threshold crossing
  threshold_gen.setUpperCallback(&onUpperThresholdCrossed); // Set callback for upper threshold crossing

  // Map ThresholdGenerator output to Channel A's target position
  threshold_gen.output.map(channel_a.input_target_position);
  threshold_gen.begin();

  // Initialize Channel A
  channel_a.begin(&output_a, SIGNAL_E); // Connect Channel A to OutputPort A's SIGNAL_E
  dance_start();
} 
void onLowerThresholdCrossed() {
  Serial.println("Lower threshold crossed!");
  // Additional actions can be added here
}  

void onUpperThresholdCrossed() {
  Serial.println("Upper threshold crossed!");
  // Additional actions can be added here
}
 
void loop(){
dance_loop();
}
