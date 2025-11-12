// Will move a channel to a target position at a specified maximum velocity when a button is pressed.
#define module_driver
#include "stepdance.hpp"

PositionGenerator pos_gen;
Channel channel_a;
OutputPort output_a;
Button start_button;

void setup() {

   // Initialize OutputPort A
  output_a.begin(OUTPUT_A);
    // Enable the output drivers
  enable_drivers();

  // Map PositionGenerator output to Channel A's target position
  pos_gen.output.map(channel_a.input_target_position);
  pos_gen.begin();

  // Initialize Channel A
  channel_a.begin(&output_a, SIGNAL_E); // Connect Channel A to OutputPort A's SIGNAL_E

  // Set speed for PositionGenerator
  pos_gen.set_speed(50.0); // Set velocity to 50 units per second

  start_button.begin(IO_D1); // Initialize button on physical input port IO_D1  
  start_button.set_callback_on_release(&onButtonRelease); // Set callback function for button press event

  dance_start();
} 
void onButtonRelease() {
  // Command PositionGenerator to move to target position when button is released
  pos_gen.go(100.0, ABSOLUTE); // Move to absolute position of 100.0 units
}
 
void loop(){
dance_loop();
}
