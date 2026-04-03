#define module_driver //specify the type of stepdance board you are programming
#include "stepdance.hpp" //include the stepdance library
 
//DEFINE YOUR COMPONENTS BEFORE SETUP
OutputPort output_a;
Channel channel_a;
OutputPort output_b;
Channel channel_b;
Encoder encoder_1;

void setup() {
//INITIALIZE AND MAP COMPONENTS WITHIN SETUP FUNCTION
output_a.begin(OUTPUT_A);

 // Enable the output drivers
enable_drivers();
// -- Configure and start the channel --
channel_a.begin(&output_a,SIGNAL_E);// Connects the channel to the "E" signal on "output_a".
                                      
channel_a.set_ratio(25.4, 2032); // Sets the input/output transmission ratio for the channel.
                                 // This provides a convenience of converting between input units and motor (micro)steps
                                 // For the axidraw plotter steppers, 25.4mm == 2032 steps

// -- Configure and start the encoder --
encoder_1.begin(ENCODER_1);
encoder_1.set_ratio(24, 2400);  // The ratio of the encoder controls the total "output distance" in real world units for one revolution
                                // of the encoder. There are 2400 pulses per rotation of the encoder. 
                                //A full turn will output 24 mm.
encoder_1.invert();

// -- Map the encoder to the channel. This will automatically update the channel target position whenever the encoder is moved.
encoder_1.output.map(&channel_a.input_target_position);


// -- Start the stepdance library --
// This activates the system.
dance_start();

}
 
void loop(){
//CALL DANCE LOOP TO AUTOMATICALLY UPDATE STEPDANCE MAPPINGS
dance_loop();
}
