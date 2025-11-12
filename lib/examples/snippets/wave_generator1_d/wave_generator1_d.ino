// Will create an oscillation in the motion of a channel based on analog inputs for amplitude, frequency, and phase
#define module_driver
#include "stepdance.hpp"

WaveGenerator1D wave_gen;
InputPort input_a;
Channel channel_a;
OutputPort output_a;

AnalogInput analog_a1; //amplitude pot
AnalogInput analog_a2; //frequency pot    
AnalogInput analog_a3; //rotary pot

void setup() {

   // Initialize OutputPort A
  output_a.begin(OUTPUT_A);
    // Enable the output drivers
  enable_drivers();

  // Initialize InputPort A
  input_a.begin(INPUT_A);
  input_a.output_x.map(wave_gen.input, INCREMENTAL); // Map SIGNAL_X to WaveGenerator1D input

  // Map WaveGenerator1D output to Channel A's target position
  wave_gen.output.map(channel_a.input_target_position);
  wave_gen.begin();

  // Initialize Channel A
  channel_a.begin(&output_a, SIGNAL_E); // Connect Channel A to OutputPort A's SIGNAL_E

 // Initialize Analog Inputs for controlling amplitude and frequency
  analog_a1.set_floor(0, 25);
  analog_a1.set_ceiling(10, 1020);
  analog_a1.map(&wave_gen.amplitude);
  analog_a1.begin(IO_A1);

  analog_a2.set_floor(0.1, 25); //avoid zero frequency
  analog_a2.set_ceiling(5.0, 1020);
  analog_a2.map(&wave_gen.frequency);
  analog_a2.begin(IO_A2);

  analog_a3.set_floor(0, 25);
  analog_a3.set_ceiling(6.28, 1020); //0 to 2pi
  analog_a3.map(&wave_gen.phase);
  analog_a3.begin(IO_A3);

  dance_start();
} 
 
void loop(){
dance_loop();
}
