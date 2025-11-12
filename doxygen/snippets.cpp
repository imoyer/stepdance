 // [OutputPort]
#define module_driver
#include "stepdance.hpp"

OutputPort output_a; 
Channel channel_a;

void setup() {
  // -- Configure and start the output port --
  output_a.begin(OUTPUT_A); // "OUTPUT_A" specifies the physical port on the PCB for the output.
    // -- Configure and start the channels --
  channel_a.begin(&output_a, SIGNAL_E); // Connects the channel to the "E" signal on "output_a".
                                        // We choose the "E" signal because it results in a step pulse of 7us,
                                        // which is more than long enough for the driver IC
  channel_a.set_ratio(40, 3200); // Sets the input/output transmission ratio for the channel.
}

void loop(){
dance_loop();
}
// [OutputPort]
  // [Channel]
  #define module_driver
#include "stepdance.hpp"

Channel channel_x;
OutputPort output_x;    

void setup() {
  // -- Configure and start the output port --
  output_x.begin(OUTPUT_A); // "OUTPUT_A" specifies the physical port on the PCB for the output.
    // -- Configure and start the channels --
  channel_x.begin(&output_x, SIGNAL_X); // Connects the channel to the "X" signal on "output_x".
                                        // We choose the "X" signal because it results in a step pulse of 2us,
                                        // which is more than long enough for the driver IC
  channel_x.set_ratio(40, 3200); // Sets the input/output transmission ratio for the channel.
} 

void loop(){
dance_loop();
}
// [Channel]
  // [InputPort]
#define module_driver
#include "stepdance.hpp"

InputPort input_a; 
 
Channel channel_z;
KinematicsCoreXY axidraw_kinematics;

void setup() {
  // -- Configure and start the input port --
  input_a.begin(INPUT_A); // "INPUT_A" specifies the physical port on the PCB for the input.
  // -- Map InputPort x, y, and z Output BlockPorts to kinematics object BlockPorts --
  input_a.output_x.set_ratio(0.01, 1); //for this mapping, 1 step is 0.01mm
  input_a.output_x.map(&axidraw_kinematics.input_x);

  input_a.output_y.set_ratio(0.01, 1); //1 step is 0.01mm
  input_a.output_y.map(&axidraw_kinematics.input_y);

  input_a.output_z.set_ratio(0.01, 1); //1 step is 0.01mm
  input_a.output_z.map(&channel_z.input_target_position);
}

void loop(){
dance_loop();
}
  // [InputPort]

    // [BlockPort]
#define module_driver
#include "stepdance.hpp"

InputPort inputA;
Channel channelX;
OutputPort outputA;
void setup() {
  // -- Configure and start the output port --
  outputA.begin(OUTPUT_A); // Initialize OutputPort A
    // Enable the output drivers
  enable_drivers();
    // -- Configure and start the channel --
  channelX.begin(&outputA, SIGNAL_X); // Connect Channel X to OutputPort A
  // -- Configure and start the input port --
  inputA.begin(INPUT_A); // Initialize InputPort A
  inputA.output_x.map(channelX.input_target_position); // Map SIGNAL_X to Channel X's input target position
}

void loop(){
dance_loop();
}
  // [BlockPort] 

   //  [AnalogInput]
#define module_driver
#include "stepdance.hpp"

AnalogInput analog_a1;
AnalogInput analog_a2;

VelocityGenerator velocity_gen;


void setup() {
//analog input a1 
  analog_a1.set_floor(0, 25); //set floor to 0 output at ADC values of 25 and below. 
  analog_a1.set_ceiling(2.75, 1020); //set ceiling to 2.75 output at ADC values of 1020 and above.
  analog_a1.map(&velocity_gen.speed_units_per_sec); //map analog output to VelocityGenerator speed ControlParameter
  analog_a1.begin(IO_A1); //initialize analog input on physical input port IO_A1

//analog input a2
  analog_a2.set_floor(0, 25);
  analog_a2.set_ceiling(10, 1020);
  analog_a2.begin(IO_A2);//initialize analog input on physical input port IO_A2
  
  dance_start();

  Serial.begin(115200);

}

void loop() {
  float extrusionMultiplier = analog_a2.read(); //read analog input a2 value during loop and store it in variable
  Serial.println(extrusionMultiplier); //print the read value to the serial console
  dance_loop(); 
}
     // [AnalogInput]

   //  [AnalogInputCallback]
#define module_driver
#include "stepdance.hpp"

AnalogInput analog_a1;



void setup() {
//analog input a1 
  analog_a1.begin(IO_A1); //initialize analog input on physical input port IO_A1
  analog_a1.set_callback(&analogInputCallback); //set the callback function to be called on new data
  
  dance_start();

}

void analogInputCallback() {
  float inputValue = analog_a1.read(); //read analog input a1 value during callback
  // Do something with inputValue, e.g., print it to the console
  Serial.println(inputValue);
}
 
void loop(){
dance_loop();
}
  //  [AnalogInputCallback]  

   // [Button]
#define module_driver
#include "stepdance.hpp"

Button button_d1;
Button button_d2;

void setup() {
  button_d1.begin(IO_D1); //initialize button on physical input port IO_D1
  button_d1.set_callback_on_release(&onButtonD1Release); //set callback function for button D1 release event
  button_d2.begin(IO_D2); //initialize button on physical input port IO_D2
  button_d2.set_mode(BUTTON_MODE_TOGGLE); //set button D2 to toggle mode
  button_d2.set_callback_on_toggle(&onButtonD2Toggle); //set callback function for button D2 press event
  
  dance_start();
}

void onButtonD1Release() {
  // Code to execute when button D1 is released
  Serial.println("Button D1 Released");
}

void onButtonD2Toggle() {
  // Code to execute when button D2 is toggled
  Serial.println("Button D2 Toggled");
}

void loop(){
dance_loop();
}
  // [Button]

   // [ThresholdGenerator]
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
  // [ThresholdGenerator]
   
   // [WaveGenerator1D]
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
  // [WaveGenerator1D]

   // [PositionGenerator]
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
  // [PositionGenerator]

  // [VelocityGenerator]
  //will drive a channel at a constant speed. WARNING, if no limits are implemented, the axis will drive until it crashes.
#define module_driver
#include "stepdance.hpp"

VelocityGenerator vel_gen;
Channel channel_x;
OutputPort output_a;

void setup() {
  // Initialize OutputPort and enable drivers
  output_a.begin(OUTPUT_A);
  enable_drivers();

  // Map VelocityGenerator output to channel target position
  vel_gen.output.map(channel_x.input_target_position);
  vel_gen.begin();
  vel_gen.speed_units_per_sec = 20.0; // example speed

  // Initialize channel on an output signal
  channel_x.begin(&output_a , SIGNAL_E);

  dance_start();
}

void loop(){
  dance_loop();
}
  // [VelocityGenerator]

  // [PathLengthGenerator2D]
#define module_driver
#include "stepdance.hpp"

CircleGenerator circle_gen;
PathLengthGenerator2D path_gen;
Channel channel_x;
Channel channel_y;
Channel channel_z;
OutputPort output_a;

void setup(){
  // Initialize OutputPort and enable drivers
  output_a.begin(OUTPUT_A);
  enable_drivers();

  // Configure CircleGenerator with radius 10mm, 1 revolution per second
  circle_gen.radius = 10.0;
  circle_gen.rotational_speed_rev_per_sec = 1.0;
  circle_gen.setNoInput(); // Use internal frame count
  circle_gen.begin();

  // Map circle outputs to X and Y channels
  circle_gen.output_x.map(channel_x.input_target_position);
  circle_gen.output_y.map(channel_y.input_target_position);
  channel_x.begin(&output_a, SIGNAL_X);
  channel_y.begin(&output_a, SIGNAL_Y);

  // Configure PathLengthGenerator2D to move Z 1mm per complete circle
  // The circle has radius 10mm, so we want 1mm output per 2π*10mm = 62.83mm of XY path
  path_gen.set_ratio_for_circle(10.0, 1.0); // 1mm per revolution of a 10mm radius circle
  path_gen.begin();

  // Map circle outputs to path length inputs
  circle_gen.output_x.map(path_gen.input_1);
  circle_gen.output_y.map(path_gen.input_2);

  // Map path length output to Z channel
  path_gen.output.map(channel_z.input_target_position);
  channel_z.begin(&output_a, SIGNAL_Z);

  dance_start();
}

void loop(){
  dance_loop();
}
  // [PathLengthGenerator2D]

  // [PathLengthGenerator3D]
#define module_driver
#include "stepdance.hpp"

PathLengthGenerator3D path3_gen;
InputPort input_b;
Channel channel_p3;
OutputPort output_p3;

void setup(){
  // Initialize OutputPort and enable drivers
  output_p3.begin(OUTPUT_A);
  enable_drivers();

  // Initialize InputPort and map X/Y/Z to path length inputs
  input_b.begin(INPUT_A);
  input_b.output_x.map(path3_gen.input_1);
  input_b.output_y.map(path3_gen.input_2);
  input_b.output_z.map(path3_gen.input_3);

  // Configure and begin generator
  path3_gen.set_ratio(1.0);
  path3_gen.begin();

  // Map generator output to a channel
  path3_gen.output.map(channel_p3.input_target_position);
  channel_p3.begin(&output_p3, SIGNAL_E);

  dance_start();
}

void loop(){
  dance_loop();
}
  // [PathLengthGenerator3D]

  // [CircleGenerator]
  // Example of using CircleGenerator to create circular motion in X-Y plane (assuming an Cartesian X-Y mechanism)
#define module_driver
#include "stepdance.hpp"

CircleGenerator circle_gen;
Channel channel_x;
Channel channel_y;
OutputPort output_x;
OutputPort output_y;

void setup(){
  // Initialize OutputPort and enable drivers
  output_x.begin(OUTPUT_A);
  output_y.begin(OUTPUT_B);
  enable_drivers();

  // Configure circle generator to run without input
  circle_gen.setNoInput();
  circle_gen.radius = 10.0; // example radius in units
  circle_gen.rotational_speed_rev_per_sec = 0.5; // example frequency in rev/s
  circle_gen.begin();

  // Map generator outputs to two channels (X and Y)
  circle_gen.output_x.map(channel_x.input_target_position);
  circle_gen.output_y.map(channel_y.input_target_position);
  channel_x.begin(&output_x, SIGNAL_E);
  channel_y.begin(&output_y, SIGNAL_E);

  dance_start();
}

void loop(){
  dance_loop();
}
  // [CircleGenerator]