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

PathLengthGenerator2D path2_gen;
Channel channel_x;
Channel channel_y;
Channel channel_z;
OutputPort output_a

void setup(){
  // Initialize OutputPort and enable drivers
  output_p2.begin(OUTPUT_A);
  enable_drivers();

  // Initialize InputPort and map X/Y to path length inputs
  input_a.begin(INPUT_A);
  input_a.output_x.map(path2_gen.input_1);
  input_a.output_y.map(path2_gen.input_2);

  // Configure and begin generator
  path2_gen.set_ratio(1.0);
  path2_gen.begin();

  // Map generator output to a channel
  path2_gen.output.map(channel_p2.input_target_position);
  channel_p2.begin(&output_p2, SIGNAL_E);

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

  // [Encoder]
#define module_driver
#include "stepdance.hpp"

Encoder encoder;
Channel channel_a;  
OutputPort output_a;

void setup(){
  // Initialize OutputPort and enable drivers
  output_a.begin(OUTPUT_A);
  enable_drivers();

  channel_a.begin(&output_a, SIGNAL_E); // Initialize Channel A on Output A

  // Initialize Encoder and map to output
  encoder.begin(ENCODER_1);
  encoder.set_ratio(24, 2400);  // 24mm per revolution, where 1 rev == 2400 encoder pulses
  encoder.output.map(output_a.input_position);

  dance_start();
}

void loop(){
  dance_loop();
}
  // [Encoder]

    // [ScalingFilter1D]
#define module_driver
#include "stepdance.hpp"
InputPort input_a;
ScalingFilter1D scale_filter;
Channel channel_x;
OutputPort output_a;

void setup(){
  // Initialize OutputPort and enable drivers
  output_a.begin(OUTPUT_A);
  enable_drivers();

  // Initialize Channel
  channel_x.begin(&output_a, SIGNAL_E);

  input_a.begin(INPUT_A);
  input_a.output_x.map(scale_filter.input); // Map InputPort to ScalingFilter1D input

  // Initialize ScalingFilter1D and map to channel
  scale_filter.set_ratio(2.0); // Example ratio- will scale inpyut by factor of 2
  scale_filter.begin();
  scale_filter.output.map(channel_x.input_target_position);

  dance_start();
}

void loop(){
  dance_loop();
}
    // [ScalingFilter1D]

    // [ScalingFilter2D]
#define module_driver
#include "stepdance.hpp"
InputPort input_a;
ScalingFilter2D scale_filter;
Channel channel_x;
Channel channel_y;
OutputPort output_a;
OutputPort output_b;

void setup(){
  // Initialize OutputPort and enable drivers
  output_a.begin(OUTPUT_A);
  output_b.begin(OUTPUT_B);
  enable_drivers();

  // Initialize Channel
  channel_x.begin(&output_a, SIGNAL_E);
  channel_y.begin(&output_b, SIGNAL_E);

  input_a.begin(INPUT_A);
  input_a.output_x.map(scale_filter.input_1); // Map InputPort to ScalingFilter2D input_1
  input_a.output_y.map(scale_filter.input_2); // Map InputPort to ScalingFilter2D input_2

  // Initialize ScalingFilter2D and map to channels
  scale_filter.set_ratio(2.0); // Example ratio- will scale inputs by factor of 2
  scale_filter.begin();
  scale_filter.output_1.map(channel_x.input_target_position);
  scale_filter.output_2.map(channel_y.input_target_position);

  dance_start();
}

void loop(){
  dance_loop();
}
    // [ScalingFilter2D]

    // [KinematicsCoreXY]
#define module_driver
#include "stepdance.hpp"
Encoder enc_1;
Encoder enc_2;
KinematicsCoreXY corexy_kinematics;
Channel channel_a;
Channel channel_b;
OutputPort output_a;
OutputPort output_b;

void setup(){
  // Initialize OutputPorts and enable drivers
  output_a.begin(OUTPUT_A);
  output_b.begin(OUTPUT_B);
  enable_drivers();

  // Initialize Channels
  channel_a.begin(&output_a, SIGNAL_E);
  channel_b.begin(&output_b, SIGNAL_E);

  // Initialize Encoders
  enc_1.begin(ENCODER_1);
  enc_1.set_ratio(24, 2400);  // 24mm per revolution, where 1 rev == 2400 encoder pulses
  enc_2.begin(ENCODER_2);
  enc_2.set_ratio(24, 2400);  // 24mm per revolution, where 1 rev == 2400 encoder pulses

  // Map Encoder outputs to KinematicsCoreXY inputs
  enc_1.output.map(corexy_kinematics.input_x);
  enc_2.output.map(corexy_kinematics.input_y);

  // Initialize KinematicsCoreXY and map to channels
  corexy_kinematics.begin();
  corexy_kinematics.output_x.map(channel_a.input_target_position);
  corexy_kinematics.output_y.map(channel_b.input_target_position);

  dance_start();
} 

void loop(){
  dance_loop();
}
    // [KinematicsCoreXY]

    // [KinematicsPolarToCartesian]
#define module_driver
#include "stepdance.hpp"
AnalogInput angle_input;
Encoder radius_enc;
KinematicsPolarToCartesian polar_to_cartesian_kinematics;
VelocityGenerator angle_velocity_gen;
Channel channel_x;
Channel channel_y;
OutputPort output_x;
OutputPort output_y;  

void setup(){
  // Initialize OutputPorts and enable drivers
  output_x.begin(OUTPUT_A);
  output_y.begin(OUTPUT_B);
  enable_drivers();

  // Initialize Channels
  channel_x.begin(&output_x, SIGNAL_E);
  channel_y.begin(&output_y, SIGNAL_E);

  vel_genocity_gen.begin();
  angle_velocity_gen.output.map(polar_to_cartesian_kinematics.input_angle); 


  
  // Initialize AnalogInput for angle
  angle_input.set_floor(0, 25); // Set floor to 0 output at ADC values of 25 and below.
  angle_input.set_ceiling(3.28, 1020); //radians per second

  angle_input.map(&angle_velocity_gen.speed_units_per_sec); // Map analog output to VelocityGenerator speed ControlParameter
  angle_input.begin(IO_A1);

  // Initialize Encoder for radius
  radius_enc.begin(ENCODER_1);
  radius_enc.set_ratio(1, 2400);  
  radius_enc.output.map(polar_to_cartesian_kinematics.input_radius);

  // Initialize KinematicsPolarToCartesian and map to channels
  polar_to_cartesian_kinematics.begin();
  polar_to_cartesian_kinematics.output_x.map(channel_x.input_target_position);
  polar_to_cartesian_kinematics.output_y.map(channel_y.input_target_position);

  dance_start();
} 
void loop(){
  dance_loop();
}
    // [KinematicsPolarToCartesian]


    // [Homing]
#define module_driver 

// Machine Selection
// Choose one of the two machines below
// #define axidraw 
#define pocket_plotter

#include "stepdance.hpp"  // Import the stepdance library
// -- Define Input Ports --
InputPort input_a;

// -- Define Output Ports --
// Output ports generate step and direction electrical signals
// Here, we control two stepper drivers and a servo driver
// We choose names that match the labels on the PCB

OutputPort output_a;  // Axidraw left motor
OutputPort output_b;  // Axidraw right motor

// -- Define Motion Channels --
// Channels track target positions and interface with output ports
// Generally, we need a channel for each output port
// We choose names that match the axes of the AxiDraw's motors

Channel channel_a;  //AxiDraw "A" axis --> left motor motion
Channel channel_b;  // AxiDraw "B" axis --> right motor motion

// -- Define Kinematics --
// Kinematics convert between two coordinate spaces.
// We think in XY, but the axidraw moves in AB according to "CoreXY" (also "HBot") kinematics
KinematicsCoreXY axidraw_kinematics;

// -- Time based interpolator (used for testing the coordinate system) --
TimeBasedInterpolator tbi;

// -- Homing --
Homing homing;

// -- RPC Interface --
RPC rpc;

void setup() {

  // -- Configure and start the output ports --
  output_a.begin(OUTPUT_A); // "OUTPUT_A" specifies the physical port on the PCB for the output.
  output_b.begin(OUTPUT_B);

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

  // -- Configure and start the kinematics module --
  axidraw_kinematics.begin();
  axidraw_kinematics.output_a.map(&channel_a.input_target_position);
  axidraw_kinematics.output_b.map(&channel_b.input_target_position);

  // -- Configure Homing --
  init_homing();

  // TBI (can be used to test that the homing works properly)
  tbi.begin();
  tbi.output_x.map(&axidraw_kinematics.input_x);
  tbi.output_y.map(&axidraw_kinematics.input_y);

  rpc.begin();

  // Start the homing routine: axes will move until the limit switch button is hit
  // See init_homing() function to configure the homing routine and axes
  // {"name": "home_axes"}
  rpc.enroll("home_axes", home_axes);

  // {"name": "go_to_xy", "args": [6, 5, 10]}
  // args are: absolute X, absolute Y, speed (mm/s)
  rpc.enroll("go_to_xy", go_to_xy);

  // -- Start the stepdance library --
  // This activates the system.
  dance_start();
}

void loop() {
  dance_loop(); // Stepdance loop provides convenience functions, and should be called at the end of the main loop
}


// This function registers the two axes we want to home in (X and Y)
// and provides specific info about the physical machine setup (homing button pins, home coordinates).
// The order in which the axes are added specifies the order in which they will be homed (eg first X then Y here).
// Note that the axes to home are workspace axes (XY) and not stepper motor axes (AB).
// Generally, we want to home in the axes that we think of as "design" axes, those in which we want to be able to specify absolute coordinates in.
void init_homing() {
  Serial.println("Initialized homing");

  homing.add_axis(
  LIMIT_A,                         // Stepdance board port for the limit switch
  0,                               // Coordinate value we want to assign at the limit switch
  HOMING_DIR_BWD,                  // Direction in which the machine should jog (backward or forward?) to hit the switch
  5,                               // Speed at which the machine should jog to find the limit 
  &axidraw_kinematics.input_x      // Blockport corresponding to the axis to home
  );

  homing.add_axis(
  LIMIT_B,                         // Stepdance board port for the limit switch
  0,                               // Coordinate value we want to assign at the limit switch
  HOMING_DIR_BWD,                  // Direction in which the machine should jog (backward or forward?) to hit the switch
  5,                               // Speed at which the machine should jog to find the limit 
  &axidraw_kinematics.input_y      // Blockport corresponding to the axis to home
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

    // [Homing]

    // [TimeBasedInterpolator]

#define module_driver
#include "stepdance.hpp" 

// -- Define Input Ports --
InputPort input_a;

// -- Define Output Ports --
OutputPort output_a;  // Axidraw left motor
OutputPort output_b;  // Axidraw right motor
OutputPort output_c;  // Z axis, a servo driver for the AxiDraw

// -- Define Motion Channels --
Channel channel_a;  //AxiDraw "A" axis --> left motor motion
Channel channel_b;  // AxiDraw "B" axis --> right motor motion

// -- Define Kinematics --
// Kinematics convert between two coordinate spaces.
// We think in XY, but the axidraw moves in AB according to "CoreXY" (also "HBot") kinematics
KinematicsCoreXY axidraw_kinematics;

TimeBasedInterpolator tbi;

RPC rpc;

void setup() {
  // -- Configure and start the output ports --
  output_a.begin(OUTPUT_A); // "OUTPUT_A" specifies the physical port on the PCB for the output.
  output_b.begin(OUTPUT_B);

  // Enable the output drivers
  enable_drivers();

  // -- Configure and start the channels --
  channel_a.begin(&output_a, SIGNAL_E); // Connects the channel to the "E" signal on "output_a".
                                        // We choose the "E" signal because it results in a step pulse of 7us,
                                        // which is more than long enough for the driver IC
  channel_a.invert_output();  // CALL THIS TO INVERT THE MOTOR DIRECTION IF NEEDED
  channel_b.begin(&output_b, SIGNAL_E);
  channel_b.invert_output();

  // Axidraw
  channel_a.set_ratio(25.4, 2874);
  channel_b.set_ratio(25.4, 2874);

  // -- Configure and start the input port --
  input_a.begin(INPUT_A);
  input_a.output_x.set_ratio(0.01, 1); //1 step is 0.01mm
  input_a.output_x.map(&axidraw_kinematics.input_x);

  input_a.output_y.set_ratio(0.01, 1); //1 step is 0.01mm
  input_a.output_y.map(&axidraw_kinematics.input_y);

  // -- Configure and start the kinematics module --
  axidraw_kinematics.begin();
  axidraw_kinematics.output_a.map(&channel_a.input_target_position);
  axidraw_kinematics.output_b.map(&channel_b.input_target_position);


  tbi.begin();
  tbi.output_x.map(&axidraw_kinematics.input_x);
  tbi.output_y.map(&axidraw_kinematics.input_y);

  rpc.begin();

  // {"name": "go_to_xy", "args": [6, 5, 10]}
  // args are: absolute X, absolute Y, speed (mm/s)
  rpc.enroll("go_to_xy", go_to_xy);

  // {"name": "corner_test"}
  rpc.enroll("corner_test", corner_test);

  // {"name": "draw_letter_h_at", "args": [10, 10]}
  // args are: start X, start Y
  rpc.enroll("draw_letter_h_at", draw_letter_h_at);


  // -- Start the stepdance library --
  // This activates the system.
  dance_start();
}

void loop() {
  dance_loop();
}


void go_to_xy(float x, float y, float v) {
  tbi.add_move(GLOBAL, v, x, y, 0, 0, 0, 0); // mode, vel, x, y, 0, 0, 0, 0

}

void corner_test() {
  tbi.add_move(GLOBAL, 10, 0, 0, 0, 0, 0, 0); // mode, vel, x, y, 0, 0, 0, 0
  tbi.add_move(GLOBAL, 10, 50, 0, 0, 0, 0, 0); // mode, vel, x, y, 0, 0, 0, 0
  tbi.add_move(GLOBAL, 10, 50, 30, 0, 0, 0, 0); // mode, vel, x, y, 0, 0, 0, 0

}

void draw_letter_h_at(float x_start, float y_start) {
  // Coordinates will draw a capital letter H
  tbi.add_move(GLOBAL, 10, x_start, y_start, 0, 0, 0, 0); // mode, vel, x, y, 0, 0, 0, 0
  tbi.add_move(GLOBAL, 10, x_start + 10, y_start, 0, 0, 0, 0); 
  tbi.add_move(GLOBAL, 10, x_start + 10, y_start + 20, 0, 0, 0, 0); 
  tbi.add_move(GLOBAL, 10, x_start + 30, y_start + 20, 0, 0, 0, 0); 
  tbi.add_move(GLOBAL, 10, x_start + 30, y_start, 0, 0, 0, 0); 
  tbi.add_move(GLOBAL, 10, x_start + 40, y_start, 0, 0, 0, 0); 
  tbi.add_move(GLOBAL, 10, x_start + 40, y_start + 50, 0, 0, 0, 0); 
  tbi.add_move(GLOBAL, 10, x_start + 30, y_start + 50, 0, 0, 0, 0); 
  tbi.add_move(GLOBAL, 10, x_start + 30, y_start + 30, 0, 0, 0, 0); 
  tbi.add_move(GLOBAL, 10, x_start + 10, y_start + 30, 0, 0, 0, 0); 
  tbi.add_move(GLOBAL, 10, x_start + 10, y_start + 50, 0, 0, 0, 0); 
  tbi.add_move(GLOBAL, 10, x_start, y_start + 50, 0, 0, 0, 0); 
  tbi.add_move(GLOBAL, 10, x_start, y_start, 0, 0, 0, 0); 
}


    // [TimeBasedInterpolator]

    // [WaveGenerator2D]

#define module_driver   // tells compiler we're using the Stepdance Driver Module PCB
                        // This configures pin assignments for the Teensy 4.1
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

// -- Define Inputs --
AnalogInput analog_a1; // foot pedal controlling the wave amplitude

// -- Position Generator for Pen Up/Down --
PositionGenerator position_gen;

// -- Wave 2D Generator and utility functions --
WaveGenerator2D wave2d_gen;
Vector2DToAngle vec2angle;
MoveDurationToFrequency durationToFreq;

TimeBasedInterpolator tbi;

RPC rpc;

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

  // For Axidraw
  channel_a.set_ratio(25.4, 2874);
  channel_b.set_ratio(25.4, 2874);

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

  // -- Configure and start the kinematics module --
  axidraw_kinematics.begin();
  axidraw_kinematics.output_a.map(&channel_a.input_target_position);
  axidraw_kinematics.output_b.map(&channel_b.input_target_position);


  // -- Configure Position Generator --
  position_gen.output.map(&channel_z.input_target_position);
  position_gen.begin();

  tbi.begin();
  tbi.output_x.map(&axidraw_kinematics.input_x);
  tbi.output_y.map(&axidraw_kinematics.input_y);
  tbi.output_parameter.map(&wave2d_gen.input_t, ABSOLUTE);

  // -- Configure wave 2D generator --

  vec2angle.input_x.map(&tbi.output_x);
  vec2angle.input_y.map(&tbi.output_y);
  vec2angle.output_theta.map(&wave2d_gen.input_theta, ABSOLUTE);
  vec2angle.begin();

  wave2d_gen.output_x.map(&axidraw_kinematics.input_x);
  wave2d_gen.output_y.map(&axidraw_kinematics.input_y);
  wave2d_gen.begin();

  durationToFreq.input_move_duration.map(&tbi.output_duration, ABSOLUTE);
  durationToFreq.output_frequency.map(&wave2d_gen.input_frequency);
  durationToFreq.target_frequency = 1.0;
  durationToFreq.begin();

  // Map pedal value to wave amplitude
  analog_a1.set_floor(0, 25);
  analog_a1.set_ceiling(10, 1020); //radians per second
  analog_a1.map(&wave2d_gen.amplitude);
  analog_a1.begin(IO_A1);

  wave2d_gen.amplitude = 0.0;

  rpc.begin();

  rpc.enroll("wave2d_gen", wave2d_gen);

  // {"name": "home_axes"}
  rpc.enroll("home_axes", home_axes);

  // {"name": "go_to_xy", "args": [6, 5, 10]}
  // args are: absolute X, absolute Y, speed (mm/s)
  rpc.enroll("go_to_xy", go_to_xy);

  // {"name": "set_noise_freq", "args": [5]}
  rpc.enroll("set_noise_freq", set_noise_freq);
  // {"name": "set_noise_amp", "args": [1]}
  rpc.enroll("set_noise_amp", set_noise_amp);

  // {"name": "corner_test"}
  rpc.enroll("corner_test", corner_test);

  // {"name": "draw_letter_h_at", "args": [10, 10]}
  // args are: start X, start Y
  rpc.enroll("draw_letter_h_at", draw_letter_h_at);


  // -- Start the stepdance library --
  // This activates the system.
  dance_start();
}

void loop() {

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


void go_to_xy(float x, float y, float v) {
  tbi.add_move(GLOBAL, v, x, y, 0, 0, 0, 0); // mode, vel, x, y, 0, 0, 0, 0
}

void corner_test() {
  tbi.add_move(GLOBAL, 10, 20, 20, 0, 0, 0, 0); // mode, vel, x, y, 0, 0, 0, 0
  tbi.add_move(GLOBAL, 10, 70, 20, 0, 0, 0, 0); // mode, vel, x, y, 0, 0, 0, 0
  tbi.add_move(GLOBAL, 10, 70, 50, 0, 0, 0, 0); // mode, vel, x, y, 0, 0, 0, 0
  tbi.add_move(GLOBAL, 10, 20, 20, 0, 0, 0, 0); // mode, vel, x, y, 0, 0, 0, 0
}

void set_noise_amp(float amp) {
  wave2d_gen.amplitude = amp;
}

void set_noise_freq(float freq) {
  durationToFreq.target_frequency = freq;
}

    // [WaveGenerator2D]