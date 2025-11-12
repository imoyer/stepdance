 // [OutputPort]
OutputPort output_a; 

void setup() {
  // -- Configure and start the output port --
  output_a.begin(OUTPUT_A); // "OUTPUT_A" specifies the physical port on the PCB for the output.
}
  // [OutputPort]

   // [Channel]
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
  // [Channel]

  // [InputPort]
InputPort input_a; 
 
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
  // [InputPort]

    // [BlockPort]
InputPort inputA;
inputA.begin(INPUT_A); // Initialize InputPort A
inputA.output_x.map(channelX.input_target_position); // Map SIGNAL_X to Channel X's input target position
     // [BlockPort] 

   //  [AnalogInput]
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

}

void loop() {
  float extrusionMultiplier =analog_a2.read(); //read analog input a2 value during loop and store it in variable 
}
     // [AnalogInput]

   //  [AnalogInputCallback]
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
   //  [AnalogInputCallback]  

   // [Button]
#include "stepdance.hpp"

Button button_a1;
Button button_a2;

void setup() {
  button_a1.begin(IO_A1); //initialize button on physical input port IO_A1
  button_a1.set_callback_on_release(&onButtonA1Release); //set callback function for button A1 release event
  button_a2.begin(IO_A2); //initialize button on physical input port IO_A2
  button_a2.set_callback_on_toggle(&onButtonA2Toggle); //set callback function for button A2 press event
  dance_start();
}

void onButtonA1Release() {
  // Code to execute when button A1 is released
  Serial.println("Button A1 Released");
}

void onButtonA2Toggle() {
  // Code to execute when button A2 is toggled
  Serial.println("Button A2 Toggled");
}
   // [Button]