#include <SD.h>

#define module_driver

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
KinematicsCoreXY axidraw_kinematics;

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
PositionGenerator position_gen;


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


    // #ifdef axidraw
    // channel_a.set_ratio(25.4, 2874);
    // channel_b.set_ratio(25.4, 2874);
    // #endif
    // #ifdef pocket_plotter
    channel_a.set_ratio(40, 3200); // Sets the input/output transmission ratio for the channel.
    channel_b.set_ratio(40, 3200);
    // #endif
                                                // This provides a convenience of converting between input units and motor (micro)steps
                                                // For the pocket plotter, 40mm == 3200 steps (1/16 microstepping)

    channel_z.begin(&output_c, SIGNAL_E); //servo motor, so we use a long pulse width
    channel_z.set_ratio(1, 50); //straight step pass-thru.


    // -- Configure and start the input port --
    // input_a.begin(INPUT_A);
    // input_a.output_x.set_ratio(1, 1); //1 step is 0.01mm
    // input_a.output_x.map(&axidraw_kinematics.input_x);

    // input_a.output_y.set_ratio(0.01, 1); //1 step is 0.01mm
    // input_a.output_y.map(&axidraw_kinematics.input_y);

    // input_a.output_z.set_ratio(0.01, 1); //1 step is 0.01mm
    // input_a.output_z.map(&channel_z.input_target_position);
  



    // -- Configure and start the kinematics module --
    axidraw_kinematics.begin();
    axidraw_kinematics.output_a.map(&channel_a.input_target_position);
    axidraw_kinematics.output_b.map(&channel_b.input_target_position);

    // -- Configure Position Generator --
    position_gen.output.map(&channel_z.input_target_position);
    position_gen.begin();

    // -- Configure Serial generator --
    // below mapping causes a crash at runtime I think
    connection_generator.output_1.map(&axidraw_kinematics.input_x);
    connection_generator.output_2.map(&axidraw_kinematics.input_y);
    // connection_generator.output_1.map(&input_a.output_x);
    // connection_generator.output_2.map(&input_a.output_y);
    connection_generator.begin();

    encoder_1.begin(ENCODER_1);
    encoder_1.set_ratio(1, 2400); 
    // encoder_1.output.map(&channel_e.input_target_position);
    encoder_1.invert();

    encoder_2.begin(ENCODER_2);
    encoder_2.set_ratio(10, 2400); 
    encoder_2.output.map(&channel_z.input_target_position);
    encoder_2.invert();


    //extrusion knob
    analog_a1.set_floor(0, 25);
    analog_a1.set_ceiling(2, 1020);
    analog_a1.begin(IO_A1);


    button_d1.begin(IO_D1, INPUT_PULLDOWN);
    button_d1.set_mode(BUTTON_MODE_TOGGLE);
    // button_d1.set_callback_on_press(&do_something_to_input_A);
    // button_d1.set_callback_on_release(&stop_recording);

    button_d2.begin(IO_D2, INPUT_PULLDOWN);
    button_d2.set_mode(BUTTON_MODE_STANDARD);
    // button_d2.set_callback_on_press(&playback_control);


    dance_start();
}

LoopDelay overhead_delay;
const int SIZE = 26;

float64_t incoming_offsets[3]; // Fixed at 3 values here for now (TODO: this should be defined based on the input port, within the plugin code)
String incoming_str[3];

void loop() {


  dance_loop();
  overhead_delay.periodic_call(&report_overhead, 100);
  
  // Reset serial buffer
  // incomingByte = 0;
}

void pen_down(){
  position_gen.go(-4, ABSOLUTE, 100);
}

void pen_up(){
  position_gen.go(4, ABSOLUTE, 100);
}

void report_overhead(){

  // int knob_value = encoder_2.read();

  // Serial.printf("%d\n", knob_value);

  // Serial.printf("%d\n", incomingByte);


  // Serial.print("running\n");

}
