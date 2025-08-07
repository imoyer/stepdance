#define module_driver

#include "stepdance.hpp"

InputPort input_a;

OutputPort output_a;
OutputPort output_b;
OutputPort output_c;
OutputPort output_d;

Channel channel_a;
Channel channel_b;
Channel channel_z;
Channel channel_e;

KinematicsPolarToCartesian polar_kinematics;

AnalogInput analog_a1; //foot pedal
AnalogInput analog_a2; //linear pot
AnalogInput analog_a3; //linear pot
AnalogInput analog_a4; //rotary pot

Button digital_d1; //toggle button
Button digital_d2; //toggle button

Encoder encoder_1; //hand lever
Encoder encoder_2; //z babystep

VelocityGenerator velocity_gen;

ScalingFilter1D z_gen;
ScalingFilter1D x_stretch;


PathLengthGenerator2D e_gen; //generates extruder signal

WaveGenerator1D xy_wave_generator;
WaveGenerator1D z_wave_generator;


float64_t layerHeight = 2.0;
float64_t nozzleDiameter = 4.0;
volatile float64_t extrusionRate = 0;

void setup() {
  input_a.begin(INPUT_A);
  input_a.set_ratio(0.05); //needs to match the output ratio in physical_ui_module.ino
  input_a.output_y.set_ratio(1); //frequency should be an integer
  input_a.output_z.set_ratio(0.01);

  output_a.begin(OUTPUT_A);
  output_b.begin(OUTPUT_B);
  output_c.begin(OUTPUT_C);
  output_d.begin(OUTPUT_D);

  enable_drivers();

  channel_a.begin(&output_a, SIGNAL_E);
  channel_a.set_ratio(1, 40);
  channel_a.invert_output();
  channel_a.enable_filtering(200);

  channel_b.begin(&output_b, SIGNAL_E);
  channel_b.set_ratio(1, 40);
  channel_b.enable_filtering(200);
  //channel_b.invert_output();

  channel_z.begin(&output_c, SIGNAL_E);
  channel_z.set_ratio(1, 1201); // testing with an 8mm lead leadscrew
  channel_z.invert_output();


  channel_e.begin(&output_d, SIGNAL_E);
  channel_e.set_ratio(1,  8.75); // testing with an 8mm lead leadscrew
  channel_e.invert_output();
  channel_e.enable_filtering(1000);

  velocity_gen.begin();
  velocity_gen.output.map(&polar_kinematics.input_angle);

  encoder_1.begin(ENCODER_1);
  encoder_1.set_ratio(1, 2400); //25mm per revolution
  encoder_1.output.map(&polar_kinematics.input_radius);
  encoder_1.invert();

  encoder_2.begin(ENCODER_2);
  encoder_2.set_ratio(1, 2400); //25mm per revolution
  encoder_2.output.map(&channel_z.input_target_position);
  encoder_2.invert();

  xy_wave_generator.input.map(&polar_kinematics.input_angle,INCREMENTAL);
  xy_wave_generator.output.map(&polar_kinematics.input_radius);
  xy_wave_generator.disable();
  xy_wave_generator.begin();

  z_wave_generator.input.map(&polar_kinematics.input_angle,INCREMENTAL);
  z_wave_generator.output.map(&channel_z.input_target_position);
  z_wave_generator.disable();

  z_wave_generator.begin();

  polar_kinematics.output_x.map(&channel_a.input_target_position);
  polar_kinematics.output_x.map(&x_stretch.input);
  polar_kinematics.output_y.map(&channel_b.input_target_position);
  polar_kinematics.begin();

  z_gen.begin();
  z_gen.set_ratio(layerHeight, TWO_PI); //1mm per rev
  z_gen.input.map(&polar_kinematics.input_angle);
  z_gen.output.map(&channel_z.input_target_position);

  x_stretch.begin(ABSOLUTE);
  x_stretch.output.map(&channel_a.input_target_position);


  e_gen.begin();

  e_gen.input_1.map(&channel_a.input_target_position);
  e_gen.input_2.map(&channel_b.input_target_position);
  e_gen.output.map(&channel_e.input_target_position);

  //pedal
  analog_a1.set_floor(0, 25);
  analog_a1.set_ceiling(2.75, 1020); //radians per second
  analog_a1.map(&velocity_gen.speed_units_per_sec);
  analog_a1.begin(IO_A1);

  //xy amplitude
  analog_a2.set_floor(-10, 25);
  analog_a2.set_ceiling(10, 1020);
  analog_a2.map(&xy_wave_generator.amplitude);
  analog_a2.begin(IO_A2);
  
  //xy phase
  analog_a3.set_floor(0, 25);
  analog_a3.set_ceiling(1, 1020);
  analog_a3.map(&xy_wave_generator.phase);
  analog_a3.begin(IO_A3);

  //xy frequency
  analog_a4.set_floor(0, 25);
  analog_a4.set_ceiling(10, 1020);
  analog_a4.map(&xy_wave_generator.rotational_speed_rev_per_sec);
  analog_a4.begin(IO_A4);

  //toggle button
  digital_d1.set_callback_on_toggle(&button_toggle1);
  digital_d1.begin(IO_D1, INPUT_PULLDOWN);
  digital_d1.set_mode(BUTTON_MODE_TOGGLE);

  //toggle button
  digital_d2.set_callback_on_toggle(&button_toggle2);
  digital_d2.begin(IO_D2, INPUT_PULLDOWN);
  digital_d2.set_mode(BUTTON_MODE_TOGGLE);
  dance_start();
}

LoopDelay overhead_delay;

void loop() {
  float64_t z_amp = input_a.output_x.read(ABSOLUTE);
  float64_t z_freq = input_a.output_y.read(ABSOLUTE);
  // float64_t z_phase =  input_a.output_z.read(ABSOLUTE);
  x_stretch.ratio = (input_a.output_z.absolute_buffer*0.015) + 0.5; //0->1 --> 0.5->2
  float64_t z_phase = 1.0;

  
  z_wave_generator.amplitude = z_amp;
  z_wave_generator.rotational_speed_rev_per_sec = z_freq;
  z_wave_generator.phase = z_phase;

  extrusionRate = input_a.output_e.read(ABSOLUTE);
  e_gen.set_ratio(extrusionRate);

  dance_loop();
  // report_overhead();
  overhead_delay.periodic_call(&report_overhead, 100);



}

void button_toggle1(){
   uint8_t state = digital_d1.read();
   if(state == 0){
     z_wave_generator.disable();
   }
   else if(state == 1){
     z_wave_generator.enable();
   }
}

void button_toggle2(){
   uint8_t state = digital_d2.read();
   if(state == 0){
    xy_wave_generator.disable();
   }
   else if(state == 1){
     xy_wave_generator.enable();
   }
}


void report_overhead(){

  Serial.print("radius:");
  Serial.print(polar_kinematics.input_radius.read(ABSOLUTE));
  Serial.print(", angle:");
  Serial.print(polar_kinematics.input_angle.read(ABSOLUTE));
  Serial.print(", extrusionRate:");
  Serial.print(extrusionRate);
  Serial.print(", xy_enabled:");
  Serial.print(xy_wave_generator.enabled);
  Serial.print(", xy_amp:");
  Serial.print(xy_wave_generator.amplitude);
  Serial.print(", xy_freq:");
  Serial.print(xy_wave_generator.rotational_speed_rev_per_sec);
  Serial.print(", xy_phase:");
  Serial.print(z_wave_generator.phase);
  Serial.print(", z_enabled:");
  Serial.print(z_wave_generator.enabled);
  Serial.print(",z_amp:");
  Serial.print(z_wave_generator.amplitude);
  Serial.print(", z_freq:");
  Serial.print(z_wave_generator.rotational_speed_rev_per_sec);
  Serial.print(", z_phase:");
  Serial.print(z_wave_generator.phase);
  Serial.print(", x_stretch:");
  Serial.println(x_stretch.ratio);
  //wave_generator1D.debugPrint();
  //Serial.println(stepdance_get_cpu_usage(), 4);
 // Serial.println(e_gen.input_1_position, 4);
 // Serial.println(e_gen.input_2_position, 4); 
  /*Serial.print("extrusion_multiplier: ");
  Serial.print(extrusionMultiplier);
 */
  //Serial.println(tiny_circles.radius);
  //Serial.println(tiny_circles.output_x.read(INCREMENTAL));
  //Serial.println(tiny_circles.output_y.read(INCREMENTAL));*/

 
}