#define module_driver

#include "stepdance.hpp"

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
Button button_1; //pushbutton


Encoder encoder_1; //hand lever
Encoder encoder_2; //z babystep

VelocityGenerator velocity_gen;

ScalingFilter1D z_gen;

PathLengthGenerator2D e_gen; //generates extruder signal

WaveGenerator1D xy_wave_generator;

float64_t layerHeight = 2.0;
float64_t nozzleDiameter = 4.0;
volatile float64_t extrusionMultiplier = 1.0;
volatile float64_t extrusionRate = 0.0;


void setup() {
  output_a.begin(OUTPUT_A);
  output_b.begin(OUTPUT_B);
  output_c.begin(OUTPUT_C);
  output_d.begin(OUTPUT_D);

  enable_drivers();

  channel_a.begin(&output_a, SIGNAL_E);
  channel_a.set_ratio(1, 40);
  channel_a.invert_output();

  channel_b.begin(&output_b, SIGNAL_E);
  channel_b.set_ratio(1, 40);
  //channel_b.invert_output();

  channel_z.begin(&output_c, SIGNAL_E);
  channel_z.set_ratio(1, 1201); // testing with an 8mm lead leadscrew
  channel_z.invert_output();


  channel_e.begin(&output_d, SIGNAL_E);
  channel_e.set_ratio(1,  8.75); // testing with an 8mm lead leadscrew
  channel_e.invert_output();

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
  xy_wave_generator.begin();


  polar_kinematics.output_x.map(&channel_a.input_target_position);
  polar_kinematics.output_y.map(&channel_b.input_target_position);
  polar_kinematics.begin();

  z_gen.begin();
  z_gen.set_ratio(layerHeight, TWO_PI); //1mm per rev
  z_gen.input.map(&polar_kinematics.input_angle);
  z_gen.output.map(&channel_z.input_target_position);

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
  analog_a3.set_floor(0, 25);
  analog_a3.set_ceiling(10, 1020);
  analog_a3.map(&xy_wave_generator.amplitude);
  analog_a3.begin(IO_A3);
  
  //xy frequency
  analog_a4.set_floor(0, 25);
  analog_a4.set_ceiling(10, 1020);
  analog_a4.map(&xy_wave_generator.rotational_speed_rev_per_sec);
  analog_a4.begin(IO_A4);


  dance_start();
}

LoopDelay overhead_delay;

void loop() {
 
  overhead_delay.periodic_call(&report_overhead, 500);
  extrusionMultiplier = analog_a3.read();
  float64_t segmentLength = 1.0;
  extrusionRate = (4*layerHeight * extrusionMultiplier * nozzleDiameter * segmentLength) / (PI*nozzleDiameter*nozzleDiameter);
  e_gen.set_ratio(extrusionRate);
  dance_loop();
  report_overhead();


}

void report_overhead(){
  //wave_generator1D.debugPrint();
  //Serial.println(stepdance_get_cpu_usage(), 4);
 // Serial.println(e_gen.input_1_position, 4);
 // Serial.println(e_gen.input_2_position, 4); 
  /*Serial.print("extrusion_multiplier: ");
  Serial.print(extrusionMultiplier);
  Serial.print(", extrusionRate:");
  Serial.println(extrusionRate);*/
  //Serial.println(tiny_circles.radius);
  //Serial.println(tiny_circles.output_x.read(INCREMENTAL));
  //Serial.println(tiny_circles.output_y.read(INCREMENTAL));*/

 
}