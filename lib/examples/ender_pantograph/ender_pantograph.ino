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

Encoder encoder_1; 
Encoder encoder_2; 

AnalogInput analog_a1; //extrusion rate controller


VelocityGenerator velocity_gen;

PathLengthGenerator2D z_gen; //generates z signal

PathLengthGenerator2D e_gen; //generates extruder signal

//need to update
float64_t layerHeight = 0.2;
float64_t nozzleDiameter = 0.4;
volatile float64_t extrusionRate = 0;

void setup() {
  input_a.begin(INPUT_A);

 
  input_a.output_x.set_ratio(0.01, 1); //1 step is 0.01mm
  input_a.output_x.map(&channel_a.input_target_position);

  input_a.output_y.set_ratio(0.01, 1); //1 step is 0.01mm
  input_a.output_y.map(&channel_b.input_target_position);

  /*input_a.output_z.set_ratio(0.01, 1); //1 step is 0.01mm
  input_a.output_z.map(&channel_z.input_target_position);*/

  output_a.begin(OUTPUT_A);
  output_b.begin(OUTPUT_B);
  output_c.begin(OUTPUT_C);
  output_d.begin(OUTPUT_D);

  enable_drivers();

  channel_a.begin(&output_a, SIGNAL_E);
  channel_a.set_ratio(1, 80);
  //channel_a.invert_output();

  channel_b.begin(&output_b, SIGNAL_E);
  channel_b.set_ratio(1, 80);
  channel_b.invert_output();

  channel_z.begin(&output_c, SIGNAL_E);
  channel_z.set_ratio(1, 400);
  channel_z.invert_output();


  channel_e.begin(&output_d, SIGNAL_E);
  channel_e.set_ratio(1, 93); // testing with an 8mm lead leadscrew

  encoder_1.begin(ENCODER_1);
  encoder_1.set_ratio(1, 2400); 
  encoder_1.output.map(&channel_e.input_target_position);
  encoder_1.invert();

  encoder_2.begin(ENCODER_2);
  encoder_2.set_ratio(10, 2400); 
  encoder_2.output.map(&channel_z.input_target_position);
  encoder_2.invert();


//extrusion knob
  analog_a1.set_floor(0, 25);
  analog_a1.set_ceiling(2, 1020);
  analog_a1.begin(IO_A1);
  
  z_gen.begin();
  z_gen.set_ratio(layerHeight); //1mm per rev
  z_gen.input_1.map(&channel_a.input_target_position);
  z_gen.input_2.map(&channel_b.input_target_position);
  z_gen.output.map(&channel_z.input_target_position);

  
  e_gen.begin();
  e_gen.input_1.map(&channel_a.input_target_position);
  e_gen.input_2.map(&channel_b.input_target_position);
  e_gen.output.map(&channel_e.input_target_position);


  
  dance_start();
}

LoopDelay overhead_delay;

void loop() {
  float64_t extrusionMultiplier = analog_a1.read();
  float64_t segmentLength = 1.0;
  extrusionRate = (4*layerHeight * extrusionMultiplier * nozzleDiameter * segmentLength) / (PI*nozzleDiameter*nozzleDiameter);
  e_gen.set_ratio(extrusionRate);

  dance_loop();
  overhead_delay.periodic_call(&report_overhead, 100);


}



void report_overhead(){
Serial.print("extrusion rate:");
Serial.print(extrusionRate);
Serial.print(", z-position:");
Serial.println(channel_z.input_target_position.read(ABSOLUTE));
   /*Serial.print(", input_x:");
  Serial.print(input_a.output_x.absolute_buffer);
 Serial.print(", input_y:");
  Serial.print(input_a.output_y.absolute_buffer);
   Serial.print(", input_z:");
  Serial.print(input_a.output_z.absolute_buffer);
   Serial.print(", input_e:");
  Serial.print(input_a.output_e.absolute_buffer);*/
 





  
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