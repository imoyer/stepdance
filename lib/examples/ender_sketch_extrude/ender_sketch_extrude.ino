#include <SD.h>

#define module_driver

#include "stepdance.hpp"

// Pantograph XY input plugged in input_a
InputPort input_a;

// Ender XYZE plugged into output ports
OutputPort output_a;
OutputPort output_b;
OutputPort output_c;
OutputPort output_d;

// Corresponding channels for the Ender
Channel channel_a;
Channel channel_b;
Channel channel_z;
Channel channel_e;

// Knobs input
Encoder encoder_1; 
Encoder encoder_2; 

// Slider input
AnalogInput analog_a1; //extrusion rate controller

// -- Define Input Button --
Button button_d1; // start/stop recording
Button button_d2; // start/stop replay


// VelocityGenerator velocity_gen;

PositionGenerator start_position_gen_x; // To bridge gap between end/start of layer path
PositionGenerator start_position_gen_y; // To bridge gap between end/start of layer path

PathLengthGenerator2D z_gen; //generates z signal

PathLengthGenerator2D e_gen; //generates extruder signal

// -- Recorder --
FourTrackRecorder recorder;
FourTrackPlayer player;

// Should continuously replay?
bool replay_until_stopped = false;
bool waiting_to_start_play = false;

// Start position of layer
float64_t start_pos_x;
float64_t start_pos_y;


// TODO: need to update
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

  // E: are all these ratios meaningful?

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
  // z_gen.input_1.map(&channel_a.input_target_position);
  // z_gen.input_2.map(&channel_b.input_target_position);
  // z_gen.output.map(&channel_z.input_target_position);

  
  e_gen.begin();
  e_gen.input_1.map(&channel_a.input_target_position);
  e_gen.input_2.map(&channel_b.input_target_position);
  e_gen.output.map(&channel_e.input_target_position);

  // -- Configure the Position Generator
  // TODO: do I need one for X and one for Y?
  start_position_gen_x.output.map(&channel_a.input_target_position);
  start_position_gen_x.begin();

  start_position_gen_y.output.map(&channel_b.input_target_position);
  start_position_gen_y.begin();


  button_d1.begin(IO_D1, INPUT_PULLDOWN);
  button_d1.set_mode(BUTTON_MODE_TOGGLE);
  button_d1.set_callback_on_press(&start_recording);
  button_d1.set_callback_on_release(&stop_recording);

  button_d2.begin(IO_D2, INPUT_PULLDOWN);
  button_d2.set_mode(BUTTON_MODE_STANDARD);
  button_d2.set_callback_on_press(&playback_control);


  recorder.input_1.map(&channel_a.input_target_position);
  recorder.input_2.map(&channel_b.input_target_position);
  recorder.begin();

  player.output_1.map(&channel_a.input_target_position);
  player.output_2.map(&channel_b.input_target_position);
  player.begin();


  
  dance_start();
}

LoopDelay overhead_delay;

void loop() {
  float64_t extrusionMultiplier = analog_a1.read();
  float64_t segmentLength = 1.0;
  extrusionRate = (4*layerHeight * extrusionMultiplier * nozzleDiameter * segmentLength) / (PI*nozzleDiameter*nozzleDiameter);
  e_gen.set_ratio(extrusionRate);

  if (waiting_to_start_play) {
    Serial.println("Waiting for jog to finish to start play...");

    float64_t x_dist_to_start_pt = channel_a.input_target_position.read(ABSOLUTE) - start_pos_x;
    float64_t y_dist_to_start_pt = channel_b.input_target_position.read(ABSOLUTE) - start_pos_y;
    // Serial.print("Dist to start:");
    // Serial.print(x_dist_to_start_pt);
    // Serial.print(",");
    // Serial.println(y_dist_to_start_pt);
    // Serial.println("Dist to start: " + x_dist_to_start_pt + "," + y_dist_to_start_pt);
    if (x_dist_to_start_pt*x_dist_to_start_pt < 0.0001 && y_dist_to_start_pt*y_dist_to_start_pt < 0.0001) {

      // Serial.print("Current position: ");
      // Serial.print(" x: ");
      // Serial.print(current_x);
      // Serial.print(" y: ");
      // Serial.println(current_y);

      Serial.println("Starting playback for real");
      start_playback();
    }
  }

  if (replay_until_stopped && !player.playback_active) {
    // Toggle replay back on
    // Maybe: first launch position generator to reach start point again
    // float64_t pos_offset_x = start_pos_x - channel_a.input_target_position.read(ABSOLUTE);
    // float64_t pos_offset_y = start_pos_y - channel_b.input_target_position.read(ABSOLUTE);
    // // start_position_gen_x.go(pos_offset_x , INCREMENTAL);
    // start_position_gen_x.go(start_pos_x , ABSOLUTE, 50);
    // start_position_gen_y.go(start_pos_y , ABSOLUTE, 50);
    // // start_position_gen_y.go(pos_offset_y , INCREMENTAL);
    // Serial.print("Replay stopped, go back to start pos! Relative motion: ");

    // Serial.print(pos_offset_x);
    // Serial.print(",");
    // Serial.println(pos_offset_y);

    // replay_until_stopped = false; // debug (stop loop)

  }

  dance_loop();
  overhead_delay.periodic_call(&report_overhead, 100);


}

void start_recording(){
  recorder.start("eazao_layer_sketch");
  Serial.println("STARTED RECORDING");

  // Recording first point value
  start_pos_x = channel_a.input_target_position.read(ABSOLUTE); 
  start_pos_y = channel_b.input_target_position.read(ABSOLUTE); 
  Serial.print("Start position: ");
  Serial.print(" x: ");
  Serial.print(start_pos_x);
  Serial.print(" y: ");
  Serial.println(start_pos_y);

}

void stop_recording(){
  recorder.stop();
  Serial.println("STOPPED RECORDING");
}

void playback_control(){
  if(player.playback_active){
    stop_playback();
    replay_until_stopped = false;
  }else{
    initiate_playback();
    replay_until_stopped = true;
  }
}

void initiate_playback(){

  // Debug
  float64_t current_x = channel_a.input_target_position.read(ABSOLUTE); 
  float64_t current_y = channel_b.input_target_position.read(ABSOLUTE); 
  Serial.print("Current position: ");
  Serial.print(" x: ");
  Serial.print(current_x);
  Serial.print(" y: ");
  Serial.println(current_y);

  // First go to start point
  start_position_gen_x.go(start_pos_x - current_x , INCREMENTAL, 50); // works
  // start_position_gen_x.go(start_pos_x , ABSOLUTE, 50); // does not work because we actually have to set a relative target
  start_position_gen_y.go(start_pos_y - current_y , INCREMENTAL, 50);

  waiting_to_start_play = true;


  // player.start("eazao_layer_sketch");
  // Serial.println("STARTED PLAYING");
  // int duration_s = player.max_num_playback_samples / (CORE_FRAME_FREQ_HZ);
  // Serial.print("DURATION: ");
  // Serial.print(static_cast<int>(duration_s / 60));
  // Serial.print("m:");
  // Serial.print(duration_s % 60);
  // Serial.println("s");
}

void start_playback(){
  waiting_to_start_play = false;
  player.start("eazao_layer_sketch");
  Serial.println("STARTED PLAYING");
  int duration_s = player.max_num_playback_samples / (CORE_FRAME_FREQ_HZ);
  Serial.print("DURATION: ");
  Serial.print(static_cast<int>(duration_s / 60));
  Serial.print("m:");
  Serial.print(duration_s % 60);
  Serial.println("s");
}

void stop_playback(){
  player.stop();
}

void report_overhead(){

  // if (waiting_to_start_play) {
  //   float64_t x_dist_to_start_pt = channel_a.input_target_position.read(ABSOLUTE) - start_pos_x;
  //   float64_t y_dist_to_start_pt = channel_b.input_target_position.read(ABSOLUTE) - start_pos_y;
  //   // float64_t x_dist_to_start_pt = start_position_gen_x.output.read(ABSOLUTE);
  //   // float64_t y_dist_to_start_pt = start_position_gen_y.output.read(ABSOLUTE);

  //   Serial.print("Dist to start:");
  //   Serial.print(x_dist_to_start_pt);
  //   Serial.print(",");
  //   Serial.println(y_dist_to_start_pt);

  // }

  if(recorder.recorder_active){
    Serial.print("RECORDING... ");
    int duration_s = recorder.current_sample_index / (CORE_FRAME_FREQ_HZ);
    Serial.print(static_cast<int>(duration_s / 60));
    Serial.print("m:");
    Serial.print(duration_s % 60);
    Serial.println("s");
  }
  if(player.playback_active){
    Serial.print("PLAYING... ");
    int duration_s = player.current_sample_index / (CORE_FRAME_FREQ_HZ);
    Serial.print(static_cast<int>(duration_s / 60));
    Serial.print("m:");
    Serial.print(duration_s % 60);
    Serial.println("s");
  }  
  // Serial.println(channel_z.target_position, 4);
  // Serial.println(stepdance_get_cpu_usage(), 4);
  // Serial.print("extrusion rate:");
  // Serial.print(extrusionRate);
  // Serial.print(", z-position:");
  // Serial.println(channel_z.input_target_position.read(ABSOLUTE));
}
