#include <cmath>
#include <stdint.h>
#include "arm_math.h"
#include "generators.hpp"
/*
Generators Module of the StepDance Control System

This module contains an assortment of motion stream generators

[More Details to be Added]

A part of the Mixing Metaphors Project
(c) 2025 Ilan Moyer, Jennifer Jacobs, Devon Frost

*/

ThresholdGenerator::ThresholdGenerator(){};

void ThresholdGenerator::begin(){
  input_a.begin(&input_a_position);
  input_b.begin(&input_b_position);
  output.begin(&output_position);
  register_plugin();
}

void ThresholdGenerator::enable(){
  output.enable();
}

void ThresholdGenerator::disable(){
  output.disable();
}

void ThresholdGenerator::debugPrint(){
  Serial.print("threshold: ");
  Serial.print(threshold);
  Serial.print(", current_value:");
  Serial.print(current_value);
}

void ThresholdGenerator::run(){
  input_a.pull();
  input_a.update();
  input_b.pull();
  input_b.update();

  current_value += input_a.read(INCREMENTAL);
  if(current_value >= threshold){
    current_value = 0;
    
  }

}


WaveGenerator1D::WaveGenerator1D(){};

void WaveGenerator1D::begin(){
  input.begin(&input_position);
  output.begin(&output_position);
  register_plugin();
}

void WaveGenerator1D::setNoInput(){
  no_input = true;
}

void WaveGenerator1D::debugPrint(){
  Serial.print("input_position: ");
  Serial.print(input.read(INCREMENTAL));
  Serial.print(",");
  Serial.print(input.read(ABSOLUTE));
  Serial.print(",");
  Serial.print(input_position);
  Serial.print("amplitude: ");
  Serial.print(amplitude);
  Serial.print(", delta_value: ");
  Serial.print(delta);
  Serial.print(", output read: ");
  Serial.println(output.read(ABSOLUTE));

}

void WaveGenerator1D::run(){
  input.pull();
  input.update();
  float64_t delta_angle_rad;
  if(no_input){
    delta_angle_rad = rotational_speed_rev_per_sec * CORE_FRAME_PERIOD_S;
  }
  else{
   delta_angle_rad = rotational_speed_rev_per_sec * input.incremental_buffer;
  }
  
  current_angle_rad += delta_angle_rad;

  if(current_angle_rad > (2*PI)){ //let's keep the angle values small
     current_angle_rad -= 2*PI;
  }
  float64_t delta_x = (amplitude+phase) * delta_angle_rad * std::cos(current_angle_rad);

  delta = delta_x;

  output.set(delta,INCREMENTAL);
  output.push();
}

void WaveGenerator1D::enable(){
  output.enable();
}

void WaveGenerator1D::disable(){
  output.disable();
}

/*WaveGenerator2D::WaveGenerator2D(){};

void WaveGenerator2D::begin(){
  input.begin(&input_position);
  output_x.begin(&output_x_position);
  output_y.begin(&output_y_position);

  register_plugin();
}

void WaveGenerator2D::debugPrint(){

}

void WaveGenerator2D::run(){
  input.pull();
  input.update();
  
  float64_t delta_angle_rad = rotational_speed_rev_per_sec * input.incremental_buffer;
  current_angle_rad += delta_angle_rad;

  if(current_angle_rad > (2*PI)){ //let's keep the angle values small
     current_angle_rad -= 2*PI;
  }
  float64_t delta_x = amplitude * delta_angle_rad * std::cos(current_angle_rad);
  float64_t delta_y = amplitude * delta_angle_rad * std::sin(current_angle_rad);

  output_x.set(delta_x,INCREMENTAL);
  output_x.push();
  output_y.set(delta_y,INCREMENTAL);
  output_y.push();
}*/


CircleGenerator::CircleGenerator(){};

void CircleGenerator::begin(){
  input.begin(&input_position);
  output_x.begin(&output_x_position);
  output_y.begin(&output_y_position);
  register_plugin();
}

void CircleGenerator::setNoInput(){
  no_input = true;
}

void CircleGenerator::debugPrint(){
}

void CircleGenerator::run(){
  input.pull();
  input.update();
  float64_t delta_angle_rad;
  if(no_input){
    delta_angle_rad = rotational_speed_rev_per_sec * CORE_FRAME_PERIOD_S;
  }
  else{
   delta_angle_rad = rotational_speed_rev_per_sec * input.incremental_buffer;
  }
   
  current_angle_rad += delta_angle_rad;

  if(current_angle_rad > (2*PI)){ //let's keep the angle values small
     current_angle_rad -= 2*PI;
  }
  float64_t delta_x = radius * delta_angle_rad * std::cos(current_angle_rad);
  float64_t delta_y = radius * delta_angle_rad * std::sin(current_angle_rad);

  output_x.set(delta_x,INCREMENTAL);
  output_x.push();
  output_y.set(delta_y,INCREMENTAL);
  output_y.push();
}

// -- VELOCITY GENERATOR --
VelocityGenerator::VelocityGenerator(){};

void VelocityGenerator::begin(){
  output.begin(&target_position);
  register_plugin();
}

void VelocityGenerator::run(){
  output.set(speed_units_per_sec * CORE_FRAME_PERIOD_S, INCREMENTAL);
  output.push();
}

// -- POSITION GENERATOR --
PositionGenerator::PositionGenerator(){};

void PositionGenerator::begin(){
  output.begin(&current_position); //configure input transmission as interface to target_position
  register_plugin();
}

void PositionGenerator::set_speed(ControlParameter speed){
  speed_units_per_sec = speed;
}

void PositionGenerator::go(float64_t distance_or_position, uint8_t mode){
  go(distance_or_position, mode, speed_units_per_sec);
}

void PositionGenerator::go(float64_t distance_or_position, uint8_t mode, ControlParameter speed){
  speed_units_per_sec = speed;
  if(mode == INCREMENTAL){
    target_position += distance_or_position;
  }else{ //ABSOLUTE
    target_position = distance_or_position;
  }
}


void PositionGenerator::run(){
  float64_t delta_position = target_position - current_position;
  float64_t max_distance_this_frame;

  // clamp delta to maximum distance imposed by velocity
  if(delta_position>=0){
    max_distance_this_frame = speed_units_per_sec * CORE_FRAME_PERIOD_S;
    if(delta_position > max_distance_this_frame){
      delta_position = max_distance_this_frame;
    }
  }else{
    max_distance_this_frame = -speed_units_per_sec * CORE_FRAME_PERIOD_S;
    if(delta_position < max_distance_this_frame){
      delta_position = max_distance_this_frame;
    }
  }

  output.set(delta_position, INCREMENTAL); // this updates the current position

  output.push();
}

// -- 2D Path Length Generator --
PathLengthGenerator2D::PathLengthGenerator2D(){};

void PathLengthGenerator2D::begin(){
  input_1.begin(&input_1_position);
  input_2.begin(&input_2_position);
  output.begin(&output_position);
  register_plugin();
}

void PathLengthGenerator2D::set_ratio(ControlParameter ratio){
  this->ratio = ratio;
}

void PathLengthGenerator2D::run(){
  input_1.pull();
  input_2.pull();
  input_1.update();
  input_2.update();

  // below was a bit tricky. pull() defaults to incremental, which reads an incremental change, which it then uses to increment input_1_position. But
  // input_1_position is cumulative. If we want to calculate based on changes to input_1, the most speed-efficient way is to read the incremental buffers
  // directly. Since we're in the block, let's consider this OK, rather than using a read() call.
  float64_t distance = std::sqrt((input_1.incremental_buffer * input_1.incremental_buffer) + (input_2.incremental_buffer * input_2.incremental_buffer));
  output.set(distance * ratio, INCREMENTAL);

  output.push();
}

// -- 3D Path Length Generator --
PathLengthGenerator3D::PathLengthGenerator3D(){};

void PathLengthGenerator3D::begin(){
  input_1.begin(&input_1_position);
  input_2.begin(&input_2_position);
  input_3.begin(&input_3_position);
  output.begin(&output_position);
  register_plugin();
}

void PathLengthGenerator3D::set_ratio(ControlParameter ratio){
  this->ratio = ratio;
}

void PathLengthGenerator3D::run(){
  input_1.pull();
  input_2.pull();
  input_3.pull();
  input_1.update();
  input_2.update();
  input_3.update();

  // below was a bit tricky. pull() defaults to incremental, which reads an incremental change, which it then uses to increment input_1_position. But
  // input_1_position is cumulative. If we want to calculate based on changes to input_1, the most speed-efficient way is to read the incremental buffers
  // directly. Since we're in the block, let's consider this OK, rather than using a read() call.
  float64_t distance = std::sqrt((input_1.incremental_buffer * input_1.incremental_buffer) + (input_2.incremental_buffer * input_2.incremental_buffer) + (input_3.incremental_buffer * input_3.incremental_buffer));
  output.set(distance * ratio, INCREMENTAL);

  output.push();
}

SerialControlTrack::SerialControlTrack(){};

void SerialControlTrack::begin() {
  output.begin(&output_position);
}


void SerialControlTrack::run() {

  // Check if we should move on to the next queue item?
  if (delta_to_current_target * delta_to_current_target < 1e-8 ) { // soft equality check
    // Empty that slot in buffer queue (it was dealt with)
    received_target_delta[current_read_idx_in_queue] = 0.0;
    // Move to read next slot
    current_read_idx_in_queue = (current_read_idx_in_queue + 1) % buffer_queue_size;
    delta_to_current_target = received_target_delta[current_read_idx_in_queue];
  }

  // Calculate the target step size:
  DecimalPosition target_frame_delta_size = received_target_delta[current_read_idx_in_queue] * CORE_FRAME_PERIOD_US / KILOHERTZ_PLUGIN_PERIOD_US;

  // Clamp to delta_to_current_target

  // Clamp to max velocity
  if (target_frame_delta_size > 0) {
    if (target_frame_delta_size > delta_to_current_target) {
      target_frame_delta_size = delta_to_current_target;
    }
    if (target_frame_delta_size > velocity_limit) {
      target_frame_delta_size = velocity_limit;
    }
  }
  else {
    if (target_frame_delta_size < delta_to_current_target) {
      target_frame_delta_size = delta_to_current_target;
    }
    if (target_frame_delta_size < -velocity_limit) {
      target_frame_delta_size = -velocity_limit;
    }
  }

  // Send

  output.set(target_frame_delta_size, INCREMENTAL);
  output.push();

  // Apply to the variable that's keeping track
  delta_to_current_target -= target_frame_delta_size;
}

void SerialControlTrack::loop(int input) {
  // Map to correct range
  DecimalPosition target_value = input * MAX_RANGE / 128; // input is an int in [-128, 127]

  if (input != 0){
    // populate buffer
    current_write_idx_in_queue = (current_write_idx_in_queue + 1) % buffer_queue_size;
    received_target_delta[current_write_idx_in_queue] = target_value;

    // start reading this value in buffer? Only if we are done reading from current value
    if (delta_to_current_target * delta_to_current_target < 1e-8 ){
      current_read_idx_in_queue = current_write_idx_in_queue;
      delta_to_current_target = received_target_delta[current_read_idx_in_queue];
    }

  }

  // send receive notification to server
  Serial.print("received:");
  Serial.print(input);
  Serial.print(", new received target:");
  Serial.print(received_target_delta[current_write_idx_in_queue]);
  Serial.print(", delta to current working target:");
  Serial.print(delta_to_current_target);
  Serial.print(", reading idx:");
  Serial.print(current_read_idx_in_queue);
  Serial.print(", writing idx:");
  Serial.print(current_write_idx_in_queue);
  Serial.print("\n");
}

SerialConnectionGenerator::SerialConnectionGenerator(){};

void SerialConnectionGenerator::begin(){
  // Serial.println("serial connection begin");
  // Register the plugin on the main loop and on the frame

  for (int i = 0; i < NUM_CHANNELS; i++){
    controlled_tracks[i].begin();
  }

  register_plugin(); // on frame
  register_plugin(PLUGIN_LOOP); // on loop


}

void SerialConnectionGenerator::run() {
  // This runs every frame
  // Interpolate values between the previous loop state and current loop state

    for (int i = 0; i < NUM_CHANNELS; i++){
      controlled_tracks[i].run();
    }
}

void SerialConnectionGenerator::loop() {

  // Read from Serial to set the target values for the current loop iteration
  // Receive input from server
  if (Serial.available()) {
    // incomingByte = Serial.read(); // read only one byte and then execute the rest of the loop code (prevent delays?)
    char serial_in_data[NUM_CHANNELS]; // one char per axis
    Serial.readBytes(serial_in_data, NUM_CHANNELS);

    for (int i = 0; i < NUM_CHANNELS; i++) {
      int raw_value = (signed char) serial_in_data[i];
      controlled_tracks[i].loop(raw_value);
    }

  }

  else {

  }
}