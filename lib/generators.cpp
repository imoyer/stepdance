#include <cmath>
#include <stdint.h>
#include "arm_math.h"
#include "generators.hpp"
#include "rpc.hpp"
/*
Generators Module of the StepDance Control System

This module contains an assortment of motion stream generators

[More Details to be Added]

A part of the Mixing Metaphors Project
(c) 2025 Ilan Moyer, Jennifer Jacobs, Devon Frost, Emilie Yu

*/

ThresholdGenerator::ThresholdGenerator(){};

void ThresholdGenerator::begin(){
  input.begin(&input_position);
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
  Serial.print("lower_threshold: ");
  Serial.print(_lower_threshold);
  Serial.print(", upper_threshold: ");
  Serial.print(_upper_threshold);
  Serial.print(", input_position: ");
  Serial.print(input.read(ABSOLUTE));
}

void ThresholdGenerator::setLowerCallback(void (*callback_function)()){
  callback_on_lower_threshold = callback_function;
}

void ThresholdGenerator::setUpperCallback(void (*callback_function)()){
  callback_on_upper_threshold = callback_function;
}

void ThresholdGenerator::setUpperThreshold(float64_t upper_threshold, bool clamp_to_upper){
  upper_set = true;
  _upper_threshold = upper_threshold;
  clamp_upper = clamp_to_upper;
}

void ThresholdGenerator::setLowerThreshold(float64_t lower_threshold, bool clamp_to_lower){
  lower_set = true;
  _lower_threshold = lower_threshold;
  clamp_lower = clamp_to_lower;
} 

void ThresholdGenerator::clearUpperThreshold(){
  upper_set = false;
  clamp_upper = false;
}

void ThresholdGenerator::clearLowerThreshold(){
  lower_set = false;
  clamp_lower = false;
}

void ThresholdGenerator::run(){
  input.pull();
  input.update();

  float64_t current_value = input.read(ABSOLUTE);
  if(upper_set){
    if(current_value >= _upper_threshold){
      if(callback_on_upper_threshold != nullptr){
        callback_on_upper_threshold();
      }
      if(clamp_upper){
        current_value = _upper_threshold;
      }
    }
  }
  if(lower_set){
    if(current_value <= _lower_threshold){
      if(callback_on_lower_threshold != nullptr){
        callback_on_lower_threshold();
      }
      if(clamp_lower){
        current_value = _lower_threshold;
      }
    }
  }
  output.set(current_value,ABSOLUTE);
  output.push();
}

void ThresholdGenerator::enroll(RPC *rpc, const String& instance_name){
  rpc->enroll(instance_name, "enable", *this, &ThresholdGenerator::enable);
  rpc->enroll(instance_name, "disable", *this, &ThresholdGenerator::disable);
  input.enroll(rpc, instance_name + ".input");
  output.enroll(rpc, instance_name + ".output");
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
    delta_angle_rad = frequency * CORE_FRAME_PERIOD_S;
  }
  else{
   delta_angle_rad = frequency * input.incremental_buffer;
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

void WaveGenerator1D::enroll(RPC *rpc, const String& instance_name){
  rpc->enroll(instance_name, "enable", *this, &WaveGenerator1D::enable);
  rpc->enroll(instance_name, "disable", *this, &WaveGenerator1D::disable);
  rpc->enroll(instance_name, "setNoInput", *this, &WaveGenerator1D::setNoInput);
  input.enroll(rpc, instance_name + ".input");
  output.enroll(rpc, instance_name + ".output");
  rpc->enroll(instance_name + ".amplitude", amplitude);
  rpc->enroll(instance_name + ".phase", phase);
  rpc->enroll(instance_name + ".frequency", frequency);
}


WaveGenerator2D::WaveGenerator2D(){};

void WaveGenerator2D::begin(){
  // input.begin(&input_position);
  input_frequency.begin(&input_frequency_value);
  input_theta.begin(&input_theta_value);
  output_x.begin(&output_x_position);
  output_y.begin(&output_y_position);

  register_plugin();
}

void WaveGenerator2D::setNoInput(){
  no_input = true;
}

void WaveGenerator2D::run(){

  input_frequency.pull();
  input_frequency.update();

  input_theta.pull();
  input_theta.update();

  input_t.pull();
  input_t.update();

  float64_t frequency = input_frequency.absolute_buffer;

  // float64_t current_angle_rad;
  if(no_input){
    float64_t delta_angle_rad = frequency * CORE_FRAME_PERIOD_S;
    current_angle_rad += delta_angle_rad;
  }
  else{
    // We multiply by PI so that we ensure a whole number of half-waves
    // This is necessary to gracefully end back on the motion segment again
    current_angle_rad = 3.1415 * frequency * input_t.absolute_buffer;
  }
  // float64_t delta_angle_rad = frequency * CORE_FRAME_PERIOD_S;

  if(current_angle_rad > (2*PI)){ //let's keep the angle values small
     current_angle_rad -= 2*PI;
  }

  // float64_t u = amplitude * std::cos(current_angle_rad);
  float64_t v = amplitude * std::sin(current_angle_rad);

  // This would create circles
  // float64_t delta_x = u * std::cos(input_theta.absolute_buffer) - v * std::sin(input_theta.absolute_buffer);
  // float64_t delta_y = u * std::sin(input_theta.absolute_buffer) + v * std::cos(input_theta.absolute_buffer);

  // This creates waves along the specified input direction
  float64_t delta_x = - v * std::sin(input_theta.absolute_buffer);
  float64_t delta_y = v * std::cos(input_theta.absolute_buffer);

  output_x.set(delta_x,ABSOLUTE);
  output_x.push();
  output_y.set(delta_y,ABSOLUTE);
  output_y.push();
}

void WaveGenerator2D::debugPrint() {
  Serial.print("current angle: ");
  Serial.print(current_angle_rad);
  Serial.print(" current u: ");
  Serial.print(amplitude * std::cos(current_angle_rad));
  Serial.print(" input_theta: ");
  Serial.print(input_theta.read(ABSOLUTE));
  Serial.print(",");
  Serial.print(" amplitude: ");
  Serial.print(amplitude);
  Serial.print(", freq: ");
  // Serial.print(frequency);
  Serial.print(input_frequency.read(ABSOLUTE));
  Serial.print(", output read: ");
  Serial.print(output_x.read(ABSOLUTE));
  Serial.print(",");
  Serial.println(output_y.read(ABSOLUTE));
}

void WaveGenerator2D::enroll(RPC *rpc, const String& instance_name){
  rpc->enroll(instance_name + ".amplitude", amplitude);
  // rpc->enroll(instance_name + ".frequency", frequency);
}


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

void CircleGenerator::enroll(RPC *rpc, const String& instance_name){
  // rpc->enroll(instance_name, "enable", *this, &CircleGenerator::enable);
  // rpc->enroll(instance_name, "disable", *this, &CircleGenerator::disable);
  rpc->enroll(instance_name, "setNoInput", *this, &CircleGenerator::setNoInput);
  input.enroll(rpc, instance_name + ".input");
  output_x.enroll(rpc, instance_name + ".output_x");
  output_y.enroll(rpc, instance_name + ".output_y");
  rpc->enroll(instance_name + ".radius", radius);
  rpc->enroll(instance_name + ".rotational_speed_rev_per_sec", rotational_speed_rev_per_sec);
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

void VelocityGenerator::enroll(RPC *rpc, const String& instance_name){
  output.enroll(rpc, instance_name + ".output");
  rpc->enroll(instance_name + ".speed_units_per_sec", speed_units_per_sec);
}

// -- POSITION GENERATOR --
PositionGenerator::PositionGenerator(){};

void PositionGenerator::begin(){
  output.begin(&current_position, BLOCKPORT_OUTPUT); //configure input transmission as interface to target_position
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
  switch(mode){
    case INCREMENTAL:
      target_position += distance_or_position;
      break;
    case ABSOLUTE:
      target_position = distance_or_position;
      break;
    case GLOBAL:
      output.pull_deep();
      target_position = distance_or_position;
      break;
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

void PositionGenerator::enroll(RPC *rpc, const String& instance_name){
  rpc->enroll(instance_name, "set_speed", *this, &PositionGenerator::set_speed);
  rpc->enroll(instance_name, "go", *this, static_cast<void(PositionGenerator::*)(float64_t, uint8_t, ControlParameter)>(&PositionGenerator::go));
  output.enroll(rpc, instance_name + ".output");
  rpc->enroll(instance_name + ".speed_units_per_sec", speed_units_per_sec);
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

void PathLengthGenerator2D::set_ratio_for_circle(ControlParameter circle_radius, ControlParameter output_per_revolution){
  // For a circle with radius r, one complete revolution traces a circumference of 2πr
  // To move output_per_revolution distance for each full circle, the ratio should be:
  // ratio = output_per_revolution / (2 * π * circle_radius)
  float64_t circumference = 2.0 * PI * circle_radius;
  this->ratio = output_per_revolution / circumference;
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

void PathLengthGenerator2D::enroll(RPC *rpc, const String& instance_name){
  rpc->enroll(instance_name, "set_ratio", *this, static_cast<void(PathLengthGenerator2D::*)(ControlParameter, ControlParameter)>(&PathLengthGenerator2D::set_ratio));
  rpc->enroll(instance_name, "set_ratio_for_circle", *this, &PathLengthGenerator2D::set_ratio_for_circle);
  input_1.enroll(rpc, instance_name + ".input_1");
  input_2.enroll(rpc, instance_name + ".input_2");
  output.enroll(rpc, instance_name + ".output");
  rpc->enroll(instance_name + ".ratio", ratio);
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

void PathLengthGenerator3D::enroll(RPC *rpc, const String& instance_name){
  rpc->enroll(instance_name, "set_ratio", *this, static_cast<void(PathLengthGenerator3D::*)(ControlParameter, ControlParameter)>(&PathLengthGenerator3D::set_ratio));
  input_1.enroll(rpc, instance_name + ".input_1");
  input_2.enroll(rpc, instance_name + ".input_2");
  input_3.enroll(rpc, instance_name + ".input_3");
  output.enroll(rpc, instance_name + ".output");
  rpc->enroll(instance_name + ".ratio", ratio);
}
