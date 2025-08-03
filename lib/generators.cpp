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

WaveGenerator1D::WaveGenerator1D(){};

void WaveGenerator1D::begin(){
  input.begin(&input_position);
  output.begin(&output_position);
  register_plugin();
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
   Serial.print(", previous_pos: ");
  Serial.print(previousTime);
  Serial.print(", output read: ");
  Serial.println(output.read(ABSOLUTE));

}

void WaveGenerator1D::run(){
  input.pull();
  input.update();
  
  //float64_t time = CORE_FRAME_PERIOD_S

  float64_t delta_angle_rad = rotational_speed_rev_per_sec * input.incremental_buffer;
  current_angle_rad += delta_angle_rad;

  if(current_angle_rad > (2*PI)){ //let's keep the angle values small
     current_angle_rad -= 2*PI;
  }
  float64_t delta_x = amplitude * delta_angle_rad * std::cos(current_angle_rad);

  delta = delta_x;

  output.set(delta,INCREMENTAL);
  output.push();
}





CircleGenerator::CircleGenerator(){};

void CircleGenerator::begin(){
  output_x.begin(&output_x_position);
  output_y.begin(&output_y_position);
  register_plugin();
}

void CircleGenerator::debugPrint(){
  Serial.print("Core Frame Period: ");
  Serial.print(CORE_FRAME_PERIOD_S,6);
  Serial.print(", rotational_speed_rev_per_sec: ");
  Serial.print(rotational_speed_rev_per_sec,6);
  Serial.print(", current_angle_rad: ");
  Serial.print(current_angle_rad,6);
    Serial.print(", radius: ");
  Serial.print(radius,6);
  Serial.print(", current_radius: ");
  Serial.print(current_radius,6);
  Serial.print(", radial_delta_mm: ");
  Serial.print(radial_delta_mm,6);
  Serial.print(", max_radial_change: ");
  Serial.println( max_radial_change,6);
}

void CircleGenerator::run(){
  float64_t time = CORE_FRAME_PERIOD_S;

  //calculate new angle
  float64_t angle_delta_rad = rotational_speed_rev_per_sec * 2 * PI * time; //change in angle this frame
  current_angle_rad += angle_delta_rad;
 
  if(current_angle_rad > (2*PI)){ //let's keep the angle values small
    current_angle_rad -= 2*PI;
  }

 
  // pre-calculate sin and cos of current angle
  float64_t sin_angle = sin(current_angle_rad);
  float64_t cos_angle = cos(current_angle_rad);

  // begin tracking delta_x and delta_y
  DecimalPosition delta_x = 0;
  DecimalPosition delta_y = 0;

  //calculate changes in radius
  radial_delta_mm = radius - current_radius;
  max_radial_change = max_radial_speed_mm_per_sec * CORE_FRAME_PERIOD_S; //maximum allowable change in radius


  if(radial_delta_mm > max_radial_change){
    radial_delta_mm = max_radial_change;
  }else if(radial_delta_mm < -max_radial_change){
    radial_delta_mm = -max_radial_change;
  }

  delta_x += radial_delta_mm * cos_angle;
  delta_y += radial_delta_mm * sin_angle;
  current_radius += radial_delta_mm;

  //calculate change in position due to rotation
  delta_x += current_radius * angle_delta_rad * sin_angle;
  delta_y += current_radius * angle_delta_rad * cos_angle;

 //Serial.print("delta_x, delta_y");
  //Serial.print(delta_x);
  //Serial.print(" ,");
  //Serial.println(delta_y);

  //add to target transmissions
  //if(x_output_transmission != nullptr){
    //x_output_transmission->increment(delta_x);
  //}
  //if(y_output_transmission != nullptr){
    //y_output_transmission->increment(delta_y);
  //}
  output_x.set(delta_x);
  output_x.push();

  output_y.set(delta_y);
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