#include "arm_math.h"
#include "generators.hpp"
/*
Generators Module of the StepDance Control System

This module contains an assortment of motion stream generators

[More Details to be Added]

A part of the Mixing Metaphors Project
(c) 2025 Ilan Moyer, Jennifer Jacobs, Devon Frost

*/

CircleGenerator::CircleGenerator(){};

void CircleGenerator::begin(){
  register_plugin();
}

void CircleGenerator::map(Transmission* x_target_transmission, Transmission* y_target_transmission){
  x_output_transmission = x_target_transmission;
  y_output_transmission = y_target_transmission;
}

void CircleGenerator::run(){
  
  //calculate new angle
  float64_t angle_delta_rad = rotational_speed_rev_per_sec * 2 * PI * CORE_FRAME_PERIOD_S; //change in angle this frame
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
  float32_t radial_delta_mm = radius - current_radius;
  float32_t max_radial_change = max_radial_speed_mm_per_sec * CORE_FRAME_PERIOD_S; //maximum allowable change in radius

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

  //add to target transmissions
  if(x_output_transmission != nullptr){
    x_output_transmission->increment(delta_x);
  }
  if(y_output_transmission != nullptr){
    y_output_transmission->increment(delta_y);
  }
}

VelocityGenerator::VelocityGenerator(){};

void VelocityGenerator::begin(){
  register_plugin();
}

void VelocityGenerator::map(Transmission* target_transmission){
  output_transmission = target_transmission;
}

void VelocityGenerator::run(){
  float64_t delta = speed_units_per_sec * CORE_FRAME_PERIOD_S;
  if(output_transmission != nullptr){
    output_transmission->increment(delta);
  }
}

PositionGenerator::PositionGenerator(){};

void PositionGenerator::begin(){
  input_transmission.begin(&target_position); //configure input transmission as interface to target_position
  register_plugin();
}

void PositionGenerator::map(Transmission* target_transmission){
  output_transmission = target_transmission;
}

void PositionGenerator::go_incremental(DecimalPosition distance){
  go_incremental(distance, max_speed_units_per_sec);
}

void PositionGenerator::go_incremental(DecimalPosition distance, ControlParameter max_speed){
  max_speed_units_per_sec = max_speed;
  input_transmission.increment(distance);
}

void PositionGenerator::go_absolute(DecimalPosition target_position){
  go_absolute(target_position, max_speed_units_per_sec);
}

void PositionGenerator::go_absolute(DecimalPosition target_position, ControlParameter max_speed){
  max_speed_units_per_sec = max_speed;
  input_transmission.set(target_position);
}

void PositionGenerator::set_max_speed(ControlParameter max_speed){
  max_speed_units_per_sec = max_speed;
}

void PositionGenerator::run(){
  float64_t delta_position = target_position - current_position;
  float64_t max_distance_this_frame;

  // clamp delta to maximum distance imposed by velocity limit
  if(delta_position>=0){
    max_distance_this_frame = max_speed_units_per_sec * CORE_FRAME_PERIOD_S;
    if(delta_position > max_distance_this_frame){
      delta_position = max_distance_this_frame;
    }
  }else{
    max_distance_this_frame = -max_speed_units_per_sec * CORE_FRAME_PERIOD_S;
    if(delta_position < max_distance_this_frame){
      delta_position = max_distance_this_frame;
    }
  }

  if(output_transmission != nullptr){
    output_transmission->increment(delta_position);
  }

  current_position += delta_position;
}