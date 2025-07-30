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
  output.begin(&target_position);
  register_plugin();
}

void VelocityGenerator::run(){
  output.set(speed_units_per_sec * CORE_FRAME_PERIOD_S, INCREMENTAL);
  output.push();
}

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