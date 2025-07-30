#include "wiring.h"
#include <cmath>
#include "arm_math.h"
/*
Kinematics Module of the StepDance Control System

This module provides a variety of mechanism kinematics to go from one motion space (e.g. XY) to another (e.g. AB or RT) via e.g. an h-bot or polar mechanism, etc.

[More Details to be Added]

A part of the Mixing Metaphors Project
(c) 2025 Ilan Moyer, Jennifer Jacobs, Devon Frost
*/

#include "kinematics.hpp"

KinematicsCoreXY::KinematicsCoreXY(){};

void KinematicsCoreXY::begin(){
  input_x.begin(&position_x);
  input_y.begin(&position_y);
  output_a.begin(&position_a);
  output_b.begin(&position_b);
  register_plugin();
  // input_transmission_y.get_function = std::bind(&KinematicsCoreXY::get_position_y, this);
}

void KinematicsCoreXY::run(){
  input_x.pull();
  input_y.pull();
  input_x.update();
  input_y.update();
  
  output_a.set(position_x + position_y);
  output_b.set(position_x - position_y);
  output_a.push();
  output_b.push();
}

// DecimalPosition KinematicsCoreXY::get_position_x(){
//   float64_t position_a = output_transmissions[COREXY_OUTPUT_A]->get();
//   float64_t position_b = output_transmissions[COREXY_OUTPUT_B]->get();
//   return (position_a + position_b)/2;
// }

// DecimalPosition KinematicsCoreXY::get_position_y(){
//   float64_t position_a = output_transmissions[COREXY_OUTPUT_A]->get();
//   float64_t position_b = output_transmissions[COREXY_OUTPUT_B]->get();
//   return (position_a - position_b)/2;
// }

KinematicsPolarToCartesian::KinematicsPolarToCartesian(){};

void KinematicsPolarToCartesian::begin(){
  input_radius.begin(&position_r);
  input_angle.begin(&position_a);
  output_x.begin(&position_x);
  output_y.begin(&position_y);
  register_plugin();
}

void KinematicsPolarToCartesian::reset(){
}

void KinematicsPolarToCartesian::run(){
  //update radius and angle positions
  input_angle.pull();
  input_radius.pull();
  input_angle.update();
  input_radius.update();

  float64_t angle_reduced = std::remainder(position_a, TWO_PI); // Reduce current_t to range of 2 PI; I believe this may speed up sin and cos functions...

  output_x.set(position_r * std::cos(angle_reduced));
  output_y.set(position_r * std::sin(angle_reduced));

  output_x.push();
  output_y.push();
}
