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

KinematicsHBot::KinematicsHBot(){};

void KinematicsHBot::begin(){
  begin(KINEMATICS_MODE_INCREMENTAL, nullptr, nullptr);
}

void KinematicsHBot::begin(uint8_t mode){
  begin(mode, nullptr, nullptr);
}

void KinematicsHBot::begin(Transmission *output_transmission_a, Transmission *output_transmission_b){
  begin(KINEMATICS_MODE_INCREMENTAL, output_transmission_a, output_transmission_b);
}

void KinematicsHBot::begin(uint8_t mode, Transmission *output_transmission_a, Transmission *output_transmission_b){
  this->mode = mode;
  if(output_transmission_a != nullptr){ //allows map() to be called before begin, if begin has no parameters.
    map(HBOT_OUTPUT_A, output_transmission_a);
  }
  if(output_transmission_b != nullptr){
    map(HBOT_OUTPUT_B, output_transmission_b);
  }
  input_transmission_x.begin(&input_position_x);
  input_transmission_x.get_function = std::bind(&KinematicsHBot::get_position_x, this);
  input_transmission_y.begin(&input_position_y);
  input_transmission_y.get_function = std::bind(&KinematicsHBot::get_position_y, this);
  reset();
  register_plugin();
}

void KinematicsHBot::reset(){
  input_position_x = 0;
  input_position_y = 0;
}

void KinematicsHBot::map(uint8_t output_index, Transmission* target_transmission){
  output_transmissions[output_index] = target_transmission;
}

void KinematicsHBot::run(){
  float64_t output_position_a = input_position_x + input_position_y;
  float64_t output_position_b = input_position_x - input_position_y;
  if(mode == KINEMATICS_MODE_INCREMENTAL){
    if(output_transmissions[HBOT_OUTPUT_A] != nullptr){
      output_transmissions[HBOT_OUTPUT_A]->increment(output_position_a);
    }
    if(output_transmissions[HBOT_OUTPUT_B] != nullptr){
      output_transmissions[HBOT_OUTPUT_B]->increment(output_position_b);
    }
    input_position_x = 0.0;
    input_position_y = 0.0;  
  }else if (mode == KINEMATICS_MODE_ABSOLUTE){
    if(output_transmissions[HBOT_OUTPUT_A] != nullptr){
      output_transmissions[HBOT_OUTPUT_A]->set(output_position_a);
    }
    if(output_transmissions[HBOT_OUTPUT_B] != nullptr){    
      output_transmissions[HBOT_OUTPUT_B]->set(output_position_b);
    }
  }
}

DecimalPosition KinematicsHBot::get_position_x(){
  float64_t position_a = output_transmissions[HBOT_OUTPUT_A]->get();
  float64_t position_b = output_transmissions[HBOT_OUTPUT_B]->get();
  return (position_a + position_b)/2;
}

DecimalPosition KinematicsHBot::get_position_y(){
  float64_t position_a = output_transmissions[HBOT_OUTPUT_A]->get();
  float64_t position_b = output_transmissions[HBOT_OUTPUT_B]->get();
  return (position_a - position_b)/2;
}

KinematicsPolarToCartesian::KinematicsPolarToCartesian(){};

void KinematicsPolarToCartesian::begin(){
  begin(KINEMATICS_MODE_INCREMENTAL, nullptr, nullptr);
}

void KinematicsPolarToCartesian::begin(uint8_t mode){
  begin(mode, nullptr, nullptr);
}

void KinematicsPolarToCartesian::begin(Transmission *output_transmission_x, Transmission *output_transmission_y){
  begin(KINEMATICS_MODE_INCREMENTAL, output_transmission_x, output_transmission_y);
}

void KinematicsPolarToCartesian::begin(uint8_t mode, Transmission *output_transmission_x, Transmission *output_transmission_y){
  this->mode = mode;
  if(output_transmission_x != nullptr){ //allows map() to be called before begin, if begin has no parameters.
    map(POLAR_OUTPUT_X, output_transmission_x);
  }
  if(output_transmission_y != nullptr){
    map(POLAR_OUTPUT_Y, output_transmission_y);
  }
  input_transmission_r.begin(&input_position_r);
  // input_transmission_r.get_function = std::bind(&KinematicsPolarToCartesian::get_position_r, this);
  input_transmission_t.begin(&input_position_t);
  // input_transmission_t.get_function = std::bind(&KinematicsPolarToCartesian::get_position_t, this);
  reset();
  register_plugin();
}

void KinematicsPolarToCartesian::reset(){
  input_position_r = 0;
  input_position_t = 0;
}

void KinematicsPolarToCartesian::map(uint8_t output_index, Transmission* target_transmission){
  output_transmissions[output_index] = target_transmission;
}

void KinematicsPolarToCartesian::run(){
  if(mode == KINEMATICS_MODE_INCREMENTAL){ 
    // Get current X and Y positions
    float64_t current_x = output_transmissions[POLAR_OUTPUT_X]->get();
    float64_t current_y = output_transmissions[POLAR_OUTPUT_Y]->get();

    // Convert to R and T
    float64_t current_r = std::sqrt(std::pow(current_x, 2) + std::pow(current_y, 2));
    float64_t current_t = atan2(current_y, current_x);

    // Increment based on inputs
    current_r += input_position_r;
    current_t += input_position_t;

    // Reduce current_t to range of 2 PI; I believe this may speed up sin and cos functions...
    current_t = std::remainder(current_t, TWO_PI);

    float64_t target_x = current_r * std::cos(current_t);
    float64_t target_y = current_r * std::sin(current_t);

    float64_t delta_x = target_x - current_x;
    float64_t delta_y = target_y - current_y;
    output_transmissions[POLAR_OUTPUT_X]->increment(delta_x);
    output_transmissions[POLAR_OUTPUT_Y]->increment(delta_y);

    input_position_r = 0.0;
    input_position_t = 0.0;
  
  }else if (mode == KINEMATICS_MODE_ABSOLUTE){
    //todo
  }
}

DecimalPosition KinematicsPolarToCartesian::get_position_r(){
  //todo
  return 0;
}

DecimalPosition KinematicsPolarToCartesian::get_position_t(){
  //todo
  return 0;
}
