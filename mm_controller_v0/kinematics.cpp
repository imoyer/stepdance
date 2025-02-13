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
  this->output_transmission_a = output_transmission_a;
  this->output_transmission_b = output_transmission_b;
  input_transmission_x.begin(&input_position_x);
  input_transmission_y.begin(&input_position_y);
  reset();
  register_plugin();
}

void KinematicsHBot::reset(){
  input_position_x = 0;
  input_position_y = 0;
}

void KinematicsHBot::run(){
  float64_t output_position_a = input_position_x + input_position_y;
  float64_t output_position_b = input_position_x - input_position_y;
  if(mode == KINEMATICS_MODE_INCREMENTAL){
    output_transmission_a->increment(output_position_a);
    output_transmission_b->increment(output_position_b);
    input_position_x = 0.0;
    input_position_y = 0.0;  
  }else if (mode == KINEMATICS_MODE_ABSOLUTE){
    output_transmission_a->set(output_position_a);
    output_transmission_b->set(output_position_b); 
  }
}