#include <stdint.h>
/*
Filters Module of the StepDance Control System

This module contains an assortment of motion filters

[More Details to be Added]

A part of the Mixing Metaphors Project
(c) 2025 Ilan Moyer, Jennifer Jacobs, Devon Frost

*/
#include "filters.hpp"

// -- 1D Scaling Filter --
ScalingFilter1D::ScalingFilter1D(){};

void ScalingFilter1D::begin(uint8_t mode){
  input.begin(&input_position);
  output.begin(&output_position);
  this->mode = mode;
  register_plugin();
}

void ScalingFilter1D::set_ratio(ControlParameter ratio){
  this->ratio = ratio;
}

void ScalingFilter1D::run(){
  input.pull();
  input.update();

  if(mode == INCREMENTAL){
    output.set(input.incremental_buffer * ratio, INCREMENTAL);
  }else{
    output.set(input_position * ratio);
  }

  output.push();
}

// -- 2D Scaling Filter --
ScalingFilter2D::ScalingFilter2D(){};

void ScalingFilter2D::begin(uint8_t mode){
  input_1.begin(&input_1_position);
  input_2.begin(&input_2_position);
  output_1.begin(&output_1_position);
  output_2.begin(&output_2_position);
  this->mode = mode;
  register_plugin();
}

void ScalingFilter2D::set_ratio(ControlParameter ratio){
  this->ratio = ratio;
}

void ScalingFilter2D::run(){
  input_1.pull();
  input_2.pull();
  input_1.update();
  input_2.update();

  if(mode == INCREMENTAL){
    output_1.set(input_1.incremental_buffer * ratio, INCREMENTAL);
    output_2.set(input_2.incremental_buffer * ratio, INCREMENTAL);
  }else{ //ABSOLUTE
    output_1.set(input_1_position * ratio);
    output_2.set(input_2_position * ratio);
  }

  output_1.push();
  output_2.push();
}