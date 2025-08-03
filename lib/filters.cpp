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

void ScalingFilter1D::begin(){
  input.begin(&input_position);
  output.begin(&output_position);
  register_plugin();
}

void ScalingFilter1D::set_ratio(ControlParameter ratio){
  this->ratio = ratio;
}

void ScalingFilter1D::run(){
  input.pull();
  input.update();

  output.set(input_position * ratio);
  output.push();
}