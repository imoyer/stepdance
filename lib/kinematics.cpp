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
#include "rpc.hpp"

KinematicsCoreXY::KinematicsCoreXY(){};

void KinematicsCoreXY::begin(){
  input_x.begin(&position_x, BLOCKPORT_INPUT, this); //only add pointer to this plugin to input blockports
  input_y.begin(&position_y, BLOCKPORT_INPUT, this);
  output_a.begin(&position_a, BLOCKPORT_OUTPUT);
  output_b.begin(&position_b, BLOCKPORT_OUTPUT);
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

void KinematicsCoreXY::enroll(RPC *rpc, const String& instance_name){
  input_x.enroll(rpc, instance_name + ".input_x");
  input_y.enroll(rpc, instance_name + ".input_y");
  output_a.enroll(rpc, instance_name + ".output_a");
  output_b.enroll(rpc, instance_name + ".output_b");
}

void KinematicsCoreXY::push_deep(){
  output_a.push_deep(position_x + position_y);
  output_b.push_deep(position_x - position_y);
}

void KinematicsCoreXY::pull_deep(){
  output_a.pull_deep();
  output_b.pull_deep();
  input_x.reset(0.5*(position_a + position_b), true);
  input_y.reset(0.5*(position_a - position_b), true);
}


KinematicsPolarToCartesian::KinematicsPolarToCartesian(){};

void KinematicsPolarToCartesian::begin(float64_t fixed_radius){
  input_radius.begin(&position_r);
  input_angle.begin(&position_a);
  output_x.begin(&position_x);
  output_y.begin(&position_y);

  if(fixed_radius > 0){
    input_radius.reset(fixed_radius);
  }

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

void KinematicsPolarToCartesian::enroll(RPC *rpc, const String& instance_name){
  input_angle.enroll(rpc, instance_name + ".input_angle");
  input_radius.enroll(rpc, instance_name + ".input_radius");
  output_x.enroll(rpc, instance_name + ".output_x");
  output_y.enroll(rpc, instance_name + ".output_y");
}


KinematicsFiveBarForward::KinematicsFiveBarForward(){};

void KinematicsFiveBarForward::begin(float32_t s, float32_t l1, float32_t l2, float32_t l3, float32_t l4, float32_t l5, float32_t a){
  this->S = s;
  this->L1 = l1;
  this->L2 = l2;
  this->L3 = l3;
  this->L4 = l4;
  this->L5 = l5;
  this->A6 = a;
  this->Xa = s/2;
  this->Xb = -s/2;
  this->Ya = 0;
  this->Yb = 0;

  input_r.begin(&position_r);
  input_l.begin(&position_l);
  output_x.begin(&position_x);
  output_y.begin(&position_y);
  register_plugin();
}

void KinematicsFiveBarForward::run(){
  //update input angles
  input_r.pull(); // in most cases, user should set up mapping for ABSOLUTE. Consider forcing ABSOLUTE here to avoid user errors.
  input_l.pull();
  input_r.update();
  input_l.update();

  float32_t A1 = static_cast<float32_t>(position_r); //right encoder angle
  float32_t A2 = static_cast<float32_t>(position_l); //left encoder angle

  float32_t Xc = Xa + L1 * std::cos(A1);
  float32_t Yc = Ya - L1 * std::sin(A1);
  float32_t Xd = Xb + L2 * std::cos(A2);
  float32_t Yd = Yb - L2 * std::sin(A2);

  float32_t L6 = Yd - Yc;
  float32_t L7 = Xc - Xd;

  float32_t A3 = std::atan2(L6, L7);
  float32_t L8 = std::sqrt((L6 * L6) + (L7 * L7));

  float32_t A4 = std::acos((L8*L8 + L3*L3 - L4*L4)/(2*L8*L3));

  float32_t A5 = PI/2.0 + A3 - A4;

  float32_t Xe = Xc - L3 * sin(A5);
  float32_t Ye = Yc - L3 * cos(A5);

  float32_t A7 = A6 - PI/2.0 + A5;
  float32_t Xt = Xe + L5 * std::cos(A7);
  float32_t Yt = Ye - L5 * std::sin(A7);

  output_x.set(static_cast<float64_t>(Xt));
  output_y.set(-static_cast<float64_t>(Yt));

  output_x.push();
  output_y.push();
}

void KinematicsFiveBarForward::enroll(RPC *rpc, const String& instance_name){
  input_r.enroll(rpc, instance_name + ".input_r");
  input_l.enroll(rpc, instance_name + ".input_l");
  output_x.enroll(rpc, instance_name + ".output_x");
  output_y.enroll(rpc, instance_name + ".output_y");
}