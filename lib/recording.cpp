#include "arm_math.h"
#include <stdint.h>
/*
Recording Module of the StepDance Control System

This module contains facilities for recording and playback of motion streams.

[More Details to be Added]

A part of the Mixing Metaphors Project
(c) 2025 Ilan Moyer, Jennifer Jacobs, Devon Frost

*/

#include "recording.hpp"

// ---- RECORDER TRACK ----
// Basic recording element used by recorder classes
RecorderTrack::RecorderTrack(){};
void RecorderTrack::begin(){
  this->input_target_position.begin(&target_position);
}

int8_t RecorderTrack::run(){
  input_target_position.pull();
  input_target_position.update();

  float64_t delta_position = target_position - current_position;
  if(delta_position > 0.5 ){
    current_position ++;
    return 1;
  }else if(delta_position < -0.5){
    current_position --;
    return -1;
  }else{
    return 0;
  }
}

// ---- FOUR TRACK RECORDER ----

FourTrackRecorder::FourTrackRecorder(){};

void FourTrackRecorder::begin(){
  for(uint8_t index = 0; index < NUM_CHANNELS; index ++){
    recorder_tracks[index].begin();
  }
  set_resolution(this->resolution_units_per_step); //either uses the default value, or a user-set value.
  register_plugin(PLUGIN_FRAME_POST_CHANNEL); //runs last
}

void FourTrackRecorder::set_resolution(float input_units, float per_steps){
  this->resolution_units_per_step = input_units / per_steps;
  for(uint8_t index = 0; index < NUM_CHANNELS; index ++){
    recorder_tracks[index].input_target_position.set_ratio(this->resolution_units_per_step);
  }
}

void FourTrackRecorder::start(const char *recording_name, uint8_t recording_length_min){
  // Starts the recorder.
};

void FourTrackRecorder::stop(){

};

void FourTrackRecorder::pause(){

};