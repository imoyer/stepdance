#include <stdint.h>
/*
Recording Module of the StepDance Control System

This module contains facilities for recording and playback of motion streams.

[More Details to be Added]

A part of the Mixing Metaphors Project
(c) 2025 Ilan Moyer, Jennifer Jacobs, Devon Frost

*/

#include "recording.hpp"

// ---- FOUR TRACK RECORDER ----

FourTrackRecorder::FourTrackRecorder(){};

void FourTrackRecorder::begin(){
  input_1.begin(&input_1_position);
  input_2.begin(&input_2_position);
  input_3.begin(&input_3_position);
  input_4.begin(&input_4_position);
  register_plugin(PLUGIN_FRAME_POST_CHANNEL); //runs last
}

void FourTrackRecorder::start(const char *recording_name, uint8_t recording_length_min){
  // Starts the recorder.
};

void FourTrackRecorder::stop(){

};

void FourTrackRecorder::pause(){

};