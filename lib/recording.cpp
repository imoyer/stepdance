#include <cstring>
#include "HardwareSerial.h"
#include <sys/_types.h>
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
  set_resolution(this->resolution_units_per_step); //either uses the default value, or a user-set value
  initialize_sd_card(); // initialize SD card
  register_plugin(PLUGIN_FRAME_POST_CHANNEL); //runs last
}

void FourTrackRecorder::set_resolution(float input_units, float per_steps){
  this->resolution_units_per_step = input_units / per_steps;
  for(uint8_t index = 0; index < NUM_CHANNELS; index ++){
    recorder_tracks[index].input_target_position.set_ratio(this->resolution_units_per_step);
  }
}

void FourTrackRecorder::start(const char* recording_name, uint8_t recording_length_min){
  if(in_recording){
    resume();
  }else{
    // store recording filename
    std::strcpy(this->recording_filename, recording_name);
    std::strcpy(this->length_filename, recording_name);
    std::strcat(this->recording_filename, ".bin");
    std::strcat(this->length_filename, "_len.bin");

    // open and pre-allocate recording file
    max_num_samples = static_cast<long>(recording_length_min) * 60 * CORE_FRAME_FREQ_HZ;
    active_recording_file = SD.sdfs.open(recording_filename, O_WRITE | O_CREAT);
    unsigned long recording_file_length = active_recording_file.fileSize();
    if (recording_file_length > 0) {
      active_recording_file.truncate(); //reduce file size to zero if it already has data
    }
    if (!active_recording_file.preAllocate(max_num_samples)){
      Serial1.println("Unable to pre-allocate space for recording file.");
    }

    // reset state and start recording
    current_sample_index = 0;
    recorder_active = true;
    in_recording = true;
  }
};

void FourTrackRecorder::stop(){
  if(in_recording){ //make sure user doesn't call when not recording.
    // stop recording
    recorder_active = false;
    in_recording = false;

    // close the active recording file
    active_recording_file.close();

    // write recording length
    FsFile record_length_file = SD.sdfs.open(length_filename, O_WRITE | O_CREAT);
    unsigned long record_length_file_length = record_length_file.fileSize(); 
    if (record_length_file_length > 0) {
      record_length_file.truncate(); //reduce file size to zero if it already has data
    }
    record_length_file.println(current_sample_index);
    record_length_file.close();
  }
};

void FourTrackRecorder::pause(){
  recorder_active = false;
};

void FourTrackRecorder::resume(){
  recorder_active = true;
}

void FourTrackRecorder::run(){
  if(recorder_active){ //we're actively recording, so write a sample
    
    // encode the sample byte
    uint8_t encoded_sample = 0;
    for(uint8_t index = 0; index < NUM_CHANNELS; index ++){
      int8_t step = recorder_tracks[index].run();
      if(step == 1){
        encoded_sample |= (0b11 << (index*2));
      }else if(step == -1){
        encoded_sample |= (0b10 << (index*2));
      }
    }

    // write the sample byte
    if(current_sample_index < max_num_samples){
      active_recording_file.write(encoded_sample);
      current_sample_index ++;
    }
  }
}

// --- FOUR TRACK PLAYER ---
FourTrackPlayer::FourTrackPlayer(){};

void FourTrackPlayer::begin(){
  for(uint8_t index = 0; index < NUM_CHANNELS; index ++){
    output_BlockPorts[index].begin(&output_positions[index]);
  }
  set_resolution(this->resolution_units_per_step); //either uses the default value, or a user-set value.
  initialize_sd_card(); // initialize SD card
  register_plugin(PLUGIN_INPUT_PORT); //run first, alongside input ports
}

void FourTrackPlayer::set_resolution(float output_units, float per_steps){
  this->resolution_units_per_step = output_units / per_steps;
  for(uint8_t index = 0; index < NUM_CHANNELS; index ++){
    output_BlockPorts[index].set_ratio(this->resolution_units_per_step);
  }
}

void FourTrackPlayer::start(const char* recording_name){
  if(in_playback){
    resume();
  }else{
    // store recording filename
    std::strcpy(this->recording_filename, recording_name);
    std::strcpy(this->length_filename, recording_name);
    std::strcat(this->recording_filename, ".bin");
    std::strcat(this->length_filename, "_len.bin");

    // // get playback length
    FsFile record_length_file = SD.sdfs.open(length_filename, O_READ); 
    String file_length_string = record_length_file.readStringUntil('\n');
    max_num_playback_samples = static_cast<long>(file_length_string.toInt());
    record_length_file.close();
    
    // // open recording
    active_playback_file = SD.sdfs.open(recording_filename, O_READ);
    current_sample_index = 0;
    in_playback = true;
    playback_active = true;
  }
};

void FourTrackPlayer::stop(){
  if(in_playback){ //make sure user doesn't call when not recording.
    // stop recording
    playback_active = false;
    in_playback = false;

    // close the active recording file
    active_playback_file.close();
  }
};

void FourTrackPlayer::pause(){
  playback_active = false;
};

void FourTrackPlayer::resume(){
  playback_active = true;
}

void FourTrackPlayer::run(){
  if(playback_active){ //we're actively playing, so read a sample
    if(current_sample_index < max_num_playback_samples){
      current_sample_index ++;
      uint8_t encoded_sample = active_playback_file.read();
      for(uint8_t index = 0; index < NUM_CHANNELS; index ++){
        float64_t delta = 0;
        if(encoded_sample & 0b10){ //taking a step
          if(encoded_sample & 0b01){ //forward
            delta = 1;
          }else{ //reverse
            delta = -1;
          }
        }
        output_BlockPorts[index].set(delta, INCREMENTAL);
        output_BlockPorts[index].push();
        encoded_sample >>= 2;
      }
    }else{
      stop();
    }
  }
}

// --- SD CARD UTILITIES ---
void initialize_sd_card(){
  bool ok;
  ok = SD.sdfs.begin(SdioConfig(FIFO_SDIO));
  if (!ok) {
    Serial1.println("initialization failed!");
    return;
  }
}