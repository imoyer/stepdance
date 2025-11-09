#include <stdint.h>
/*
Recording Module of the StepDance Control System

This module contains facilities for recording and playback of motion streams.

[More Details to be Added]

A part of the Mixing Metaphors Project
(c) 2025 Ilan Moyer, Jennifer Jacobs, Devon Frost

*/

#include "core.hpp"
#include <SD.h>
#include <string>

#ifndef recording_h //prevent importing twice
#define recording_h

class RecorderTrack {
  // a single track used for recording
  public:
    RecorderTrack();
    void begin();
    int8_t run(); //returns 1 or -1 if step is taken, 0 if not.
    BlockPort input_target_position;

  private:
    DecimalPosition target_position = 0;
    DecimalPosition current_position = 0;
};

class FourTrackRecorder : public Plugin{
  // Simultaneously records four motion streams to an SD card file.

  public:
    FourTrackRecorder();
    void begin();
    void start(const char *recording_name, uint8_t recording_length_min = 30); // start recording
    void pause(); // pause recording
    void resume(); //continues recording
    void stop(); // stop recording
    void set_resolution(float input_units, float per_steps = 1.0);
    void enroll(RPC *rpc, const String& instance_name);

    static const uint8_t NUM_CHANNELS = 4;
    RecorderTrack recorder_tracks[NUM_CHANNELS];
    BlockPort& input_1 = recorder_tracks[0].input_target_position;
    BlockPort& input_2 = recorder_tracks[1].input_target_position;
    BlockPort& input_3 = recorder_tracks[2].input_target_position;
    BlockPort& input_4 = recorder_tracks[3].input_target_position;

    volatile long current_sample_index = 0;
    long max_num_samples = 0;
    volatile bool recorder_active = false;

  private:
    float32_t resolution_units_per_step = STANDARD_RATIO_MM;
    volatile bool in_recording = false; //true when paused
    char recording_filename[30];
    char length_filename[30];
    FsFile active_recording_file;

  protected:
    void run();
};

class FourTrackPlayer : public Plugin{

  public:
    FourTrackPlayer();
    void begin();
    void start(const char *recording_name = "recording");
    void pause();
    void resume();
    void stop();
    void set_resolution(float output_units, float per_steps = 1.0);
    void enroll(RPC *rpc, const String& instance_name);
    
    //BlockPorts
    static const uint8_t NUM_CHANNELS = 4;
    BlockPort output_BlockPorts[NUM_CHANNELS];

    BlockPort& output_1 = output_BlockPorts[0];
    BlockPort& output_2 = output_BlockPorts[1];
    BlockPort& output_3 = output_BlockPorts[2];
    BlockPort& output_4 = output_BlockPorts[3];
  
    volatile long current_sample_index = 0;
    long max_num_playback_samples = 0;
    volatile bool playback_active = false;

  private:
    // BlockPort output positions
    DecimalPosition output_positions[NUM_CHANNELS];
    float32_t resolution_units_per_step = STANDARD_RATIO_MM;
    volatile bool in_playback = false; //true when paused
    char recording_filename[30];
    char length_filename[30];
    FsFile active_playback_file;
  
  protected:
    void run();
};

// --- SD CARD UTILITIES ---
void initialize_sd_card();

#endif //recording_h