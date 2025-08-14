#include <stdint.h>
/*
Recording Module of the StepDance Control System

This module contains facilities for recording and playback of motion streams.

[More Details to be Added]

A part of the Mixing Metaphors Project
(c) 2025 Ilan Moyer, Jennifer Jacobs, Devon Frost

*/

#include "core.hpp"

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
    void start(const char *recording_name = "recording", uint8_t recording_length_min = 30); // start recording
    void pause(); // pause recording
    void stop(); // stop recording
    void set_resolution(float input_units, float per_steps = 1.0);

    static const uint8_t NUM_CHANNELS = 4;
    RecorderTrack recorder_tracks[NUM_CHANNELS];
    BlockPort& input_1 = recorder_tracks[0].input_target_position;
    BlockPort& input_2 = recorder_tracks[1].input_target_position;
    BlockPort& input_3 = recorder_tracks[2].input_target_position;
    BlockPort& input_4 = recorder_tracks[3].input_target_position;
    
  private:
    float32_t resolution_units_per_step = STANDARD_RATIO_MM;

  protected:
    void run();
};

class FourTrackPlayer : public Plugin{

  public:
    FourTrackPlayer();
    void begin();
    void start(const char *recording_name = "recording");
    void pause();
    void stop();

    //BlockPorts
    BlockPort output_1;
    BlockPort output_2;
    BlockPort output_3;
    BlockPort output_4;
  
  private:
    // BlockPort output positions
    DecimalPosition output_1_position;
    DecimalPosition output_2_position;
    DecimalPosition output_3_position;
    DecimalPosition output_4_position;
  
  protected:
    void run();
};
#endif //recording_h