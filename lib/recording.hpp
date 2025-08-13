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

class FourTrackRecorder : public Plugin{
  // Simultaneously records four motion streams to an SD card file.

  public:
    FourTrackRecorder();
    void begin();
    void start(const char *recording_name = "recording", uint8_t recording_length_min = 30); // start recording
    void pause(); // pause recording
    void stop(); // stop recording

    // BlockPorts
    BlockPort input_1;
    BlockPort input_2;
    BlockPort input_3;
    BlockPort input_4;
    
  private:
    // BlockPort input positions
    DecimalPosition input_1_position;
    DecimalPosition input_2_position;
    DecimalPosition input_3_position;
    DecimalPosition input_4_position;

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