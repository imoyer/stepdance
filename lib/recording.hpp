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

/**
 * @brief RecorderTrack handles recording of a single motion stream.
 * @ingroup recording
 * @details The RecorderTrack class records a single motion stream by tracking the input target position and determining when steps occur based on the configured resolution. It is used internally by the FourTrackRecorder class to manage multiple recording channels. You should not have to intialize it directly.
 */
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

/**
 * @brief FourTrackRecorder records four motion streams to an SD card file.
 * @ingroup recording
 * @details The FourTrackRecorder class allows simultaneous recording of four motion streams to an SD card file. Each track can be mapped to a BlockPort for input. The recorder supports starting, pausing, resuming, and stopping recordings, as well as setting the resolution for recording. Recorded data is stored in a binary format on the SD card for efficient storage and retrieval. It is typically used in conjunction with the FourTrackPlayer class for playback of recorded motion streams.
 * Here's an example of how to instantiate and configure a FourTrackRecorder:
 * @ref axidraw_recorder/axidraw_recorder.ino "AxiDraw Recorder Example".
 */
class FourTrackRecorder : public Plugin{
  // Simultaneously records four motion streams to an SD card file.

  public:
    FourTrackRecorder();
    /** 
     * @brief Initializes the FourTrackRecorder. Must be called before using the recorder.
     */
    void begin();
    /** 
     * @brief Starts a new recording with the specified name and optional length.
     * @param recording_name Name of the recording file. This should be the same name you provide to the FourTrackPlayer for playback.
     * @param recording_length_min Optional length of the recording in minutes. Default is 30 minutes.
     */
    void start(const char *recording_name, uint8_t recording_length_min = 30); // start recording
    /** 
     * @brief Pauses the current recording.
     */
    void pause(); // pause recording
    /** 
     * @brief Resumes a paused recording.
     */
    void resume(); //continues recording
    /** 
     * @brief Stops the current recording.
     */
    void stop(); // stop recording
    /** 
     * @brief Sets the resolution for recording in world units per step.
     * @param input_units World units corresponding to one step.
     * @param per_steps Number of steps. Default is 1.0.
     */
    void set_resolution(float input_units, float per_steps = 1.0);
    /**
     * \cond
     */
    void enroll(RPC *rpc, const String& instance_name);
  

    static const uint8_t NUM_CHANNELS = 4;
    RecorderTrack recorder_tracks[NUM_CHANNELS];
    /** \endcond */
    /**
     * @brief Input BlockPort for the first recording channel. Map upstream components to this port.
     */
    BlockPort& input_1 = recorder_tracks[0].input_target_position;
    /**
     * @brief Input BlockPort for the second recording channel. Map upstream components to this port.
     */
    BlockPort& input_2 = recorder_tracks[1].input_target_position;
    /**
     * @brief Input BlockPort for the third recording channel. Map upstream components to this port.
     */
    BlockPort& input_3 = recorder_tracks[2].input_target_position;
    /**
     * @brief Input BlockPort for the fourth recording channel. Map upstream components to this port.
     */
    BlockPort& input_4 = recorder_tracks[3].input_target_position;
    /** 
     * @brief Current sample index in the recording.
     */
    volatile long current_sample_index = 0;
    /** 
     * @brief Maximum number of samples in the recording.
     */
    long max_num_samples = 0;
    /** 
     * @brief Indicates whether the recorder is currently active.
     */
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

/**
 * @brief FourTrackPlayer plays back four motion streams from an SD card file.
 * @ingroup recording
 * @details The FourTrackPlayer class allows simultaneous playback of four motion streams from a recorded SD card file. Each track can be mapped to a BlockPort for output. The player supports starting, pausing, resuming, and stopping playback, as well as setting the resolution for playback. It reads recorded data stored in a binary format on the SD card for efficient retrieval. It is typically used in conjunction with the FourTrackRecorder class for recording motion streams.
 * Here's an example of how to instantiate and configure a FourTrackPlayer:
 * @ref axidraw_recorder/axidraw_recorder.ino "AxiDraw Recorder Example".
 */ 
class FourTrackPlayer : public Plugin{

  public:
    FourTrackPlayer();
    /**
     * @brief Initializes the FourTrackPlayer. Must be called before using the player.
     */
    void begin();
    /**
     * @brief Starts playback of a recording with the specified name.
     * @param recording_name Name of the recording file to play back. This should match the name used during recording.
     */
    void start(const char *recording_name = "recording");
    /**
     * @brief Pauses the current playback.
     */
    void pause();
    /**
     * @brief Resumes a paused playback.
     */
    void resume();
    /**
     * @brief Stops the current playback.
     */
    void stop();
    /**
     * @brief Sets the resolution for playback in world units per step.
     * @param output_units World units corresponding to one step.
     * @param per_steps Number of steps. Default is 1.0.
     */
    void set_resolution(float output_units, float per_steps = 1.0);
    /**
     * \cond
     */
    void enroll(RPC *rpc, const String& instance_name);
    
    //BlockPorts
    static const uint8_t NUM_CHANNELS = 4;
    BlockPort output_BlockPorts[NUM_CHANNELS];
/** \endcond */ 
  /** 
     * @brief Output BlockPort for the first playback channel. Map downstream components to this port.
     */
    BlockPort& output_1 = output_BlockPorts[0];
    /**
     * @brief Output BlockPort for the second playback channel. Map downstream components to this port.
     */
    BlockPort& output_2 = output_BlockPorts[1];
    /**
     * @brief Output BlockPort for the third playback channel. Map downstream components to this port.
     */
    BlockPort& output_3 = output_BlockPorts[2];
    /**
     * @brief Output BlockPort for the fourth playback channel. Map downstream components to this port.
     */ 
    BlockPort& output_4 = output_BlockPorts[3];
    /** 
     * @brief Current sample index in the playback.
     */
    volatile long current_sample_index = 0;
    /** 
     * @brief Maximum number of samples in the playback.
     */
    long max_num_playback_samples = 0;
    /** 
     * @brief Indicates whether the player is currently active.
     */
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