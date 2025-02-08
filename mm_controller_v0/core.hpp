/*
Core Module of the StepDance Control System

This contains core system functions such as the frame interrupt timer.

A part of the Mixing Metaphors Project
(c) 2025 Ilan Moyer, Jennifer Jacobs, Devon Frost
*/
#ifndef core_h //prevent importing twice
#define core_h

typedef volatile int32_t IntegerPosition; //used to store positions across the system
typedef void (*frame_function_pointer)(); //defines function pointers that can be called at each frame

#define CORE_FRAME_PERIOD_US 40 //microseconds. This yields a max output step rate of 25k steps/sec.
#define MAX_NUM_FRAME_FUNCTIONS 10 //maximum number of functions that can be called on the frame interrupt

void add_function_to_frame(frame_function_pointer target_function);
void stepdance_start();

#endif