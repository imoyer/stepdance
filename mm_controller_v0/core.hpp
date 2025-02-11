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
#define CORE_FRAME_FREQ_HZ 1000000 / CORE_FRAME_PERIOD_US //framerate in Hz. This is 25k
#define MAX_NUM_FRAME_FUNCTIONS 10 //maximum number of functions that can be called on the frame interrupt

void add_function_to_frame(frame_function_pointer target_function);
void stepdance_start();
void stepdance_metrics_reset(); //resets the CPU usage metrics
float stepdance_get_cpu_usage(); //returns a value from 0-1 indicating the maximum CPU usage.

static volatile float stepdance_max_cpu_usage = 0; //stores a running count of the maximum CPU usage, in the range 0-1;
static volatile uint32_t stepdance_interrupt_entry_cycle_count = 0; //stores the entry value of ARM_DWT_CYCCNT 
#endif