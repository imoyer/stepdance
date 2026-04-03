/*
Encoder Test

Displays encoder readings on ENC1, ENC2, and optionally extended encoders (e.g. using input ports as encoder inputs)

A part of the Mixing Metaphors Project

// (c) 2025 Ilan Moyer, Jennifer Jacobs, Devon Frost
*/

#define encoder_a_enable //comment out to disable reading encoder_a (which hijacks input_a)
                          //NOTE: Currently, only supported on the Basic Module, and Teensy Pins 6 and 7 need
                          // to be shorted together. Future PCB revs will change the pinouts so this works out of the box.

// #define module_driver   // tells compiler we're using the Stepdance Driver Module PCB
#define module_basic   // tells compiler we're using the Stepdance Basic Module PCB
                        // This configures pin assignments for the Teensy 4.1

#include "stepdance.hpp"  // Import the stepdance library

Encoder encoder_1;
Encoder encoder_2;
#ifdef encoder_a_enable
  Encoder encoder_a;
#endif //encoder_a_enable


void setup() {
  encoder_1.begin(ENCODER_1);
  encoder_2.begin(ENCODER_2);
  #ifdef encoder_a_enable
    encoder_a.begin(ENCODER_A);
  #endif //encoder_a_enable

  Serial.begin(115200);

  dance_start();
}


void loop() {
  report_encoder_values();
  Serial.println(" ");
  delay(500);
}

void report_encoder_values(){
  Serial.print("ENC1: ");
  Serial.println(encoder_1.read());
  Serial.print("ENC2: ");
  Serial.println(encoder_2.read());
  #ifdef encoder_a_enable
    Serial.print("ENCA: ");
    Serial.println(encoder_a.read());
  #endif //encoder_a_enable   
}