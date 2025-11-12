#define module_driver
#include "stepdance.hpp"

Button button_d1;
Button button_d2;

void setup() {
  button_d1.begin(IO_D1); //initialize button on physical input port IO_D1
  button_d1.set_callback_on_release(&onButtonD1Release); //set callback function for button D1 release event
  button_d2.begin(IO_D2); //initialize button on physical input port IO_D2
  button_d2.set_mode(BUTTON_MODE_TOGGLE); //set button D2 to toggle mode
  button_d2.set_callback_on_toggle(&onButtonD2Toggle); //set callback function for button D2 press event
  
  dance_start();
}

void onButtonD1Release() {
  // Code to execute when button D1 is released
  Serial.println("Button D1 Released");
}

void onButtonD2Toggle() {
  // Code to execute when button D2 is toggled
  Serial.println("Button D2 Toggled");
}

void loop(){
dance_loop();
}
