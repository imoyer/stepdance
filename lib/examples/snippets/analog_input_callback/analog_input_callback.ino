#define module_driver
#include "stepdance.hpp"

AnalogInput analog_a1;



void setup() {
//analog input a1 
  analog_a1.begin(IO_A1); //initialize analog input on physical input port IO_A1
  analog_a1.set_callback(&analogInputCallback); //set the callback function to be called on new data
  
  dance_start();

}

void analogInputCallback() {
  float inputValue = analog_a1.read(); //read analog input a1 value during callback
  // Do something with inputValue, e.g., print it to the console
  Serial.println(inputValue);
}
 
void loop(){
dance_loop();
}
