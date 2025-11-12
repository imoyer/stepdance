#define module_driver
#include "stepdance.hpp"

AnalogInput analog_a1;
AnalogInput analog_a2;

VelocityGenerator velocity_gen;


void setup() {
//analog input a1 
  analog_a1.set_floor(0, 25); //set floor to 0 output at ADC values of 25 and below. 
  analog_a1.set_ceiling(2.75, 1020); //set ceiling to 2.75 output at ADC values of 1020 and above.
  analog_a1.map(&velocity_gen.speed_units_per_sec); //map analog output to VelocityGenerator speed ControlParameter
  analog_a1.begin(IO_A1); //initialize analog input on physical input port IO_A1

//analog input a2
  analog_a2.set_floor(0, 25);
  analog_a2.set_ceiling(10, 1020);
  analog_a2.begin(IO_A2);//initialize analog input on physical input port IO_A2
  
  dance_start();

  Serial.begin(115200);

}

void loop() {
  float extrusionMultiplier = analog_a2.read(); //read analog input a2 value during loop and store it in variable
  Serial.println(extrusionMultiplier); //print the read value to the serial console
  dance_loop(); 
}
