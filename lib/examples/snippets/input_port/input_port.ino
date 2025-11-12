#define module_driver
#include "stepdance.hpp"

InputPort input_a; 
 
Channel channel_z;
KinematicsCoreXY axidraw_kinematics;

void setup() {
  // -- Configure and start the input port --
  input_a.begin(INPUT_A); // "INPUT_A" specifies the physical port on the PCB for the input.
  // -- Map InputPort x, y, and z Output BlockPorts to kinematics object BlockPorts --
  input_a.output_x.set_ratio(0.01, 1); //for this mapping, 1 step is 0.01mm
  input_a.output_x.map(&axidraw_kinematics.input_x);

  input_a.output_y.set_ratio(0.01, 1); //1 step is 0.01mm
  input_a.output_y.map(&axidraw_kinematics.input_y);

  input_a.output_z.set_ratio(0.01, 1); //1 step is 0.01mm
  input_a.output_z.map(&channel_z.input_target_position);
}

void loop(){
dance_loop();
}
