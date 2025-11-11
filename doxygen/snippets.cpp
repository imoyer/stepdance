 // [OutputPort]
OutputPort output_a; 

void setup() {
  // -- Configure and start the output port --
  output_a.begin(OUTPUT_A); // "OUTPUT_A" specifies the physical port on the PCB for the output.
}
  // [OutputPort]

   // [Channel]
OutputPort output_a; 

Channel channel_a;

void setup() {
  // -- Configure and start the output port --
  output_a.begin(OUTPUT_A); // "OUTPUT_A" specifies the physical port on the PCB for the output.
    // -- Configure and start the channels --
  channel_a.begin(&output_a, SIGNAL_E); // Connects the channel to the "E" signal on "output_a".
                                        // We choose the "E" signal because it results in a step pulse of 7us,
                                        // which is more than long enough for the driver IC
  channel_a.set_ratio(40, 3200); // Sets the input/output transmission ratio for the channel.
}
  // [Channel]

  // [InputPort]
InputPort input_a; 
 
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
  // [InputPort]

    // [BlockPort]
InputPort inputA;
inputA.begin(INPUT_A); // Initialize InputPort A
inputA.output_x.map(channelX.input_target_position); // Map SIGNAL_X to Channel X's input target position
     // [BlockPort] 