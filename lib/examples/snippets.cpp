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