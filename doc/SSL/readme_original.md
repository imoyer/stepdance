## Stepdance Software Library User Manual

Stepdance is a motion control operating system that scaffolds a wide range of modular software components. These can do things like:
 - real-time control of motors from encoders and potentiometers 
 - emulate standard motion controllers to interpret pre-planned motion formats like g-code and AxiDraw
 - record and replay motion streams
 - apply filters such as scaling and rotation
 - generate a host of motion signals including transiting to a position, constant velocity, sinusoidal position, etc.
 - convert between coordinate systems, e.g. polar and cartesian
 - mix signal sources in real-time

Most of the stepdance framework is interrupt-driven and operates in the background, by taking advantage of the relatively massive computing power of the Teensy 4/4.1 (based on the iMXRT1062).

 In this user manual, we describe the overall framework, the currently available elements, and how to connect everything together.

### Overview

### OutputPorts

OutputPorts are modules that convert internal step commands into a frame of pulse signals on the physical output port on the **machine controller module** or **basic module**. Output ports contain step and direction signal flags for each of the six output signal types (X, Y, Z, E, R, and T).

Sample code for intializing an output port:

```arduino
OutputPort output_a; 

void setup() {
  // -- Configure and start the output port --
  output_a.begin(OUTPUT_A); // "OUTPUT_A" specifies the physical port on the PCB for the output.
}
```

### Channels
Channels are modules that store the machine's positional state. A Channel directly interfaces with an OutputPort by mapping to a particular signal flag (e.g. "X").

 Channels contain a target position variable that can be modified by upstream components such as motion generators. 
 
 They also contain a variable representing their current position. Each interrupt frame, the channel increments or decrements the current position when needed to drive |target-current| < 0.5, and correspondingly sets step and direction flags on its mapped OutputPort. Any number of channels can be instantiated, often corresponding to axes of the machine machine. Basic Modules may contain multiple channels that map to distinct flags on the same OutputPort. Driver Modules typically have multiple channels that each map to a unique OutputPort, that then generate step/direction signals for a motor drive module.


Example: initializing a channel and mapping it to an output port
```arduino
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
```

### Inputs


#### Encoders
