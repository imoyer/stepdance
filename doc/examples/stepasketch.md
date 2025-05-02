## Overview
![](/doc/images/step-a-sketch_teaser.png)

This example uses two encoders to control a pen plotter (e.g. an AxiDraw) in realtime, a la etch-a-sketch. One knob is mapped to the X axis, and the other to the Y axis. You will learn how to:

- configure outputs ports, in this case to control stepper motor drivers
- create "channels" that each generate output stream components for individual axes of motion.
- use kinematic modules to convert between cartesian XY space and machine motor space. 
- read position values from encoders, and use them to drive output channels.

## Configuring the Driver Module

The axidraw has two stepper motors for XY positioning and a single servo motor for the pen height. The AxiDraw Inkscape plugin _and_ the Stepdance AxiDraw software module both expect that the motors are 200 steps/rev and driven at 1/16th microstepping. You'll want to configure the Stepdance Driver Module as follows:

- Install jumpers on both microstepping headers for the A and B output ports as shown below to set 1/16 microstepping. The jumpers should be oriented vertically. [More details are here](../readme.md#stepper-drivers-and-motor-interfaces)

## Wiring
![](/doc/images/step-a-sketch_wiring.png)

There are seven wiring connections to be made:

- Both AxiDraw stepper motors to Output Ports A and B. We've wired the left motor to A, and the right motor to B.
- The servo motor to Output Port C.
- Two encoders, one to each of the encoder input ports ENC1 and ENC2. [Details on wiring Taiss encoders are here](../taiss.md). Refer to the [board reference](../readme.md#encoders-and-analog--digital-inputs) for general info on wiring encoders.
- 5VDC to the Teensy 4.1, delivered over a micro-usb cable. This powers the logic for all of the electronics, except for the hobby servo driver, which generates its own 5V supply. Initially, power this via your computer while programming the Teensy with firmware. Then, you can provide power from a USB charger etc.
- 12VDC to the 5.5mm OD / 2.1mm ID barrel plug.

### Wiring Stepper Motors
![](/doc/images/stepper_wiring.png)

The Stepdance driver board supports _two phase_ stepper motors. These phases "A" and "B" (represented by blue and red inductor symbols in the diagram above) each has two wires coming off the motor, for four leads total. In order for the motor to spin, it is essential that each phase's two wires enter the connector at adjacent pins.

Sometimes you can rely on the stepper motor mfg for the color code, but sometimes you can't, or you don't know the manufacturer. A simple test is to put a multimeter in ohmeter mode across any two of the four motor wires. If you read a low resistance (typically < 20 ohms), these wires belong to the same phase and should be adjacent to each other on the connector. If you read an infinite resistance, try a different combination.

Flipping the two wires within a phase, or flipping the two phases, will change the direction that the motor spins; this can be corrected in software.

Below illustrates the color codes used in the AxiDraw V3.

![](/doc/images/axidraw_wiring.png)

### Wiring the Servo Motor
![](/doc/images/servo_wiring.png)

Note that the servo is wired into the same 4-pin connector and header as the stepper motors.