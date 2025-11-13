\page ssl Stepdance Software Library Documentation

## Introduction

Stepdance is a motion control operating system that scaffolds a wide range of modular software components. These can do things like:
 - real-time control of motors from encoders and potentiometers 
 - emulate standard motion controllers to interpret pre-planned motion formats like g-code and AxiDraw
 - record and replay motion streams
 - apply filters such as scaling and rotation
 - generate a host of motion signals including transiting to a position, constant velocity, sinusoidal position, etc.
 - convert between coordinate systems, e.g. polar and cartesian
 - mix signal sources in real-time

Most of the stepdance framework is interrupt-driven and operates in the background, by taking advantage of the relatively massive computing power of the Teensy 4/4.1 (based on the iMXRT1062).

In this documentation we describe the overall framework, the currently available elements, and how to connect everything together.

## Overview

The Stepdance library provides a flexible architecture for motion control through several key components:

- **BlockPorts**: Unified interface for connecting components and routing data between them (see BlockPort and @ref core)
- **Channels**: Position state management and step/direction signal generation (see Channel and @ref channels)
- **Input/Output Ports**: Physical hardware interface for receiving and transmitting motion signals (see InputPort, OutputPort, and @ref io)
- **Generators**: Motion pattern and trajectory generation (see @ref generators)
- **Kinematics**: Coordinate system transformations (Cartesian, polar, CoreXY, etc.) (see @ref kinematics)
- **Filters**: Signal smoothing and noise reduction (see @ref filters)
- **Interfaces**: External protocol support (G-code, EiBotBoard, etc.) (see GCodeInterface, Eibotboard, and @ref interfaces)
- **Recording**: Motion recording and playback functionality (see FourTrackRecorder, FourTrackPlayer, and @ref recording)
- **Inputs**: Physical input device handling (see AnalogInput, Button, Encoder, and @ref inputs)

## Getting Started

For a complete guide to getting started with Stepdance, see the @ref getting_started page.

To get started with Stepdance programming:

1. **Understand BlockPorts**: BlockPorts are the fundamental communication mechanism between components. See the BlockPort class and @ref core module for details.

2. **Configure Output Ports**: Set up your physical output ports to send step/direction signals. See the OutputPort class and @ref io module for configuration.

3. **Create Channels**: Channels drive motors to target positions. See the Channel class and @ref channels module for setup and usage.

4. **Add Motion Sources**: Connect input ports, generators, or other motion sources to your channels using BlockPort mapping. See @ref generators and @ref inputs for available sources.

## Examples

For complete working examples, see the @ref examples page which includes examples organized by category:
- Core examples (analog input, encoders, I/O tests)
- Motion control examples (pantograph, wheelbot, harmonograph)
- Plotting examples (AxiDraw interface, G-code)
- 3D printer examples (clay printer, extrusion)
- And more...

Example code snippets are also provided throughout the documentation for individual classes.

## Module Organization

The library is organized into functional modules:

- **core** - Core framework components
- **io** - Hardware I/O interfaces
- **channels** - Position control
- **generators** - Motion generation
- **kinematics** - Coordinate transformations
- **filters** - Signal processing
- **interfaces** - External protocols
- **recording** - Motion recording and playback
- **inputs** - Physical input devices

## Supported Hardware

Stepdance runs on Teensy microcontrollers and supports:

- Basic Modules (single input port)
- Machine Controller Modules (multiple I/O ports)

## Additional Resources

- GitHub Repository: https://github.com/imoyer/stepdance
- Hardware Documentation: [Add link to hardware docs]
- Tutorials: [Add link to tutorials]
