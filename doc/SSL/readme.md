# Stepdance Software Library Documentation

## Introduction

Welcome to the Stepdance Software Library documentation. Stepdance is a framework for building motion control systems using modular, composable components.

## Overview

The Stepdance library provides a flexible architecture for motion control through several key concepts:

- **BlockPorts**: Unified interface for connecting components and routing data between them
- **Channels**: Position state management and step/direction signal generation
- **Input/Output Ports**: Physical hardware interface for receiving and transmitting motion signals
- **Generators**: Motion pattern and trajectory generation
- **Kinematics**: Coordinate system transformations (Cartesian, polar, CoreXY, etc.)
- **Filters**: Signal smoothing and noise reduction
- **Interfaces**: External protocol support (G-code, EiBotBoard, etc.)

## Getting Started

To get started with Stepdance:

1. **Understand BlockPorts**: BlockPorts are the fundamental communication mechanism between components. See the core module for details.

2. **Configure Output Ports**: Set up your physical output ports to send step/direction signals. See the I/O module for OutputPort configuration.

3. **Create Channels**: Channels drive motors to target positions. See the channels module for Channel setup and usage.

4. **Add Motion Sources**: Connect input ports, generators, or other motion sources to your channels using BlockPort mapping.

## Examples

Example code snippets are provided throughout the documentation. Look for code examples in the documentation for each class.

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

## License

[Add your license information here]
