\page ebb Axidraw EBB Controller Emulator Example
## Overview

This example demonstrates using a Stepdance Interface module to emulate the AxiDraw EBB controller so that the standard Inkscape extension can drive the machine. It builds on the \ref stepasketch real-time jogging example by adding:

- An Interface module that accepts pre-planned motion commands (G-code or SVG-derived paths).
- An Interpolator that converts these commands into real-time motion streams.
- A real-time speed control via an analog input (e.g. a potentiometer) mapped to interpolator velocity.

Additional documentation and wiring details will be expanded here.
