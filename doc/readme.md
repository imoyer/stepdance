# Stepdance Documentation
***
## What is Stepdance?
Stepdance is a framework for creative motion control. Stepdance is:

- **Real-time**: Control motors with essentially zero latency using a variety of inputs such as potentiometers and encoders.
- **Modular**: Stepdance programs can be composed of software modules, and physical stepdance boards can be chained together.
- **Hybrid**: Blend real-time inputs and manipulation with traditional pre-planned approaches such as G-code.
***
## Stepdance Hardware
### Driver Module v1.0
![](/doc/images/module_driver.png)

#### Stepper Drivers and Motor Interfaces
This board is designed for [BIGTREETECH TMC2209](https://pages.github.com/) driver modules. The sockets have the following pinouts, and microstepping is set by bridging the 2x2 male header pins as shown:
![](/doc/images/stepper_driver_sockets.png)

Note that the +5V supply pin is additional to the pins on the TMC2209, and is for future expansion options. These sockets provide a flexible interface for a variety of outputs. For example, our servo stepper board accepts Step/Direction inputs and converts into a hobby servo output.

**All driver socket logic connections (e.g. MS1/MS2, STEP, DIR, EN) are at 3.3V**

Motors interface with the Stepdance driver module via a 4-pin motor header and a 3-pin limit switch header. **The limit switch input is at 5V** for compatibility with industrial proximity switches.

Motor interface header pinouts are as shown:
![](/doc/images/motor_connectors.png)

#### Encoders and Analog / Digital Inputs
Two quadrature encoder inputs and six general-purpose IO (GPIO) ports are provided:

- ENC1 and ENC2 are **5V** quadrature inputs, and are compatible with standard optical encoders like those from US Digital or [TAISS](/doc/taiss.md).
- A1 thru A4 are **3.3V** _analog_ and _digital_ inputs, as well as digital outputs.
- D1 and D2 are **3.3V** _digital_ inputs and outputs.

These are all Molex SL series connectors. Pinouts are as shown:
![](/doc/images/input_connectors.png)

The A and B encoder inputs are pulled high thru 10k resistors, for compatibility with open-collector encoders like the TAISS modules.

A1-A4 and D1-D2 do _not_ have external pullups, but internal pullups on the microcontroller can be enabled.

#### QWIIC Connectors
Two are provided. It is intended that QWIIC1 is for interfacing with a board-mounted display and knob, while QWIIC0 is for user accessories.

Note though that QWIIC1 exposes the TX4/RX4 serial port **(at 3.3V)**, which can be useful for interfacing with certain accessories. Pinout as shown:
![](/doc/images/qwiic.png)
![](/doc/images/sparkfun_qwiic.jpg)

#### DC Power Inputs
The onboard Teensy 4.1 and most IO gets power from the USB connector on the Teensy itself. A 5.5mm OD / 2.1mm ID barrel connector accepts 12V-24V power that is routed to the stepper drivers.

#### Stepdance Ports

- four independent stepdance input ports A->D.
- stepdance output ports A->D are connected to stepper driver sockets
- additionally, output port D is mirrored to a standard stepdance TRRS connector.

We discuss stepdance ports in detail later in this document.

#### Display and Control Knob Mounting
Board-mounted standoffs are provided for a [Sparkfun QWIIC OLED](https://www.sparkfun.com/sparkfun-qwiic-oled-display-0-91-in-128x32-lcd-24606.html) and a [Sparkfun QWIIC Twist knob](https://www.sparkfun.com/sparkfun-qwiic-twist-rgb-rotary-encoder-breakout.html).

Although three standoffs are shown for the OLED, **only two should be installed on the PCB**. For an old-style OLED display, these are the left and right-most standoffs. For a new-style OLED, these are the top row of standoffs.

#### Microcontroller
The Stepdance driver module uses a Teensy 4.1, which is based off the badass IMXRT1062 (32-bit, 600MHz). The pinout we use is shown below:
![](/doc/images/module_driver_teensy.png)

Notes on pin selection:

- stepdance outputs A-D use FlexIO
- stepdance inputs A-D use FlexPMW
- quadrature inputs use the QuadEncoder library
- I2C0 is connected to QWIIC0
- I2C1 is connected to QWIIC1, and also exposes TX4/RX4
- output ref pins are analog inputs that allow the MCU to read the current setting on the drivers. Eventually we'll use this to help guide the user in setting the current.
- output enable pins individually enable each driver
- "LIM" are limit switch inputs for each output port.