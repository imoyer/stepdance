\page highspeed High-Speed Frame Configuration
## High-Speed Frame Configuration

Stepdance normally operates with a 40µs frame period, allowing up to four axis signals (R, θ, Z, E) in a single frame with 2µs inter-signal gaps. Certain applications benefit from higher frame rates (shorter frame periods) to reduce motion granularity and improve control loop responsiveness.

### Concepts
At higher frame rates:
- The available time budget for signals shrinks, potentially reducing the number of distinct axis signals that can fit into one frame.
- Inter-signal gaps may need to be reduced or eliminated; firmware must guarantee DIR stability before the STEP falling edge is sampled.
- Maximum aggregate step frequency increases; verify motor driver pulse width requirements.

### Considerations
When enabling a high-speed mode:
1. Confirm that peripheral timers (FlexIO / PWM) can generate the required pulse widths at the increased rate.
2. Ensure ISR execution time per frame remains below the new frame period to avoid jitter and missed frames.
3. Evaluate signal integrity: shorter pulses may demand tighter PCB trace impedance control and cleaner edges.
4. Thermal / driver limits: higher step rates can increase driver and motor heating.

### Future Work
Implementation details (API to select frame period, dynamic signal packing algorithm) will be documented here once stabilized.

### See Also
\ref stepasketch for a baseline example at the default frame rate.