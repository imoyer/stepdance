// --- MIXING METAPHORS ---
//
// A COLLABORATION BETWEEN EXPRESSIVE COMPUTATION AND THE CLAY WORLD PROJECT
// ILAN MOYER, JENNIFER JACOBS, SAM BOURGAULT, AND DEVON FROST
//
// --- FIRST MUX / DEMUX TEST ---
//
// Written January 2025
//
// This code builds on the wheelprint control system, with an important distinction:
// Whereas previously one motion stream carried one axis, we now multiplex multiple axes
// into a single motion stream, using pulse length encoding. The goal is to make
// the overall control system topology more understandable, flexible, and easier to wire.
// 
// The risk we take is that aspects of the controller may be _less_ understandable, because
// the mapping between axes and channels is now done digitally, and is not obvious from following
// the wires.

void setup() {
  // put your setup code here, to run once:

}

void loop() {
  // put your main code here, to run repeatedly:

}
