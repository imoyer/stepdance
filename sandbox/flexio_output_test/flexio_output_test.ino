#define PIN_HARDCODE_OUT  20 //this is just a simple test as a baseline

// ---- FLEXIO STUFF BELOW ----
#define PIN_STEP_OUT    35  //Teensy Pin 35, Pad B1_12, FlexIO Pin 2:28
#define PIN_DIR_OUT     34  //Teensy Pin 34, Pad B1_13, FlexIO Pin 2:29

void setup() {
  pinMode(PIN_HARDCODE_OUT, OUTPUT);
  pinMode(PIN_STEP_OUT, OUTPUT);
  pinMode(PIN_DIR_OUT, OUTPUT);

  // --- CONFIG FLEXIO ---
  // I'm heavily referencing this post: https://forum.pjrc.com/index.php?threads/teensy-4-1-how-to-start-using-flexio.66201/

  // -- Enable Clock to FLEXIO2 --
  // We do this throught the CCM Clock Gating Register 3 (See P1021 for assignment, P1087 for register definition)
  CCM_CCGR3 |= CCM_CCGR3_FLEXIO2(CCM_CCGR_ON);

  // -- Set Clock Speed --
  // Default is PLL3_SW_CLK (480MHz) / 16 = 30 MHz
  // According to table on 1032, FLEXIO2 can go up to 120 MHz
  // So, we'll set CS1CDR[FLEXIO2_CLK_PODF] to /2. In combination with the default /2 value of
  // CS1CDR[FLEXIO2_CLK_PRED], we'll end up with /4 and a clock speed of 480MHz/4 = 120MHz
  CCM_CS1CDR &= ~(CCM_CS1CDR_FLEXIO2_CLK_PODF(7)); //clear all bits
  CCM_CS1CDR |= CCM_CS1CDR_FLEXIO2_CLK_PODF(1); //set to 1 (/2, see p1063)

  // -- Enable FlexIO --
  FLEXIO2_CTRL |= 1; // Bit0 of FLEXIO2_CTRL enables the module. (P2916)

  // -- MAP PADS TO FLEXIO MODULE PINS --
  IOMUXC_SW_MUX_CTL_PAD_GPIO_B1_12 = 4; //Map pad B1_12 to FLEXIO2 Module Pin FLEXIO28
  IOMUXC_SW_MUX_CTL_PAD_GPIO_B1_13 = 4; //Map pad B1_13 to FLEXIO2 Module Pin FLEXIO29
  
  // -- Configure Shifter 0 For Step Output --

  // Set Shifter Control Register (P2926)
  FLEXIO2_SHIFTCTL0	= FLEXIO_SHIFTCTL_TIMSEL(0) |	// select timer0 as input. Multiple shifters can share a timer input.
			  // FLEXIO_SHIFTCTL_TIMPOL	|			          // on positive edge
			  FLEXIO_SHIFTCTL_PINCFG(3) |			          // pin output enabled
			  FLEXIO_SHIFTCTL_PINSEL(28)|			          // FLEXIO PIN 28
			  // FLEXIO_SHIFTCTL_PINPOL	|			          // active high
			  FLEXIO_SHIFTCTL_SMOD(2);					        // transmit mode

  // Set Shifter Config Register (P2927)
  FLEXIO2_SHIFTCFG0	= FLEXIO_SHIFTCFG_PWIDTH(0)	|	// single bit
			  // FLEXIO_SHIFTCFG_INSRC |	              // pin input
			  FLEXIO_SHIFTCFG_SSTOP(0) |			        // stop bit disabled
			  FLEXIO_SHIFTCFG_SSTART(0);					    // start bit disabled

  // Set Shifter Control Register (P2926)
  FLEXIO2_SHIFTCTL1	= FLEXIO_SHIFTCTL_TIMSEL(0) |	// select timer0 as input. Multiple shifters can share a timer input.
			  // FLEXIO_SHIFTCTL_TIMPOL	|			          // on positive edge
			  FLEXIO_SHIFTCTL_PINCFG(3) |			          // pin output enabled
			  FLEXIO_SHIFTCTL_PINSEL(29)|			          // FLEXIO PIN 29
			  // FLEXIO_SHIFTCTL_PINPOL	|			          // active high
			  FLEXIO_SHIFTCTL_SMOD(2);					        // transmit mode

  // Set Shifter Config Register (P2927)
  FLEXIO2_SHIFTCFG1	= FLEXIO_SHIFTCFG_PWIDTH(0)	|	// single bit
			  // FLEXIO_SHIFTCFG_INSRC |	              // pin input
			  FLEXIO_SHIFTCFG_SSTOP(0) |			        // stop bit disabled
			  FLEXIO_SHIFTCFG_SSTART(0);					    // start bit disabled

  // -- Configure Timer --

  // Set Timer0 Compare Register
  FLEXIO2_TIMCMP0 = (63 <<8) | //According to the datasheet this should give us 32 bits of output. We'll see... 
        ((120/2)-1); // We want an output every us, so we need CLK/120

  // Set Timer0 Control Register (P2933)
  FLEXIO2_TIMCTL0	= FLEXIO_TIMCTL_TRGSEL(1)|			  // Trigger on Shifter 0 Status Flag
			  FLEXIO_TIMCTL_TRGPOL |			                // Invert the Trigger -- we want to trigger when the status flag is cleared,
                                                    // which happens when new data is loaded into Shifter 0's buffer
			  FLEXIO_TIMCTL_TRGSRC |			                // internal trigger
			  FLEXIO_TIMCTL_PINCFG(0) |			              // timer pin output disabled
			  FLEXIO_TIMCTL_PINSEL(0)	|			              // Input pin select, not in use
			  // FLEXIO_TIMCTL_PINPOL		|			            // input pin polarity, not in use
			  FLEXIO_TIMCTL_TIMOD(1);					            // timer mode: 8 bit baud mode
  
  FLEXIO2_TIMCFG0	= FLEXIO_TIMCFG_TIMOUT(0)	|		// timer output = high, not affcted by reset
			  FLEXIO_TIMCFG_TIMDEC(0)	|			            // decrement on FlexIO clock
			  FLEXIO_TIMCFG_TIMRST(0) |			            // dont reset timer 
			  FLEXIO_TIMCFG_TIMDIS(2)	|			            // disable timer on timer compare
			  FLEXIO_TIMCFG_TIMENA(2)	|			            // enable timer on trigger high
			  FLEXIO_TIMCFG_TSTOP(0);			              // stop bit disabled
			  //FLEXIO_TIMCFG_TSTART					          // start bit disabled
}

void loop() {
  // put your main code here, to run repeatedly:
  // digitalWrite(PIN_HARDCODE_OUT, HIGH);
  // delayMicroseconds(1);
  // digitalWrite(PIN_HARDCODE_OUT, LOW);
  // delayMicroseconds(1);
  delay(10);
  // FLEXIO2_SHIFTBUF1 = 0xFFFFFFFF;
  // FLEXIO2_SHIFTBUF1 = 0x55555555;
  FLEXIO2_SHIFTBUF1 = 0b1111111100000001111110;
  FLEXIO2_SHIFTBUF0 = 0b1111111001111110011111001111000; //WRITE LAST... this triggers the transmit

}
