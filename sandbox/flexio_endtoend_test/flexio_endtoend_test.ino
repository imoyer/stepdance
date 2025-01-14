#define PIN_HARDCODE_OUT  20 //this is just a simple test as a baseline

// ---- PIN DEFINITIONS ----
#define PIN_STEP_OUT    23  //Teensy Pin 23, Pad AD_B1_09, FlexIO Pin 3:9
#define PIN_DIR_OUT     22  //Teensy Pin 22, Pad AD_B1_08, FlexIO Pin 3:08

#define PIN_STEP_IN     3  //Teensy Pin 3, Pad EMC_05, PWM4 2B
#define PIN_DIR_IN      2  //Teensy Pin 2, Pad EMC_04, PWM4 2A

#define PIN_BUTTON_START 16 //BTN A
#define PIN_BUTTON_RESET 15 //BTN B

// ---- FLEXPWM MODULE DEFINITIONS ----
#define PIN_STEP_MUX    1   //ALT1
IMXRT_FLEXPWM_t *PIN_STEP_FLEXPWM = &IMXRT_FLEXPWM4; //This structure maps into the register memory map, and is defined in imxrt.h
#define PIN_STEP_SUBMODULE 2
#define PIN_STEP_SUBMODULE_MASK 1 << PIN_STEP_SUBMODULE
#define PIN_STEP_CHANNEL 2 //X:0, A:1, B:2
const uint8_t PIN_STEP_IRQ = IRQ_FLEXPWM4_2;

// ---- STATE MEMORY ----
volatile uint16_t capture_0;
volatile uint16_t capture_1;
volatile uint16_t capture_difference;
volatile uint32_t fire_count = 0;
volatile int32_t capture_x_count = 0;
volatile int32_t capture_y_count = 0;
volatile int32_t capture_r_count = 0;
volatile int32_t capture_t_count = 0;
volatile int32_t capture_z_count = 0;
volatile int32_t capture_e_count = 0;

const uint32_t max_output_count = 1000000;
volatile uint32_t output_count = max_output_count; //when not equal, we'll send
volatile uint8_t releasing_output_set = 0; //if 1, indicates that currently working towards sending all the pulses.

void setup() {
  pinMode(PIN_HARDCODE_OUT, OUTPUT);
  pinMode(PIN_STEP_OUT, OUTPUT);
  pinMode(PIN_DIR_OUT, OUTPUT);
  pinMode(PIN_STEP_IN, INPUT);
  pinMode(PIN_DIR_IN, INPUT);

  pinMode(PIN_BUTTON_START, INPUT_PULLDOWN);
  pinMode(PIN_BUTTON_RESET, INPUT_PULLDOWN);

  // --- CONFIG FLEXIO ---
  // I'm heavily referencing this post: https://forum.pjrc.com/index.php?threads/teensy-4-1-how-to-start-using-flexio.66201/

  // -- Enable Clock to FLEXIO2 --
  // We do this throught the CCM Clock Gating Register 3 (See P1021 for assignment, P1087 for register definition)

  CCM_CCGR7 |= CCM_CCGR7_FLEXIO3(CCM_CCGR_ON);

  // -- Set Clock Speed --
  // Default is PLL3_SW_CLK (480MHz) / 16 = 30 MHz
  // According to table on 1032, FLEXIO2 can go up to 120 MHz
  // So, we'll set CS1CDR[FLEXIO2_CLK_PODF] to /2. In combination with the default /2 value of
  // CS1CDR[FLEXIO2_CLK_PRED], we'll end up with /4 and a clock speed of 480MHz/4 = 120MHz
  // Note: This should also set the clock dividers for FlexIO3
  CCM_CS1CDR &= ~(CCM_CS1CDR_FLEXIO2_CLK_PODF(7)); //clear all bits
  CCM_CS1CDR |= CCM_CS1CDR_FLEXIO2_CLK_PODF(1); //set to 1 (/2, see p1063)

  // -- Enable FlexIO --
  FLEXIO3_CTRL |= 1; // Bit0 of FLEXIO2_CTRL enables the module. (P2916)

  // -- MAP PADS TO FLEXIO MODULE PINS --
  // IOMUXC_SW_MUX_CTL_PAD_GPIO_B1_12 = 4; //Map pad B1_12 to FLEXIO2 Module Pin FLEXIO28
  // IOMUXC_SW_MUX_CTL_PAD_GPIO_B1_13 = 4; //Map pad B1_13 to FLEXIO2 Module Pin FLEXIO29

  IOMUXC_SW_MUX_CTL_PAD_GPIO_AD_B1_09 = 9;
  IOMUXC_SW_MUX_CTL_PAD_GPIO_AD_B1_08 = 9;
  
  // -- Configure Shifter 0 For Step Output --

  // Set Shifter Control Register (P2926)
  FLEXIO3_SHIFTCTL0	= FLEXIO_SHIFTCTL_TIMSEL(0) |	// select timer0 as input. Multiple shifters can share a timer input.
			  // FLEXIO_SHIFTCTL_TIMPOL	|			          // on positive edge
			  FLEXIO_SHIFTCTL_PINCFG(3) |			          // pin output enabled
			  FLEXIO_SHIFTCTL_PINSEL(9)|			          // FLEXIO PIN 9
			  // FLEXIO_SHIFTCTL_PINPOL	|			          // active high
			  FLEXIO_SHIFTCTL_SMOD(2);					        // transmit mode

  // Set Shifter Config Register (P2927)
  FLEXIO3_SHIFTCFG0	= FLEXIO_SHIFTCFG_PWIDTH(0)	|	// single bit
			  // FLEXIO_SHIFTCFG_INSRC |	              // pin input
			  FLEXIO_SHIFTCFG_SSTOP(0) |			        // stop bit disabled
			  FLEXIO_SHIFTCFG_SSTART(0);					    // start bit disabled

  // Set Shifter Control Register (P2926)
  FLEXIO3_SHIFTCTL1	= FLEXIO_SHIFTCTL_TIMSEL(0) |	// select timer0 as input. Multiple shifters can share a timer input.
			  // FLEXIO_SHIFTCTL_TIMPOL	|			          // on positive edge
			  FLEXIO_SHIFTCTL_PINCFG(3) |			          // pin output enabled
			  FLEXIO_SHIFTCTL_PINSEL(8)|			          // FLEXIO PIN 29
			  // FLEXIO_SHIFTCTL_PINPOL	|			          // active high
			  FLEXIO_SHIFTCTL_SMOD(2);					        // transmit mode

  // Set Shifter Config Register (P2927)
  FLEXIO3_SHIFTCFG1	= FLEXIO_SHIFTCFG_PWIDTH(0)	|	// single bit
			  // FLEXIO_SHIFTCFG_INSRC |	              // pin input
			  FLEXIO_SHIFTCFG_SSTOP(0) |			        // stop bit disabled
			  FLEXIO_SHIFTCFG_SSTART(0);					    // start bit disabled

  // -- Configure Timer --

  // Set Timer0 Compare Register
  FLEXIO3_TIMCMP0 = (63 <<8) | //According to the datasheet this should give us 32 bits of output. We'll see... 
        ((120/2)-1); // We want an output every us, so we need CLK/120

  // Set Timer0 Control Register (P2933)
  FLEXIO3_TIMCTL0	= FLEXIO_TIMCTL_TRGSEL(1)|			  // Trigger on Shifter 0 Status Flag
			  FLEXIO_TIMCTL_TRGPOL |			                // Invert the Trigger -- we want to trigger when the status flag is cleared,
                                                    // which happens when new data is loaded into Shifter 0's buffer
			  FLEXIO_TIMCTL_TRGSRC |			                // internal trigger
			  FLEXIO_TIMCTL_PINCFG(0) |			              // timer pin output disabled
			  FLEXIO_TIMCTL_PINSEL(0)	|			              // Input pin select, not in use
			  // FLEXIO_TIMCTL_PINPOL		|			            // input pin polarity, not in use
			  FLEXIO_TIMCTL_TIMOD(1);					            // timer mode: 8 bit baud mode
  
  FLEXIO3_TIMCFG0	= FLEXIO_TIMCFG_TIMOUT(0)	|		// timer output = high, not affcted by reset
			  FLEXIO_TIMCFG_TIMDEC(0)	|			            // decrement on FlexIO clock
			  FLEXIO_TIMCFG_TIMRST(0) |			            // dont reset timer 
			  FLEXIO_TIMCFG_TIMDIS(2)	|			            // disable timer on timer compare
			  FLEXIO_TIMCFG_TIMENA(2)	|			            // enable timer on trigger high
			  FLEXIO_TIMCFG_TSTOP(0);			              // stop bit disabled
			  //FLEXIO_TIMCFG_TSTART					          // start bit disabled


  // ---- CONFIG FLEXPWM ----
  // We'll use the "e-capture" feature of the PWM module to precisely capture and time our multiplexed input signals.
  // For this I'm referencing the manual, as well as the Teensy FreqMeasureMultiIMXRT library.
  //
  // I'll follow a similar pattern to the FlexIO: configure the clock, then the pins, then the registers.
  // What's new are the interrupts, which will be fun/interesting to tackle.
  // We have STEP on FLEXPWM1 2X, and DIR on FLEXPWM1 3X


  // -- Enable Clock --
  // We'll do this through CCGR4, which gates the clock to FLEXPWM1
  CCM_CCGR4 |= CCM_CCGR4_PWM4(CCM_CCGR_ON);
  
  // -- MUX the STEP PIN --
  *(portConfigRegister(PIN_STEP_IN)) = PIN_STEP_MUX;
  
  // -- CONFIGURE THE FLEXPWM MODULE ---
  PIN_STEP_FLEXPWM->FCTRL0 |= FLEXPWM_FCTRL0_FLVL(PIN_STEP_SUBMODULE_MASK);
	PIN_STEP_FLEXPWM->FSTS0 = PIN_STEP_SUBMODULE_MASK;
	PIN_STEP_FLEXPWM->MCTRL |= FLEXPWM_MCTRL_CLDOK(PIN_STEP_SUBMODULE_MASK);
	PIN_STEP_FLEXPWM->SM[PIN_STEP_SUBMODULE].CTRL2 = FLEXPWM_SMCTRL2_INDEP;
	PIN_STEP_FLEXPWM->SM[PIN_STEP_SUBMODULE].CTRL = FLEXPWM_SMCTRL_HALF;
	PIN_STEP_FLEXPWM->SM[PIN_STEP_SUBMODULE].INIT = 0;
	PIN_STEP_FLEXPWM->SM[PIN_STEP_SUBMODULE].VAL0 = 0;
	PIN_STEP_FLEXPWM->SM[PIN_STEP_SUBMODULE].VAL1 = 65535; //used to capture overflow, this is the X register
	PIN_STEP_FLEXPWM->SM[PIN_STEP_SUBMODULE].VAL2 = 0;
	PIN_STEP_FLEXPWM->SM[PIN_STEP_SUBMODULE].VAL3 = 0;
	PIN_STEP_FLEXPWM->SM[PIN_STEP_SUBMODULE].VAL4 = 0;
	PIN_STEP_FLEXPWM->SM[PIN_STEP_SUBMODULE].VAL5 = 0;
	PIN_STEP_FLEXPWM->MCTRL |= FLEXPWM_MCTRL_LDOK(PIN_STEP_SUBMODULE_MASK) | FLEXPWM_MCTRL_RUN(PIN_STEP_SUBMODULE_MASK);

  // Configure the capture functionality
  uint16_t capture_mode = FLEXPWM_SMCAPTCTRLB_EDGB0(2) | FLEXPWM_SMCAPTCTRLB_EDGB1(1); //capture time between rising and falling edges
  PIN_STEP_FLEXPWM->SM[PIN_STEP_SUBMODULE].CAPTCTRLB = capture_mode | FLEXPWM_SMCAPTCTRLB_ARMB;

  // Configure and Enable Interrupts
  uint16_t interrupt_enable_flags = FLEXPWM_SMINTEN_CB1IE; //interrupt on second capture
  PIN_STEP_FLEXPWM->SM[PIN_STEP_SUBMODULE].INTEN |= interrupt_enable_flags;

  // Attach interrupts
  __disable_irq(); //disable interrupts
  attachInterruptVector((IRQ_NUMBER_t) PIN_STEP_IRQ, &flexpwm_interrupt);
  NVIC_SET_PRIORITY(PIN_STEP_IRQ, 48);
  NVIC_ENABLE_IRQ(PIN_STEP_IRQ);
  __enable_irq(); //re-enable interrupts
  Serial.begin(115200);
}

void flexpwm_interrupt(){
  int8_t dir = digitalReadFast(PIN_DIR_IN);
  // The following line is important. If we don't clear the status flag, the interrupt gets called constantly and the program hangs.
  // If we set it too late in the program, the interrupt gets triggered multiple times.
  // If we set it too early though, then things also behave weirdly.
  PIN_STEP_FLEXPWM->SM[PIN_STEP_SUBMODULE].STS = FLEXPWM_SMSTS_CFB1; //this is important! Otherwise it hangs.

  if(dir == 0){
    dir = -1;
  }
  capture_difference = PIN_STEP_FLEXPWM->SM[PIN_STEP_SUBMODULE].CVAL5 - PIN_STEP_FLEXPWM->SM[PIN_STEP_SUBMODULE].CVAL4;
  fire_count ++;
  if (capture_difference < 375){
    capture_x_count += dir;
  }else if (capture_difference < 525) {
    capture_y_count += dir;
  }else if (capture_difference < 675){
    capture_r_count += dir;
  }else if (capture_difference < 825) {
    capture_t_count += dir;
  }else if (capture_difference < 975) {
    capture_z_count += dir;
  }else if (capture_difference < 1125){
    capture_e_count += dir;
  }
}
uint32_t loopcount = 0;

void loop() {
  // put your main code here, to run repeatedly:
  // digitalWrite(PIN_HARDCODE_OUT, HIGH);
  // delayMicroseconds(1);
  // digitalWrite(PIN_HARDCODE_OUT, LOW);
  // delayMicroseconds(1);
  // FLEXIO2_SHIFTBUF1 = 0b1111111100000001111110;
  // FLEXIO2_SHIFTBUF0 = 0b1111111001111110011111001111000; //WRITE LAST... this triggers the transmit

  if(digitalReadFast(PIN_BUTTON_RESET)){
    capture_x_count = 0;
    capture_y_count = 0;
    capture_r_count = 0;
    capture_t_count = 0;
    capture_z_count = 0;
    capture_e_count = 0;
    fire_count = 0;
  }
  if(digitalReadFast(PIN_BUTTON_START) && output_count == max_output_count){
    output_count = 0;
  }
  
  delayMicroseconds(40); //fastest frame rate is 25 khz
  if(output_count < max_output_count){
    transmit();
    output_count ++;
  }
  if(loopcount%25000 == 0){
    Serial.print("TOTAL INTERRUPT FIRES: ");
    Serial.println(fire_count);
    Serial.print("X COUNT: ");
    Serial.println(capture_x_count);
    Serial.print("Y COUNT: ");
    Serial.println(capture_y_count);
    Serial.print("R COUNT: ");
    Serial.println(capture_r_count);
    Serial.print("T COUNT: ");
    Serial.println(capture_t_count);
    Serial.print("Z COUNT: ");
    Serial.println(capture_z_count);
    Serial.print("E COUNT: ");
    Serial.println(capture_e_count);  
  }
  loopcount ++;
}

void transmit(){
  FLEXIO3_SHIFTBUF1 = 0b00000000000000111111110000011110;
  FLEXIO3_SHIFTBUF0 = 0b00000011111110011111100111001100;  //WRITE LAST... this triggers the transmit
}