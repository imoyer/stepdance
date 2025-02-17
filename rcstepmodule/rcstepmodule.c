// RC Stepper Module Makefile
// Mixing Metaphors Project
// Ilan E. Moyer
// February 16th, 2025

#include <stdlib.h>
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>

// HEADERS
void setup();
void loop();
void set_pulse_period(uint16_t pulse_period_us);
void update_servo();

// PORT AND PIN DEFINITIONS
#define IO_PORT	PORTA
#define IO_DDR	DDRA
#define IO_PIN	PINA
#define DIR_PIN		PA6
#define STEP_PIN	PA7
#define SERVO_PIN	PA5

// ---- SETTINGS ----
#define PULSE_PERIOD_US 24000 //24ms pulse period
#define PULSE_PERIOD_OFFSET 850 //adjustment on time between pulses
#define PULSE_WIDTH_OFFSET 40 //adjustment to pulse width

#define SERVO_TOP_LIMIT_STEPS 500 // +/- 500 steps is full-range, if 1 STEP == 1uS of pulse width
#define SERVO_BOTTOM_LIMIT_STEPS -500
#define SERVO_MIDPOINT_PULSE_WIDTH_US 1500

// ---- STATE VARIABLES ----
volatile int32_t input_position_steps = 0; // tracks the input position


//	---- MAIN ----
int main(){
	setup();
	while(1){
		loop();
	}
};

// SETUP
void setup(){
	// Configure IO pins
	IO_DDR &= ~((1<<DIR_PIN) | (1<<STEP_PIN)); //step and dir are inputs
	IO_DDR |= (1<<SERVO_PIN); //servo is an output

	// Configure Timer1
	TCCR1A = (1<<COM1B1)|(0<<COM1B0)|(1<<WGM11)|(0<<WGM10); //Fast PWM, set OC1B on BOTTOM, clear on OCR1B, top at ICR1;
	TCCR1B = (1<<WGM13)|(1<<WGM12)|(0b010<<CS10); //Fast PWM, CLK/8 (1us per timer tick)

	set_pulse_period(PULSE_PERIOD_US);
	OCR1B = SERVO_MIDPOINT_PULSE_WIDTH_US + PULSE_WIDTH_OFFSET; //1.5ms pulse length

	// Enable Pin Change Interrupt on PA7 (STEP)
    GIMSK |= (1 << PCIE0);      // Enable PCINT interrupt
    PCMSK0 |= (1 << PCINT7);    // Enable PA7 as interrupt source

	//enable global interrupts
	sei();
};


void loop(){
	update_servo();
}

void set_pulse_period(uint16_t pulse_period_us){
	ICR1 = pulse_period_us + PULSE_PERIOD_OFFSET;
}

// Interrupt Service Routine: Detect step pulses
ISR(PCINT0_vect) {
	uint8_t input_port = PINA;
    if (input_port & (1 << STEP_PIN)) {  // Rising edge detected
        if (input_port & (1 << DIR_PIN)) {
            input_position_steps++;  // Move forward
        } else {
            input_position_steps--;  // Move backward
        }
    }
}


void update_servo(){
// Update the servo position based on current step position
	if(TIFR1 & (1<<TOV1)){ //run when counter has reached top value.
		if((input_position_steps >= SERVO_BOTTOM_LIMIT_STEPS) && (input_position_steps <= SERVO_TOP_LIMIT_STEPS)){ //enforce limits
			cli();
			OCR1B = SERVO_MIDPOINT_PULSE_WIDTH_US + PULSE_WIDTH_OFFSET + input_position_steps; //we turn off interrupts b/c this is not an atomic operation
			sei();
		}
		TIFR1 = (1<<TOV1); //clear the flag
	}
}