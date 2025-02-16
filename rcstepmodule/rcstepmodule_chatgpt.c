#include <avr/io.h>
#include <avr/interrupt.h>

#define STEP_PIN    PA7  // Step input (Pin 7)
#define DIR_PIN     PA6  // Direction input (Pin 6)
#define SERVO_PIN   PA5  // Servo PWM output (Pin 5)

#define SERVO_MIN  1000  // 1ms pulse (0 degrees)
#define SERVO_MAX  2000  // 2ms pulse (180 degrees)
#define SERVO_MID  1500  // 1.5ms pulse (90 degrees)

volatile int16_t step_count = 0;
volatile uint16_t servo_pulse = SERVO_MID;

void setup();
void update_servo();

int main(void) {
    setup();
    
    while (1) {
        update_servo(); // Adjust servo based on step input
    }
}

// Configure Timer1 for PWM and enable pin change interrupt
void setup() {
    // Configure PA5 (SERVO) as output
    DDRA |= (1 << SERVO_PIN);
    
    // Configure PA7 (STEP) and PA6 (DIR) as inputs with pull-ups
    DDRA &= ~((1 << STEP_PIN) | (1 << DIR_PIN));
    PORTA |= (1 << STEP_PIN) | (1 << DIR_PIN);

    // Enable Pin Change Interrupt on PA7 (STEP)
    GIMSK |= (1 << PCIE0);      // Enable PCINT interrupt
    PCMSK0 |= (1 << PCINT7);    // Enable PA7 as interrupt source
    sei();                      // Enable global interrupts

    // Configure Timer1 (16-bit) for 50Hz PWM
    TCCR1A = (1 << COM1A1) | (1 << WGM11);  // Fast PWM, non-inverting
    TCCR1B = (1 << WGM13) | (1 << WGM12) | (1 << CS11); // Prescaler 8
    ICR1 = 20000;   // 20ms period (50Hz)

    OCR1A = SERVO_MID; // Start at middle position
}

// Interrupt Service Routine: Detect step pulses on PA7
ISR(PCINT0_vect) {
    if (PINA & (1 << STEP_PIN)) {  // Rising edge detected
        if (PINA & (1 << DIR_PIN)) {
            step_count++;  // Move forward
        } else {
            step_count--;  // Move backward
        }
    }
}

// Update servo PWM based on step count
void update_servo() {
    servo_pulse = SERVO_MID + (step_count * 5); // Adjust scale as needed

    // Limit pulse width within valid range
    if (servo_pulse < SERVO_MIN) servo_pulse = SERVO_MIN;
    if (servo_pulse > SERVO_MAX) servo_pulse = SERVO_MAX;

    OCR1A = servo_pulse; // Set PWM duty cycle
}
