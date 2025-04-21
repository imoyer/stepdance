#include <stdint.h>
#include "core.hpp"
/*
Digital Input Module of the StepDance Control System

This module is responsible for providing an interface to digital inputs, e.g. as buttons.

[More Details to be Added]

A part of the Mixing Metaphors Project
(c) 2025 Ilan Moyer, Jennifer Jacobs, Devon Frost

*/

#ifndef digital_in_h //prevent importing twice
#define digital_in_h

#define DIGITAL_IN_DEFAULT_DEBOUNCE_MS  50
#define MAX_NUM_BUTTONS 20

// Button Mode
#define BUTTON_MODE_STANDARD 0 // button state reflects pin state, modulo debounce and invert flag
#define BUTTON_MODE_TOGGLE 1 // button state toggles each button press
#define BUTTON_NON_INVERTED 0
#define BUTTON_INVERTED 1

#define BUTTON_STATE_RELEASED 0
#define BUTTON_STATE_PRESSED 1
#define BUTTON_CHANGED 1

class ButtonKilohertzPlugin : public Plugin{ //we'll set up to run on every kilohertz frame
  public:
    ButtonKilohertzPlugin();
    void begin();
  protected:
    void run();
};

class Button{
  public:
    Button();
    uint8_t read_state(); //returns button state
    uint8_t has_changed(); //returns 1 if the button state has changed
    void set_callback_on_toggle(void (*callback_function)());
    void set_callback_on_press(void (*callback_function)());
    void set_callback_on_release(void (*callback_function)());
    void set_debounce_ms(uint16_t debounce_ms);
    void begin(uint8_t pin); //defaults to INPUT_PULLUP
    void begin(uint8_t pin, uint8_t mode);
    void set_mode(uint8_t button_mode); //standard, or toggle
    void on_frame(); //gets called at each kilohertz frame
    void set_invert(); //inverts the button state
    void clear_invert(); //clears the invert button mode
    static Button* registered_buttons[MAX_NUM_BUTTONS]; //stores all registered buttons
    static uint8_t num_registered_buttons;
    static ButtonKilohertzPlugin kilohertz_plugin; // runs in the kilohertz frame

  private:
    uint8_t input_pin;
    uint8_t invert = BUTTON_NON_INVERTED;
    uint16_t debounce_period_ms = DIGITAL_IN_DEFAULT_DEBOUNCE_MS;
    uint8_t button_mode = BUTTON_MODE_STANDARD;
    volatile uint16_t debounce_blackout_remaining_ms = 0; //starts at debounce_period_ms, and counts down to zero
    volatile uint8_t button_state = 0;
    volatile uint8_t change_flag = 0; //set when the button state changes, unless a callback is provided
    volatile uint8_t last_raw_pin_state = 0;
    void (*callback_on_toggle)() = nullptr;
    void (*callback_on_press)() = nullptr;
    void (*callback_on_release)() = nullptr;
};





#endif //digital_in_h