#include <stdint.h>
#include "core.hpp"
/*
Digital Input Module of the StepDance Control System

This module is responsible for providing an interface to digital inputs, e.g. as buttons.

[More Details to be Added]

A part of the Mixing Metaphors Project
(c) 2025 Ilan Moyer, Jennifer Jacobs, Devon Frost, Emilie Yu

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
/** \cond */
 /**
   * This struct will be hidden from Doxygen documentation.
   */ 
class ButtonKilohertzPlugin : public Plugin{ //we'll set up to run on every kilohertz frame
  public:
    ButtonKilohertzPlugin();
    void begin();
  protected:
    void run();
};
/** \endcond */

/**/
/**
 * @brief Button class for handling digital input buttons.
 * @ingroup inputs
 * 
 * The Button class provides an interface for reading and managing digital input buttons. It supports debouncing, state change detection, and callback functions for button events such as press, release, and toggle. Buttons can be configured in standard or toggle mode, and their state can be inverted if needed.
 * Here's an example of how to instantiate and configure a Button:
 * @snippet snippets.cpp Button
 */
class Button : public Plugin{
  public:
  /**
   * @brief Default constructor for Button. Does not initialize board hardware. Call begin() to set up the button pin.
   **/
    Button();
    /** 
     * @brief Reads the debounced state of the button.
     * @return uint8_t Returns BUTTON_STATE_PRESSED or BUTTON_STATE_RELEASED.
     **/
    /**
     * @brief Initializes the Button with the specified pin and mode.
     * @param pin The digital pin number where the button is connected (e.g. )
     */
    void begin(uint8_t pin); //defaults to INPUT_PULLUP
    /** 
     * @brief Initializes the Button with the specified pin and mode.
     * @param pin The digital pin number where the button is connected (e.g. )
     * @param mode The pin mode, e.g. INPUT, INPUT_PULLUP, etc.
     **/
    /**
     * @brief Reads the state of the button.
     * @return uint8_t Returns BUTTON_STATE_PRESSED or BUTTON_STATE_RELEASED.
     */
    uint8_t read(); //returns button state
    /** 
     * @brief Reads the raw state of the input pin without debouncing.
     * @return uint8_t Returns the raw pin state.
     **/
    uint8_t read_raw(); //returns the asynchronous raw state of the input pin
    /** 
     * @brief Checks if the button state has changed since the last read.
     * @return uint8_t Returns 1 if the button state has changed, otherwise returns 0.
     **/
    uint8_t has_changed(); //returns 1 if the button state has changed
    /** 
     * @brief Sets a callback function to be called when the button state toggles-either pressed or released.
     * @param callback_function Pointer to the callback function. 
     **/
    void set_callback_on_toggle(void (*callback_function)());
    /** 
     * @brief Sets a callback function to be called when the button is pressed.
     * @param callback_function Pointer to the callback function. 
     **/
    void set_callback_on_press(void (*callback_function)());
    /** 
     * @brief Sets a callback function to be called when the button is released.
     * @param callback_function Pointer to the callback function. 
     **/
    void set_callback_on_release(void (*callback_function)());
    /** 
     * @brief Sets the debounce period for the button. Debouncing refers to checking the button state over a short period to avoid false triggering due to mechanical noise.
     * @param debounce_ms Debounce period in milliseconds.
     **/
    void set_debounce_ms(uint16_t debounce_ms);
    /** 
     * @brief Sets the button mode to either standard or toggle. Toggle mode triggers the button state only on presses. Standard will trigger callbacks on both press and release.
     * @param button_mode BUTTON_MODE_STANDARD or BUTTON_MODE_TOGGLE.
     **/
    void set_mode(uint8_t button_mode); //standard, or toggle
    /** 
     * @brief Inverts the button state logic. When inverted, a pressed button reads as released and vice versa. Not sure why you want this, but it's here if you need it.
     **/
    void set_invert(); //inverts the button state
    /** 
     * @brief Clears the invert flag, restoring normal button state logic.
     **/
    void clear_invert(); //clears the invert button mode

    /** \cond
     * These properties and methods will be hidden from Doxygen documentation.
     */
    
    static Button* registered_buttons[MAX_NUM_BUTTONS]; //stores all registered buttons
    static uint8_t num_registered_buttons;
    static ButtonKilohertzPlugin kilohertz_plugin; // runs in the kilohertz frame
    void begin(uint8_t pin, uint8_t mode);
    void on_frame(); //gets called at each kilohertz frame
    void enroll(RPC *rpc, const String& instance_name);
    /** \endcond */
    
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