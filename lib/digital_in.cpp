#include <iterator>
#include <stdint.h>
#include "core_pins.h"
#include "digital_in.hpp"

Button::Button(){};

uint8_t Button::num_registered_buttons = 0;
Button* Button::registered_buttons[MAX_NUM_BUTTONS] = {nullptr, nullptr};
ButtonKilohertzPlugin Button::kilohertz_plugin;

uint8_t Button::read_state(){
  return button_state;
}

uint8_t Button::has_changed(){
  if(change_flag){
    change_flag = 0;
    return 1; //clear the change flag
  }else{
    return 0;
  }
}

void Button::set_callback_on_toggle(void (*callback_function)()){
  callback_on_toggle = callback_function;
}

void Button::set_callback_on_press(void (*callback_function)()){
  callback_on_press = callback_function;
}

void Button::set_callback_on_release(void (*callback_function)()){
  callback_on_release = callback_function;
}

void Button::set_debounce_ms(uint16_t debounce_ms){
  debounce_period_ms = debounce_ms;
}

void Button::set_mode(uint8_t button_mode){
  this->button_mode = button_mode;
}

void Button::set_invert(){
  if(invert == BUTTON_NON_INVERTED){
    button_state ^= 1; //we need to invert this so as not to trigger a false change
  }
  invert = BUTTON_INVERTED;
}

void Button::clear_invert(){
  if(invert == BUTTON_INVERTED){
    button_state^=1;
  }
  invert = BUTTON_NON_INVERTED;
}

void Button::begin(uint8_t pin){
  begin(pin, INPUT_PULLUP);
}

void Button::begin(uint8_t pin, uint8_t pin_mode){
  input_pin = pin;
  pinMode(input_pin, pin_mode); //DEFAULT TO PULLUP FOR NOW
  last_raw_pin_state = digitalReadFast(input_pin);
  registered_buttons[num_registered_buttons] = this;
  num_registered_buttons ++;
  if(num_registered_buttons == 1){ //this is the first button, so we need to start up the kilohertz plugin
    kilohertz_plugin.begin();
  }
}

void Button::on_frame(){
  if(debounce_blackout_remaining_ms > 0){ // do nothing if the button has changed within the debounce blackout period
    debounce_blackout_remaining_ms --;
  }else{
    uint8_t raw_pin_state = digitalReadFast(input_pin);
    if(raw_pin_state != last_raw_pin_state){ //input pin has changed
      uint8_t pin_state = raw_pin_state ^ invert;
      if(button_mode == BUTTON_MODE_STANDARD){ //
        button_state = pin_state;
        change_flag = BUTTON_CHANGED;
      }else if(button_mode == BUTTON_MODE_TOGGLE){
        if(pin_state == 1){ //only change button state on low-to-high transition
          button_state ^= 1;
          change_flag = BUTTON_CHANGED;
        }
      }
      last_raw_pin_state = raw_pin_state;
      debounce_blackout_remaining_ms = debounce_period_ms;
    }
  }

  if(change_flag){ //run callbacks
    if((button_state == BUTTON_STATE_PRESSED) && (callback_on_press != nullptr)){
      callback_on_press();
      change_flag = 0;
    }else if((button_state == BUTTON_STATE_RELEASED) && (callback_on_release != nullptr)){
      callback_on_release();
      change_flag = 0;      
    }
    if((button_mode == BUTTON_MODE_TOGGLE) && (callback_on_toggle != nullptr)){
      callback_on_toggle();
      change_flag = 0;    
    }
  }
}


ButtonKilohertzPlugin::ButtonKilohertzPlugin(){};

void ButtonKilohertzPlugin::begin(){
  register_plugin(PLUGIN_KILOHERTZ);
}

void ButtonKilohertzPlugin::run(){
  for(uint8_t button_index = 0; button_index < Button::num_registered_buttons; button_index++){
    Button::registered_buttons[button_index]->on_frame();
  }
}