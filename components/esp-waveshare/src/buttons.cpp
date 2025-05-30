#include "esp-waveshare.hpp"

using namespace espp;

////////////////////////
// Button Functions   //
////////////////////////

bool EspWaveshare::initialize_boot_button(const EspWaveshare::button_callback_t &callback) {
  logger_.info("Initializing boot button");

  // save the callback
  boot_button_callback_ = callback;

  // configure the button
  if (!boot_button_initialized_) {
    interrupts_.add_interrupt(boot_button_interrupt_pin_);
  }
  boot_button_initialized_ = true;
  return true;
}

bool EspWaveshare::boot_button_state() const {
  if (!boot_button_initialized_) {
    return false;
  }
  return interrupts_.is_active(boot_button_interrupt_pin_);
}

bool EspWaveshare::initialize_mute_button(const EspWaveshare::button_callback_t &callback) {
  logger_.info("Initializing mute button");

  // save the callback
  mute_button_callback_ = callback;

  // configure the button
  if (!mute_button_initialized_) {
    interrupts_.add_interrupt(mute_button_interrupt_pin_);
  }
  mute_button_initialized_ = true;
  return true;
}

bool EspWaveshare::mute_button_state() const {
  if (!mute_button_initialized_) {
    return false;
  }
  return interrupts_.is_active(mute_button_interrupt_pin_);
}
