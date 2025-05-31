#pragma once

#include <functional>

#include "base_peripheral.hpp"

namespace espp {
/**
 * Class for communicating with and controlling a TCA9554 GPIO expander
 * Datasheet here:
 * https://www.ti.com/lit/ds/symlink/tca9554.pdf
 *
 * \section tca9554_ex1 TCA9554 Example
 * \snippet tca9554_example.cpp tca9554 example
 */
class Tca9554 : public BasePeripheral<> {
public:
  static constexpr uint8_t DEFAULT_ADDRESS = 0x20; ///< Supports 3-bit address pins.

  /**
   * @brief Configuration information for the Tca9554.
   */
  struct Config {
    uint8_t address_pins = 0x00; ///< Address pins customize slave address
    uint8_t direction_mask = 0x00; ///< Direction mask of the pins
    uint8_t polarity_mask = 0x00; ///< Polarity mask of the pins
    BasePeripheral::write_fn write; ///< Function to write to the device.
    BasePeripheral::write_then_read_fn
        write_then_read;   ///< Function to write then read from the device.
    bool auto_init = true; ///< Automatically initialize the device.
    Logger::Verbosity log_level{Logger::Verbosity::WARN}; ///< Log verbosity for the component.
  };

  /**
   * @brief Construct the Tca9554. Will call initialize() if auto_init is true.
   * @param config Config structure for configuring the Tca9554
   */
  explicit Tca9554(const Config &config)
      : BasePeripheral({.address = (uint8_t)(DEFAULT_ADDRESS + (config.address_pins & 7)),
                        .write = config.write,
                        .write_then_read = config.write_then_read},
                       "Tca9554", config.log_level)
      , config_(config) {
    if (config.auto_init) {
      std::error_code ec;
      initialize(ec);
      if (ec) {
        logger_.error("Failed to initialize TCA9554: {}", ec.message());
      }
    }
  }

  /**
   * @brief Initialize the component class.
   * @param ec Error code to set if an error occurs.
   */
  void initialize(std::error_code &ec) { init(config_, ec); }

  /**
   * @brief Read the pin values on the provided port.
   * @param ec Error code to set if an error occurs.
   * @return The pin values as an 8 bit mask.
   */
  uint8_t get_pins(std::error_code &ec) {
    auto addr = Registers::INPUT_PORT;
    return read_u8_from_register((uint8_t)addr, ec);
  }

  /**
   * @brief Write the pin values on the output port.
   * @param value The pin values to apply.
   * @param ec Error code to set if an error occurs.
   * @note This will overwrite any previous pin values on the port for all
   *       output pins on the port.
   */
  void output(uint8_t value, std::error_code &ec) {
    auto addr = Registers::OUTPUT_PORT;
    write_u8_to_register((uint8_t)addr, value, ec);
  }

  /**
   * @brief Clear the pin values on the output port according to the provided mask.
   * @details Reads the current pin values and clears any bits set in the mask.
   * @param mask The pin values as an 8 bit mask to clear.
   * @param ec Error code to set if an error occurs.
   */
  void clear_pins(uint8_t mask, std::error_code &ec) {
    auto addr = Registers::OUTPUT_PORT;
    std::lock_guard<std::recursive_mutex> lock(base_mutex_);
    auto data = read_u8_from_register((uint8_t)addr, ec);
    if (ec)
      return;
    data &= ~mask;
    write_u8_to_register((uint8_t)addr, data, ec);
  }

  /**
   * @brief Set the pin values on the output port according to the provided mask.
   * @brief Reads the current pin values and sets any bits set in the mask.
   * @param mask The pin values as an 8 bit mask to set.
   * @param ec Error code to set if an error occurs.
   */
  void set_pins(uint8_t mask, std::error_code &ec) {
    auto addr = Registers::OUTPUT_PORT;
    std::lock_guard<std::recursive_mutex> lock(base_mutex_);
    auto data = read_u8_from_register((uint8_t)addr, ec);
    if (ec)
      return;
    data |= mask;
    write_u8_to_register((uint8_t)addr, data, ec);
  }

  /**
   * @brief Read the output pin values on the output port.
   * @param ec Error code to set if an error occurs.
   * @return The pin values as an 8 bit mask.
   */
  uint8_t get_output(std::error_code &ec) {
    auto addr = Registers::OUTPUT_PORT;
    return read_u8_from_register((uint8_t)addr, ec);
  }

  /**
   * @brief Set the i/o polarity for the pins according to mask.
   * @param mask The mask indicating polarity (1 = invert, 0 = no inversion)
   * @param ec Error code to set if an error occurs.
   */
  void set_polarity(uint8_t mask, std::error_code &ec) {
    logger_.debug("Setting polarity for Port to {}", mask);
    auto addr = Registers::POLARITY_INVERSION;
    write_u8_to_register((uint8_t)addr, mask, ec);
  }

  /**
   * @brief Set the i/o direction for the pins according to mask.
   * @param mask The mask indicating direction (1 = input, 0 = output)
   * @param ec Error code to set if an error occurs.
   */
  void set_direction(uint8_t mask, std::error_code &ec) {
    logger_.debug("Setting direction for Port to {}", mask);
    auto addr = Registers::CONFIGURATION;
    write_u8_to_register((uint8_t)addr, mask, ec);
  }

protected:
  /**
   * @brief Register map for the TCA9554
   */
  enum class Registers : uint8_t {
    INPUT_PORT = 0x00,    ///< Read expander input values
    OUTPUT_PORT = 0x01,    ///< Write expander output values
    POLARITY_INVERSION = 0x02, ///< Invert polarity of input pins
    CONFIGURATION = 0x03, ///< Configuration register
  };

  void init(const Config &config, std::error_code &ec) {
    std::lock_guard<std::recursive_mutex> lock(base_mutex_);
    set_direction(config.direction_mask, ec);
    if (ec)
      return;
    set_polarity(config.polarity_mask, ec);
  }

  Config config_;
};
} // namespace espp
