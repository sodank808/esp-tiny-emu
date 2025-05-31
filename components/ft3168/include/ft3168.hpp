#pragma once

#include <functional>

#include "base_peripheral.hpp"

namespace espp {
/// @brief The FT3168 touch controller.
/// @details This class is used to communicate with the FT3168 touch controller.
///
/// \section ft3168_ex1 Example
/// \snippet ft3168_example.cpp ft3168 example
class Ft3168 : public BasePeripheral<> {
public:
  /// @brief The default I2C address for the FT3168.
  static constexpr uint8_t DEFAULT_ADDRESS = (0x38);

  /// @brief The power modes
  enum class PowerMode : uint8_t {
    POWER_ACTIVE = 0b00,
    POWER_SLEEP = 0b01,
    POWER_STANDBY = 0b10,
    POWER_HIBERNATE = 0b11,
  };

  /// @brief The gesture that was detected.
  enum class Gesture : uint8_t {
    NONE = 0x00,
    SWIPE_LEFT = 0x20,
    SWIPE_RIGHT = 0x21,
    SWIPE_UP = 0x22,
    SWIPE_DOWN = 0x23,
    DOUBLE_CLICK = 0x24,
  };
  

  /// @brief The configuration for the FT3168.
  struct Config {
    BasePeripheral::write_fn write; ///< The function to write data to the I2C bus.
    BasePeripheral::read_register_fn
        read_register; ///< The function to write then read data from the I2C bus.
    espp::Logger::Verbosity log_level{espp::Logger::Verbosity::WARN}; ///< The log level.
  };

  /// @brief Construct a new FT3168.
  /// @param config The configuration for the FT3168.
  explicit Ft3168(const Config &config)
      : BasePeripheral({.address = DEFAULT_ADDRESS,
                        .write = config.write,
                        .read_register = config.read_register},
                       "Ft3168", config.log_level) {
    std::error_code ec;
    init(ec);
    if (ec) {
      logger_.error("Failed to initialize FT3168");
    }
  }

  /// @brief Get the device ID.
  /// @param ec The error code if the function fails.
  /// @return The device ID.
  uint8_t get_device_id(std::error_code &ec) {
    return read_u8_from_register((uint8_t)Registers::DEVICE_ID, ec);
  }

  /// @brief Set the power mode.
  /// @param mode The power mode.
  /// @param ec The error code if the function fails.
  void set_power_mode(PowerMode mode, std::error_code &ec) {
    write_u8_to_register((uint8_t)Registers::POWER_MODE, (uint8_t)mode, ec);
  }

  /// @brief Set the proximity sensing mode.
  /// @param on Whether to enable proximity sensing.
  /// @param ec The error code if the function fails.
  void set_proximity_sensing(bool on, std::error_code &ec) {
    write_u8_to_register((uint8_t)Registers::PROXIMITY_SENSING_MODE, (uint8_t)(on ? 0x01 : 0x00), ec);
  }

  /// @brief Set the gesture identification mode.
  /// @param on Whether to enable gesture identification mode.
  /// @note When set the gesture ID can be read from the GESTURE_ID register.
  /// @param ec The error code if the function fails.
  void set_gesture_mode(bool on, std::error_code &ec) {
    write_u8_to_register((uint8_t)Registers::GESTURE_MODE, (uint8_t)(on ? 0x01 : 0x00), ec);
  }

  /// @brief Get the identified gesture.
  /// @param ec The error code if the function fails.
  /// @return The identified gesture.
  Gesture get_gesture(std::error_code &ec) {
    return (Gesture)read_u8_from_register((uint8_t)Registers::GESTURE_ID, ec);
  }

  /// @brief Get the number of touch points.
  /// @param ec The error code if the function fails.
  /// @return The number of touch points.
  uint8_t get_num_touch_points(std::error_code &ec) {
    return read_u8_from_register((uint8_t)Registers::TOUCH_POINTS, ec);
  }

  /// @brief Get the touch point.
  /// @param num_touch_points The number of touch points.
  /// @param x The x coordinate of the touch point.
  /// @param y The y coordinate of the touch point.
  /// @param ec The error code if the function fails.
  void get_touch_point(uint8_t *num_touch_points, uint16_t *x, uint16_t *y, std::error_code &ec) {
    std::lock_guard<std::recursive_mutex> lock(base_mutex_);
    auto tp = get_num_touch_points(ec);
    if (ec) {
      return;
    }
    *num_touch_points = tp;
    // TODO: read all touch points
    if (*num_touch_points != 0) {
      uint8_t data[4];
      read_many_from_register((uint8_t)Registers::TOUCH1_XH, data, 4, ec);
      if (ec) {
        return;
      }
      *x = ((data[0] & 0x0f) << 8) + data[1];
      *y = ((data[2] & 0x0f) << 8) + data[3];
      logger_.info("Got touch ({}, {})", *x, *y);
    }
  }

  /// @brief Get the gesture that was detected.
  /// @param ec The error code if the function fails.
  /// @return The gesture that was detected.
  Gesture read_gesture(std::error_code &ec) {
    return (Gesture)read_u8_from_register((uint8_t)Registers::GESTURE_ID, ec);
  }

protected:
  void init(std::error_code &ec) {
    std::lock_guard<std::recursive_mutex> lock(base_mutex_);
  #if 0 // Arduino code has this, but it's not clear if it's needed
    // Enter monitor mode
    write_u8_to_register((uint8_t)Registers::POWER_MODE, 1, ec);
    if (ec)
      return;
  #endif
    auto device_id = read_u8_from_register((uint8_t)Registers::DEVICE_ID, ec);
    if (ec)
      return;
    logger_.info("Device ID: {}", device_id);

    auto power_mode = read_u8_from_register((uint8_t)Registers::POWER_MODE, ec);
    if (ec)
      return;
    logger_.info("Power mode: {}", power_mode);

    auto proximity_sensing_mode = read_u8_from_register((uint8_t)Registers::PROXIMITY_SENSING_MODE, ec);
    if (ec)
      return;
    logger_.info("Proximity sensing mode: {}", proximity_sensing_mode);

    auto gesture_mode = read_u8_from_register((uint8_t)Registers::GESTURE_MODE, ec);
    if (ec)
      return;
    logger_.info("Gesture mode: {}", gesture_mode);
  }

  enum class Registers : uint8_t {
    TOUCH_POINTS = 0x02,

    TOUCH1_EV_FLAG = 0x03,
    TOUCH1_XH = 0x03,
    TOUCH1_XL = 0x04,
    TOUCH1_YH = 0x05,
    TOUCH1_YL = 0x06,

    TOUCH2_EV_FLAG = 0x09,
    TOUCH2_XH = 0x09,
    TOUCH2_XL = 0x0A,
    TOUCH2_YH = 0x0B,
    TOUCH2_YL = 0x0C,

    DEVICE_ID = 0xA0,
    POWER_MODE = 0xA5,
    PROXIMITY_SENSING_MODE = 0xB0,
    GESTURE_MODE = 0xD0,
    GESTURE_ID = 0xD3,
  };
};
} // namespace espp
