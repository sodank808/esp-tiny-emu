#include <chrono>
#include <sdkconfig.h>
#include <vector>

#include "tca9554.hpp"
#include "i2c.hpp"
#include "task.hpp"

using namespace std::chrono_literals;

extern "C" void app_main(void) {

  fmt::print("Starting tca9554 example\n");
  //! [tca9554 example]
  // make the I2C that we'll use to communicate
  espp::I2c i2c({
      .port = I2C_NUM_1,
      .sda_io_num = (gpio_num_t)CONFIG_EXAMPLE_I2C_SDA_GPIO,
      .scl_io_num = (gpio_num_t)CONFIG_EXAMPLE_I2C_SCL_GPIO,
  });
  // now make the tca9554 which handles GPIO
  espp::Tca9554 tca9554(
      {
        .address_pins = 0b000,
        .direction_mask = 0b11111000,
        .polarity_mask = 0b00000000,
       .write = std::bind(&espp::I2c::write, &i2c, std::placeholders::_1, std::placeholders::_2,
                          std::placeholders::_3),
       .write_then_read =
           std::bind(&espp::I2c::write_read, &i2c, std::placeholders::_1, std::placeholders::_2,
                     std::placeholders::_3, std::placeholders::_4, std::placeholders::_5),
       .log_level = espp::Logger::Verbosity::WARN});
  std::error_code ec;
  tca9554.initialize(ec); // Initialized separately from the constructor.
  if (ec) {
    fmt::print("tca9554 initialization failed: {}\n", ec.message());
    return;
  }
  tca9554.output(0b000, ec);
  if (ec) {
    fmt::print("tca9554 write output failed: {}\n", ec.message());
    return;
  }

  std::this_thread::sleep_for(20ms);

  tca9554.output(0b111, ec);
  if (ec) {
    fmt::print("tca9554 write output failed: {}\n", ec.message());
    return;
  }

  auto pins = tca9554.get_pins(ec);
  if (ec) {
    fmt::print("tca9554 get pins failed: {}\n", ec.message());
    return;
  }
  fmt::print("pins: 0b{:08b}\n", pins);
  fmt::print("Tca9554 example complete!\n");

  while (true) {
    std::this_thread::sleep_for(1s);
  }
}
