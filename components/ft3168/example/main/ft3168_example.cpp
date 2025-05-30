#include <chrono>
#include <sdkconfig.h>
#include <vector>

#include "ft3168.hpp"
#include "i2c.hpp"
#include "task.hpp"

using namespace std::chrono_literals;

extern "C" void app_main(void) {
  {
    fmt::print("Starting ft3168 example\n");
    //! [ft3168 example]
    // make the I2C that we'll use to communicate
    espp::I2c i2c({
        .port = I2C_NUM_0,
        .sda_io_num = (gpio_num_t)CONFIG_EXAMPLE_I2C_SDA_GPIO,
        .scl_io_num = (gpio_num_t)CONFIG_EXAMPLE_I2C_SCL_GPIO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .timeout_ms = 20, // 10ms default was too short
    });
    // now make the ft3168 which decodes the data
    espp::Ft3168 ft3168({.write = std::bind(&espp::I2c::write, &i2c, std::placeholders::_1,
                                            std::placeholders::_2, std::placeholders::_3),
                         .read_register = std::bind(&espp::I2c::read_at_register, &i2c,
                                                    std::placeholders::_1, std::placeholders::_2,
                                                    std::placeholders::_3, std::placeholders::_4),
                         .log_level = espp::Logger::Verbosity::DEBUG});
    // and finally, make the task to periodically poll the ft3168 and print
    // the state
    auto task_fn = [&ft3168](std::mutex &m, std::condition_variable &cv) {
      std::error_code ec;
      // get the state
      uint8_t num_touch_points = 0;
      uint16_t x = 0, y = 0;
      ft3168.get_touch_point(&num_touch_points, &x, &y, ec);
      if (ec) {
        fmt::print("Could not get touch point\n");
        return false;
      }
      fmt::print("num_touch_points: {}, x: {}, y: {}\n", num_touch_points, x, y);
      // NOTE: sleeping in this way allows the sleep to exit early when the
      // task is being stopped / destroyed
      {
        std::unique_lock<std::mutex> lk(m);
        cv.wait_for(lk, 500ms);
      }
      return false; // don't stop the task
    };
    auto task = espp::Task({.callback = task_fn,
                            .task_config = {.name = "Ft3168 Task"},
                            .log_level = espp::Logger::Verbosity::WARN});
    task.start();
    //! [ft3168 example]
    while (true) {
      std::this_thread::sleep_for(100ms);
    }
  }

  fmt::print("Ft3168 example complete!\n");

  while (true) {
    std::this_thread::sleep_for(1s);
  }
}
