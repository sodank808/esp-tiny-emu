#include "tiny-emu.hpp"

TinyEmu::TinyEmu() : espp::BaseComponent("TinyEmu") {
  detect();
}

TinyEmu::Version TinyEmu::version() const {
  return version_;
}

void TinyEmu::detect() {
  bool version0_found = external_i2c_.probe_device(version0::input_address);
  bool version1_found = external_i2c_.probe_device(version1::input_address);
  if (version1_found) {
    // Version 1
    version_ = TinyEmu::Version::V1;
  } else if (version0_found) {
    // Version 0
    version_ = TinyEmu::Version::V0;
  } else {
    logger_.warn("No box-emu hardware detected");
    logger_.warn("\tProbed for MCP23x17 (version0) at 0x{:02x}", version0::input_address);
    logger_.warn("\tProbed for AW9523 (version1) at 0x{:02x}", version1::input_address);
    // No box detected
    version_ = TinyEmu::Version::UNKNOWN;
    return;
  }
  logger_.info("version {}", version_);
}

espp::I2c &TinyEmu::internal_i2c() {
  return Bsp::get().internal_i2c();
}

espp::I2c &TinyEmu::external_i2c() {
  return external_i2c_;
}

bool TinyEmu::initialize_bsp() {
  logger_.info("Initializing underlying BSP");
  auto &bsp = Bsp::get();
  // initialize the sound
  if (!bsp.initialize_sound()) {
    logger_.error("Failed to initialize sound!");
    return false;
  }
  // initialize the LCD
  if (!bsp.initialize_lcd()) {
    logger_.error("Failed to initialize LCD!");
    return false;
  }
  static constexpr size_t pixel_buffer_size = Bsp::lcd_width() * num_rows_in_framebuffer;
  // initialize the LVGL display for the esp-box
  if (!bsp.initialize_display(pixel_buffer_size)) {
    logger_.error("Failed to initialize display!");
    return false;
  }
  // initialize the touchpad
  if (!bsp.initialize_touch()) {
    logger_.error("Failed to initialize touchpad!");
    return false;
  }

  // initialize the mute button to broadcast the mute state
  logger_.info("Initializing mute button");
  mute_button_ = std::make_shared<espp::Button>(espp::Button::Config{
      .name = "mute button",
      .interrupt_config =
          {
              .gpio_num = Bsp::get_mute_pin(),
              .callback =
                  [](const espp::Interrupt::Event &event) {
                    Bsp::get().mute(event.active);
                    // simply publish that the mute button was presssed
                    espp::EventManager::get().publish(mute_button_topic, {});
                  },
              .active_level = espp::Interrupt::ActiveLevel::LOW,
              .interrupt_type = espp::Interrupt::Type::ANY_EDGE,
              .pullup_enabled = true,
              .pulldown_enabled = false,
          },
      .task_config =
          {
              .name = "mute button task",
              .stack_size_bytes = 4 * 1024,
              .priority = 5,
          },
      .log_level = espp::Logger::Verbosity::WARN,
  });

  // update the mute state (since it's a flip-flop and may have been set if we
  // restarted without power loss)
  Bsp::get().mute(mute_button_->is_pressed());
  return true;
}

/////////////////////////////////////////////////////////////////////////////
// uSD Card
/////////////////////////////////////////////////////////////////////////////

bool TinyEmu::initialize_sdcard() {
  if (sdcard_) {
    logger_.error("SD card already initialized!");
    return false;
  }

  logger_.info("Initializing SD card");

  esp_err_t ret;
  // Options for mounting the filesystem. If format_if_mount_failed is set to
  // true, SD card will be partitioned and formatted in case when mounting
  // fails.
  esp_vfs_fat_sdmmc_mount_config_t mount_config;
  memset(&mount_config, 0, sizeof(mount_config));
  mount_config.format_if_mount_failed = false;
  mount_config.max_files = 5;
  mount_config.allocation_unit_size = 2 * 1024;

  // Use settings defined above to initialize SD card and mount FAT filesystem.
  // Note: esp_vfs_fat_sdmmc/sdspi_mount is all-in-one convenience functions.
  // Please check its source code and implement error recovery when developing
  // production applications.
  logger_.debug("Using SPI peripheral");

  // By default, SD card frequency is initialized to SDMMC_FREQ_DEFAULT (20MHz)
  // For setting a specific frequency, use host.max_freq_khz (range 400kHz - 20MHz for SDSPI)
  // Example: for fixed frequency of 10MHz, use host.max_freq_khz = 10000;
  sdmmc_host_t host = SDSPI_HOST_DEFAULT();
  host.slot = sdcard_spi_num;
  // host.max_freq_khz = 20 * 1000;

  spi_bus_config_t bus_cfg;
  memset(&bus_cfg, 0, sizeof(bus_cfg));
  bus_cfg.mosi_io_num = sdcard_mosi;
  bus_cfg.miso_io_num = sdcard_miso;
  bus_cfg.sclk_io_num = sdcard_sclk;
  bus_cfg.quadwp_io_num = -1;
  bus_cfg.quadhd_io_num = -1;
  bus_cfg.max_transfer_sz = 4096;
  spi_host_device_t host_id = (spi_host_device_t)host.slot;
  ret = spi_bus_initialize(host_id, &bus_cfg, SDSPI_DEFAULT_DMA);
  if (ret != ESP_OK) {
    logger_.error("Failed to initialize bus.");
    return false;
  }

  // This initializes the slot without card detect (CD) and write protect (WP) signals.
  // Modify slot_config.gpio_cd and slot_config.gpio_wp if your board has these signals.
  sdspi_device_config_t slot_config = SDSPI_DEVICE_CONFIG_DEFAULT();
  slot_config.gpio_cs = sdcard_cs;
  slot_config.host_id = host_id;

  logger_.debug("Mounting filesystem");
  ret = esp_vfs_fat_sdspi_mount(mount_point, &host, &slot_config, &mount_config, &sdcard_);

  if (ret != ESP_OK) {
    if (ret == ESP_FAIL) {
      logger_.error("Failed to mount filesystem. "
                    "If you want the card to be formatted, set the CONFIG_EXAMPLE_FORMAT_IF_MOUNT_FAILED menuconfig option.");
      return false;
    } else {
      logger_.error("Failed to initialize the card ({}). "
                    "Make sure SD card lines have pull-up resistors in place.", esp_err_to_name(ret));
      return false;
    }
    return false;
  }

  logger_.info("Filesystem mounted");

  // Card has been initialized, print its properties
  sdmmc_card_print_info(stdout, sdcard_);

  return true;
}

sdmmc_card_t *TinyEmu::sdcard() const {
  return sdcard_;
}

/////////////////////////////////////////////////////////////////////////////
// Memory
/////////////////////////////////////////////////////////////////////////////

static constexpr size_t memory_size = 4*1024*1024;

extern "C" uint8_t *osd_getromdata() {
  auto &emu = TinyEmu::get();
  return emu.romdata();
}

bool TinyEmu::initialize_memory() {
  if (romdata_) {
    logger_.error("ROM already initialized!");
    return false;
  }

  logger_.info("Initializing memory (romdata)");
  // allocate memory for the ROM and make sure it's on the SPIRAM
  romdata_ = (uint8_t*)heap_caps_malloc(memory_size, MALLOC_CAP_8BIT | MALLOC_CAP_SPIRAM);
  if (romdata_ == nullptr) {
    logger_.error("Couldn't allocate memory for ROM!");
    return false;
  }

  return true;
}

void TinyEmu::deinitialize_memory() {
  if (romdata_) {
    logger_.info("Deinitializing memory (romdata)");
    free(romdata_);
    romdata_ = nullptr;
  }
}

size_t TinyEmu::copy_file_to_romdata(const std::string& filename) {
  // load the file data and iteratively copy it over
  std::ifstream romfile(filename, std::ios::binary | std::ios::ate); //open file at end
  if (!romfile.is_open()) {
    logger_.error("ROM file does not exist");
    return 0;
  }
  size_t filesize = romfile.tellg(); // get size from current file pointer location;
  romfile.seekg(0, std::ios::beg); //reset file pointer to beginning;
  romfile.read((char*)(romdata_), filesize);
  romfile.close();

  return filesize;
}

uint8_t *TinyEmu::romdata() const {
  return romdata_;
}

/////////////////////////////////////////////////////////////////////////////
// Gamepad
/////////////////////////////////////////////////////////////////////////////

bool TinyEmu::initialize_gamepad() {
  logger_.info("Initializing gamepad");
  if (version_ == TinyEmu::Version::V0) {
    auto raw_input = new version0::InputType(
                                             std::make_shared<version0::InputDriver>(version0::InputDriver::Config{
                                                 .device_address = version0::input_address,
                                                 .port_0_direction_mask = version0::PORT_0_DIRECTION_MASK,
                                                 .port_0_interrupt_mask = version0::PORT_0_INTERRUPT_MASK,
                                                 .port_1_direction_mask = version0::PORT_1_DIRECTION_MASK,
                                                 .port_1_interrupt_mask = version0::PORT_1_INTERRUPT_MASK,
                                                 .write = std::bind(&espp::I2c::write, &external_i2c_, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3),
                                                 .read_register = std::bind(&espp::I2c::read_at_register, &external_i2c_, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3, std::placeholders::_4),
                                                 .log_level = espp::Logger::Verbosity::WARN
                                               })
                                             );
    input_.reset(raw_input);
  } else if (version_ == TinyEmu::Version::V1) {
    auto raw_input = new version1::InputType(
                                             std::make_shared<version1::InputDriver>(version1::InputDriver::Config{
                                                 .device_address = version1::input_address,
                                                 .port_0_direction_mask = version1::PORT_0_DIRECTION_MASK,
                                                 .port_0_interrupt_mask = version1::PORT_0_INTERRUPT_MASK,
                                                 .port_1_direction_mask = version1::PORT_1_DIRECTION_MASK,
                                                 .port_1_interrupt_mask = version1::PORT_1_INTERRUPT_MASK,
                                                 .write = std::bind(&espp::I2c::write, &external_i2c_, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3),
                                                 .write_then_read = std::bind(&espp::I2c::write_read, &external_i2c_, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3, std::placeholders::_4, std::placeholders::_5),
                                                 .log_level = espp::Logger::Verbosity::WARN
                                               })
                                             );
    input_.reset(raw_input);
  } else {
    return false;
  }

  // now initialize the keypad driver
  keypad_ = std::make_shared<espp::KeypadInput>(espp::KeypadInput::Config{
      .read = std::bind(&TinyEmu::keypad_read, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3, std::placeholders::_4, std::placeholders::_5, std::placeholders::_6),
      .log_level = espp::Logger::Verbosity::WARN
    });

  // now initialize the input timer
  input_timer_ = std::make_shared<espp::HighResolutionTimer>(espp::HighResolutionTimer::Config{
      .name = "Input timer",
      .callback = [this]() {
        update_gamepad_state();
      }});
  uint64_t period_us = 30 * 1000;
  input_timer_->periodic(period_us);

  return true;
}

GamepadState TinyEmu::gamepad_state() {
  std::lock_guard<std::recursive_mutex> lock(gamepad_state_mutex_);
  return gamepad_state_;
}

bool TinyEmu::update_gamepad_state() {
  if (!input_) {
    return false;
  }
  if (!can_read_gamepad_) {
    return false;
  }
  std::error_code ec;
  auto pins = input_->get_pins(ec);
  if (ec) {
    logger_.error("Error reading input pins: {}", ec.message());
    can_read_gamepad_ = false;
    return false;
  }

  auto new_gamepad_state = input_->pins_to_gamepad_state(pins);
  bool changed = false;
  {
    std::lock_guard<std::recursive_mutex> lock(gamepad_state_mutex_);
    changed = gamepad_state_ != new_gamepad_state;
    gamepad_state_ = new_gamepad_state;
  }
  input_->handle_volume_pins(pins);

  return changed;
}

void TinyEmu::keypad_read(bool *up, bool *down, bool *left, bool *right, bool *enter, bool *escape) {
  std::lock_guard<std::recursive_mutex> lock(gamepad_state_mutex_);
  *up = gamepad_state_.up;
  *down = gamepad_state_.down;
  *left = gamepad_state_.left;
  *right = gamepad_state_.right;

  *enter = gamepad_state_.a || gamepad_state_.start;
  *escape = gamepad_state_.b || gamepad_state_.select;
}

std::shared_ptr<espp::KeypadInput> TinyEmu::keypad() const {
  return keypad_;
}

/////////////////////////////////////////////////////////////////////////////
// Battery
/////////////////////////////////////////////////////////////////////////////

bool TinyEmu::initialize_battery() {
  // Disable the battery for now
  return false;
}

/////////////////////////////////////////////////////////////////////////////
// Video
/////////////////////////////////////////////////////////////////////////////

bool TinyEmu::initialize_video() {
  if (video_task_) {
    logger_.error("Video task already initialized!");
    return false;
  }

  logger_.info("initializing video task");

  video_queue_ = xQueueCreate(1, sizeof(uint16_t*));
  using namespace std::placeholders;
  video_task_ = espp::Task::make_unique({
      .callback = std::bind(&TinyEmu::video_task_callback, this, _1, _2, _3),
      .task_config = {
        .name = "video task",
        .stack_size_bytes = 4*1024,
        .priority = 20,
        .core_id = 1
      },
    });
  video_task_->start();

  return true;
}

void TinyEmu::clear_screen() {
  static int buffer = 0;
  xQueueSend(video_queue_, &buffer, 10);
}

void TinyEmu::display_size(size_t width, size_t height) {
  display_width_ = width;
  display_height_ = height;
}

void TinyEmu::native_size(size_t width, size_t height, int pitch) {
  native_width_ = width;
  native_height_ = height;
  native_pitch_ = pitch == -1 ? width : pitch;
}

void TinyEmu::palette(const uint16_t *palette, size_t size) {
  palette_ = palette;
  palette_size_ = size;
}

void IRAM_ATTR TinyEmu::push_frame(const void* frame) {
  if (video_queue_ == nullptr) {
    logger_.error("video queue is null, make sure to call initialize_video() first!");
    return;
  }
  xQueueSend(video_queue_, &frame, 0);
}

VideoSetting TinyEmu::video_setting() const {
  return video_setting_;
}

void TinyEmu::video_setting(const VideoSetting setting) {
  video_setting_ = setting;
}

/////////////////////////////////////////////////////////////////////////////
// USB
/////////////////////////////////////////////////////////////////////////////

#define TUSB_DESC_TOTAL_LEN (TUD_CONFIG_DESC_LEN + TUD_MSC_DESC_LEN)

enum {
    ITF_NUM_MSC = 0,
    ITF_NUM_TOTAL
};

enum {
    EDPT_CTRL_OUT = 0x00,
    EDPT_CTRL_IN  = 0x80,

    EDPT_MSC_OUT  = 0x01,
    EDPT_MSC_IN   = 0x81,
};

static uint8_t const desc_configuration[] = {
    // Config number, interface count, string index, total length, attribute, power in mA
    TUD_CONFIG_DESCRIPTOR(1, ITF_NUM_TOTAL, 0, TUSB_DESC_TOTAL_LEN, TUSB_DESC_CONFIG_ATT_REMOTE_WAKEUP, 100),

    // Interface number, string index, EP Out & EP In address, EP size
    TUD_MSC_DESCRIPTOR(ITF_NUM_MSC, 0, EDPT_MSC_OUT, EDPT_MSC_IN, TUD_OPT_HIGH_SPEED ? 512 : 64),
};

static tusb_desc_device_t descriptor_config = {
    .bLength = sizeof(descriptor_config),
    .bDescriptorType = TUSB_DESC_DEVICE,
    .bcdUSB = 0x0200,
    .bDeviceClass = TUSB_CLASS_MISC,
    .bDeviceSubClass = MISC_SUBCLASS_COMMON,
    .bDeviceProtocol = MISC_PROTOCOL_IAD,
    .bMaxPacketSize0 = CFG_TUD_ENDPOINT0_SIZE,
    .idVendor = 0x303A, // This is Espressif VID. This needs to be changed according to Users / Customers
    .idProduct = 0x4002,
    .bcdDevice = 0x100,
    .iManufacturer = 0x01,
    .iProduct = 0x02,
    .iSerialNumber = 0x03,
    .bNumConfigurations = 0x01
};

static char const *string_desc_arr[] = {
    (const char[]) { 0x09, 0x04 },  // 0: is supported language is English (0x0409)
    "sodank808",                      // 1: Manufacturer
    "ESP-Tiny-Emu",                  // 2: Product
    "123456",                       // 3: Serials
    "Tiny-Emu uSD Card",                     // 4. MSC
};

bool TinyEmu::is_usb_enabled() const {
  return usb_enabled_;
}

bool TinyEmu::initialize_usb() {
  if (usb_enabled_) {
    logger_.error("USB MSC already initialized!");
    return false;
  }

  logger_.info("USB MSC initialization");

  // get the card from the filesystem initialization
  auto card = sdcard();
  if (!card) {
    logger_.error("No SD card found, skipping USB MSC initialization");
    return false;
  }

  logger_.debug("Deleting JTAG PHY");
  usb_del_phy(jtag_phy_);

  fmt::print("USB MSC initialization\n");
  // register the callback for the storage mount changed event.
  tinyusb_msc_sdmmc_config_t config_sdmmc = {
    .card = card,
    .callback_mount_changed = nullptr,
    .callback_premount_changed = nullptr,
    .mount_config = {
      .format_if_mount_failed = false,
      .max_files = 5,
      .allocation_unit_size = 2 * 1024, // sector size is 512 bytes, this should be between sector size and (128 * sector size). Larger means higher read/write performance and higher overhead for small files.
      .disk_status_check_enable = false, // true if you see issues or are unmounted properly; slows down I/O
    },
  };
  ESP_ERROR_CHECK(tinyusb_msc_storage_init_sdmmc(&config_sdmmc));
  // ESP_ERROR_CHECK(tinyusb_msc_register_callback(TINYUSB_MSC_EVENT_MOUNT_CHANGED, storage_mount_changed_cb));

  // initialize the tinyusb stack
  fmt::print("USB MSC initialization\n");
  tinyusb_config_t tusb_cfg;
  memset(&tusb_cfg, 0, sizeof(tusb_cfg));
  tusb_cfg.device_descriptor = &descriptor_config;
  tusb_cfg.string_descriptor = string_desc_arr;
  tusb_cfg.string_descriptor_count = sizeof(string_desc_arr) / sizeof(string_desc_arr[0]);
  tusb_cfg.external_phy = false;
  tusb_cfg.configuration_descriptor = desc_configuration;
  ESP_ERROR_CHECK(tinyusb_driver_install(&tusb_cfg));
  fmt::print("USB MSC initialization DONE\n");
  usb_enabled_ = true;

  return true;
}

bool TinyEmu::deinitialize_usb() {
  if (!usb_enabled_) {
    logger_.warn("USB MSC not initialized");
    return false;
  }
  logger_.info("USB MSC deinitialization");
  auto err = tinyusb_driver_uninstall();
  if (err != ESP_OK) {
    logger_.error("tinyusb_driver_uninstall failed: {}", esp_err_to_name(err));
    return false;
  }
  usb_enabled_ = false;
  // and reconnect the CDC port, see:
  // https://github.com/espressif/idf-extra-components/pull/229
  usb_phy_config_t phy_conf;
  memset(&phy_conf, 0, sizeof(phy_conf));
  phy_conf.controller = USB_PHY_CTRL_SERIAL_JTAG;
  usb_new_phy(&phy_conf, &jtag_phy_);
  return true;
}

/////////////////////////////////////////////////////////////////////////////
// Static Video Task:
/////////////////////////////////////////////////////////////////////////////

bool TinyEmu::has_palette() const {
  return palette_ != nullptr;
}

bool TinyEmu::is_native() const {
  return native_width_ == display_width_ && native_height_ == display_height_;
}

int TinyEmu::x_offset() const {
  return (Bsp::lcd_width()-display_width_)/2;
}

int TinyEmu::y_offset() const {
  return (Bsp::lcd_height()-display_height_)/2;
}

const uint16_t* TinyEmu::palette() const {
  return palette_;
}

bool TinyEmu::video_task_callback(std::mutex &m, std::condition_variable& cv, bool &task_notified) {
  const void *_frame_ptr;
  if (xQueueReceive(video_queue_, &_frame_ptr, portMAX_DELAY) != pdTRUE) {
    return false;
  }
  static constexpr int num_lines_to_write = num_rows_in_framebuffer;
  static auto &bsp = Bsp::get();
  static const uint16_t *vram0 = (uint16_t*)bsp.vram0();
  static const uint16_t *vram1 = (uint16_t*)bsp.vram1();
  static uint16_t vram_index = 0; // has to be static so that it persists between calls
  const int _x_offset = x_offset();
  const int _y_offset = y_offset();
  const uint16_t* _palette = palette();
  // special case: if _frame_ptr is null, then we ignore all palette, scaling,
  // etc. and simply fill the screen with 0
  if (_frame_ptr == nullptr) {
    for (int y=0; y<lcd_height(); y+= num_lines_to_write) {
      Pixel* _buf = (Pixel*)((uint32_t)vram0 * (vram_index ^ 0x01) + (uint32_t)vram1 * vram_index);
      vram_index = vram_index ^ 0x01;
      int num_lines = std::min<int>(num_lines_to_write, lcd_height()-y);
      // memset the buffer to 0
      memset(_buf, 0, lcd_width() * num_lines * sizeof(Pixel));
      bsp.write_lcd_frame(0, y + _y_offset, lcd_width(), num_lines, (uint8_t*)&_buf[0]);
    }

    // now return
    return false;
  }

  if (is_native()) {
    if (has_palette()) {
      for (int y=0; y<display_height_; y+= num_lines_to_write) {
        uint16_t* _buf = (uint16_t*)((uint32_t)vram0 * (vram_index ^ 0x01) + (uint32_t)vram1 * vram_index);
        vram_index = vram_index ^ 0x01;
        int num_lines = std::min<int>(num_lines_to_write, display_height_-y);
        const uint8_t* _frame = (const uint8_t*)_frame_ptr;
        for (int i=0; i<num_lines; i++) {
          // write two pixels (32 bits) at a time because it's faster
          for (int j=0; j<display_width_/2; j++) {
            int src_index = (y+i)*native_pitch_ + j * 2;
            int dst_index = i*display_width_ + j * 2;
            _buf[dst_index] = _palette[_frame[src_index] % palette_size_];
            _buf[dst_index + 1] = _palette[_frame[src_index + 1] % palette_size_];
          }
        }
        bsp.write_lcd_frame(_x_offset, y + _y_offset, display_width_, num_lines, (uint8_t*)&_buf[0]);
      }
    } else {
      // no palette
      for (int y=0; y<display_height_; y+= num_lines_to_write) {
        uint16_t* _buf = (uint16_t*)((uint32_t)vram0 * (vram_index ^ 0x01) + (uint32_t)vram1 * vram_index);
        vram_index = vram_index ^ 0x01;
        int num_lines = std::min<int>(num_lines_to_write, display_height_-y);
        const uint16_t* _frame = (const uint16_t*)_frame_ptr;
        for (int i=0; i<num_lines; i++) {
          // write two pixels (32 bits) at a time because it's faster
          for (int j=0; j<display_width_/2; j++) {
            int src_index = (y+i)*native_pitch_ + j * 2;
            int dst_index = i*display_width_ + j * 2;
            // memcpy(&_buf[i*display_width_ + j * 2], &_frame[(y+i)*native_pitch_ + j * 2], 4);
            _buf[dst_index] = _frame[src_index];
            _buf[dst_index + 1] = _frame[src_index + 1];
          }
        }
        bsp.write_lcd_frame(_x_offset, y + _y_offset, display_width_, num_lines, (uint8_t*)&_buf[0]);
      }
    }
  } else {
    // we are scaling the screen (and possibly using a custom palette)
    // if we don't have a custom palette, we just need to scale/fill the frame
    [[maybe_unused]] float y_scale = (float)display_height_/native_height_;
    float x_scale = (float)display_width_/native_width_;
    float inv_x_scale = (float)native_width_/display_width_;
    float inv_y_scale = (float)native_height_/display_height_;
    int max_y = Bsp::lcd_height();
    int max_x = std::clamp<int>(x_scale * native_width_, 0, Bsp::lcd_width());
    if (has_palette()) {
      for (int y=0; y<max_y; y+=num_lines_to_write) {
        // each iteration of the loop, we swap the vram index so that we can
        // write to the other buffer while the other one is being transmitted
        int i = 0;
        uint16_t* _buf = (uint16_t*)((uint32_t)vram0 * (vram_index ^ 0x01) + (uint32_t)vram1 * vram_index);
        vram_index = vram_index ^ 0x01;
        for (; i<num_lines_to_write; i++) {
          int _y = y+i;
          if (_y >= max_y) {
            break;
          }
          int source_y = (float)_y * inv_y_scale;
          const uint8_t* _frame = (const uint8_t*)_frame_ptr;
          // write two pixels (32 bits) at a time because it's faster
          for (int x=0; x<max_x/2; x++) {
            int source_x = (float)x * 2 * inv_x_scale;
            int src_index = source_y*native_pitch_ + source_x;
            int dst_index = i*max_x + x * 2;
            _buf[dst_index] = _palette[_frame[src_index] % palette_size_];
            _buf[dst_index + 1] = _palette[_frame[src_index + 1] % palette_size_];
          }
        }
        bsp.write_lcd_frame(0 + _x_offset, y, max_x, i, (uint8_t*)&_buf[0]);
      }
    } else {
      // no palette
      for (int y=0; y<max_y; y+=num_lines_to_write) {
        // each iteration of the loop, we swap the vram index so that we can
        // write to the other buffer while the other one is being transmitted
        int i = 0;
        uint16_t* _buf = (uint16_t*)((uint32_t)vram0 * (vram_index ^ 0x01) + (uint32_t)vram1 * vram_index);
        vram_index = vram_index ^ 0x01;
        for (; i<num_lines_to_write; i++) {
          int _y = y+i;
          if (_y >= max_y) {
            break;
          }
          int source_y = (float)_y * inv_y_scale;
          const uint16_t* _frame = (const uint16_t*)_frame_ptr;
          // write two pixels (32 bits) at a time because it's faster
          for (int x=0; x<max_x/2; x++) {
            int source_x = (float)x * 2 * inv_x_scale;
            int src_index = source_y*native_pitch_ + source_x;
            int dst_index = i*max_x + x * 2;
            _buf[dst_index] = _frame[src_index];
            _buf[dst_index + 1] = _frame[src_index + 1];
          }
        }
        bsp.write_lcd_frame(0 + _x_offset, y, max_x, i, (uint8_t*)&_buf[0]);
      }
    }
  }
  return false;
}
