#include "aqara_fp2_accel.h"
#include "esphome/core/log.h"
#include "esphome/core/hal.h"
#include <cmath>

namespace esphome {
namespace aqara_fp2_accel {

bool AqaraFP2Accel::i2c_init_bus() {
  ESP_LOGI(TAG, "Initializing I2C bus on port %d (SDA=%d, SCL=%d, freq=%d Hz)",
           i2c_port_, sda_pin_, scl_pin_, frequency_);

  i2c_master_bus_config_t conf = {};
  conf.clk_source = I2C_CLK_SRC_DEFAULT;
  conf.i2c_port = i2c_port_;
  conf.sda_io_num = static_cast<gpio_num_t>(sda_pin_);
  conf.scl_io_num = static_cast<gpio_num_t>(scl_pin_);
  conf.glitch_ignore_cnt = 7;
  conf.flags.enable_internal_pullup = true;

  esp_err_t err = i2c_new_master_bus(&conf, &bus_);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "I2C bus create failed: %s", esp_err_to_name(err));
    return false;
  }

  i2c_device_config_t acc_conf = {};
  acc_conf.dev_addr_length = I2C_ADDR_BIT_LEN_7;
  acc_conf.device_address = ACC_SENSOR_ADDR;
  acc_conf.scl_speed_hz = frequency_;
  acc_conf.scl_wait_us = 20000;  // tolerate clock stretching before declaring the bus stuck
  err = i2c_master_bus_add_device(bus_, &acc_conf, &acc_dev_);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "I2C add accelerometer failed: %s", esp_err_to_name(err));
    return false;
  }

  ESP_LOGI(TAG, "I2C bus initialized successfully");
  i2c_initialized_ = true;

  // (No full bus scan with the new driver: probing every address can leave
  // the bus flagged busy.)  Known devices: accelerometer 0x27, OPT3001 0x44.
  return true;
}

void AqaraFP2Accel::setup() {
  ESP_LOGCONFIG(TAG, "Setting up Aqara FP2 Accelerometer...");

  // Initialize I2C bus
  if (!i2c_init_bus()) {
    ESP_LOGE(TAG, "Failed to initialize I2C bus");
    this->mark_failed();
    return;
  }

  // Initialize the accelerometer
  i2c_init_acc();

  // Initialize the ambient light sensor
  lux_init_();

  // Create mutex for thread-safe access
  mutex_ = xSemaphoreCreateMutex();
  if (mutex_ == nullptr) {
    ESP_LOGE(TAG, "Failed to create mutex");
    this->mark_failed();
    return;
  }

  // Create FreeRTOS task for I2C operations
  task_running_ = true;
  BaseType_t result = xTaskCreate(
    accel_task_,           // Task function
    "accel_task",          // Task name
    4096,                  // Stack size
    this,                  // Parameter (this pointer)
    1,                     // Priority
    &task_handle_          // Task handle
  );

  if (result != pdPASS) {
    ESP_LOGE(TAG, "Failed to create accelerometer task");
    this->mark_failed();
    task_running_ = false;
  } else {
    ESP_LOGI(TAG, "Accelerometer task created successfully");
  }
}

void AqaraFP2Accel::accel_task_(void *param) {
  AqaraFP2Accel *accel = static_cast<AqaraFP2Accel *>(param);
  accel->task_loop_();
}

void AqaraFP2Accel::task_loop_() {
  ESP_LOGI(TAG, "Accelerometer task started (interval: %d ms)", update_interval_ms_);

  while (task_running_) {
    // Read and process accelerometer data
    read_process_accel();

    // Ambient light: the OPT3001 converts every 800 ms, poll at 1 Hz
    if (lux_present_ && millis() - lux_last_read_ms_ >= 1000) {
      lux_last_read_ms_ = millis();
      lux_read_();
    }

    // Delay for the configured interval
    vTaskDelay(pdMS_TO_TICKS(update_interval_ms_));
  }

  ESP_LOGI(TAG, "Accelerometer task stopped");
  vTaskDelete(nullptr);
}

// Tamper: runs on the task thread once per good 10-sample block (~1 s).
void AqaraFP2Accel::update_tamper_(int32_t ax, int32_t ay, int32_t az) {
  uint32_t now = millis();
  if (mutex_ != nullptr) xSemaphoreTake(mutex_, portMAX_DELAY);
  bool want_baseline = baseline_request_ || (!baseline_.valid && good_blocks_ >= 30);  // ~30 s of clean data after boot
  if (want_baseline) {
    float m = sqrtf((float) ax * ax + (float) ay * ay + (float) az * az);
    baseline_ = {ax / m, ay / m, az / m, 1};
    baseline_request_ = false;
    baseline_save_pending_ = true;
    dev_over_since_ = dev_under_since_ = 0;
    tamper_ = false;
    ESP_LOGI(TAG, "Tamper baseline set: (%.3f, %.3f, %.3f)", baseline_.x, baseline_.y, baseline_.z);
  }
  if (baseline_.valid) {
    float m = sqrtf((float) ax * ax + (float) ay * ay + (float) az * az);
    float dot = (ax / m) * baseline_.x + (ay / m) * baseline_.y + (az / m) * baseline_.z;
    if (dot > 1.0f) dot = 1.0f; if (dot < -1.0f) dot = -1.0f;
    deviation_deg_ = acosf(dot) * 180.0f / 3.14159265f;
    bool over = deviation_deg_ >= tamper_threshold_deg_;
    if (over) { dev_under_since_ = 0; if (dev_over_since_ == 0) dev_over_since_ = now; }
    else      { dev_over_since_ = 0;  if (dev_under_since_ == 0) dev_under_since_ = now; }
    if (!tamper_ && over && now - dev_over_since_ >= tamper_hold_ms_) {
      tamper_ = true;  ESP_LOGW(TAG, "TAMPER detected: deviation %.1f deg >= %.1f for %u ms", deviation_deg_, tamper_threshold_deg_, (unsigned) tamper_hold_ms_);
    } else if (tamper_ && !over && now - dev_under_since_ >= tamper_hold_ms_) {
      tamper_ = false; ESP_LOGI(TAG, "Tamper clear: deviation %.1f deg", deviation_deg_);
    }
  }
  if (mutex_ != nullptr) xSemaphoreGive(mutex_);
}

// Main-thread loop: publish values produced by the task
void AqaraFP2Accel::loop() {
  if (!baseline_loaded_) {
    baseline_loaded_ = true;
    baseline_pref_ = global_preferences->make_preference<TamperBaseline>(0x7A4B5E01);
    TamperBaseline b{};
    if (baseline_pref_.load(&b) && b.valid) {
      if (mutex_ != nullptr) xSemaphoreTake(mutex_, portMAX_DELAY);
      baseline_ = b;
      if (mutex_ != nullptr) xSemaphoreGive(mutex_);
      ESP_LOGI(TAG, "Tamper baseline restored from flash: (%.3f, %.3f, %.3f)", b.x, b.y, b.z);
    } else {
      ESP_LOGI(TAG, "No tamper baseline in flash; capturing after ~30 s of clean samples");
    }
  }
  if (mutex_ == nullptr) return;
  xSemaphoreTake(mutex_, portMAX_DELAY);
  float lux = lux_value_;
  bool lux_dirty = lux_dirty_;
  lux_dirty_ = false;
  bool accel_dirty = accel_dirty_;
  accel_dirty_ = false;
  Orientation orient = stable_orientation_;
  int angle = output_angle_z_;
  bool vib = acc_state_.is_vibrating;
  bool tamper = tamper_;
  float deviation = deviation_deg_;
  bool save_baseline = baseline_save_pending_;
  baseline_save_pending_ = false;
  TamperBaseline b = baseline_;
  xSemaphoreGive(mutex_);
  if (save_baseline) {
    baseline_pref_.save(&b);
    ESP_LOGI(TAG, "Tamper baseline saved to flash");
  }
  if (tamper_sensor_ != nullptr && (int) tamper != published_tamper_) {
    published_tamper_ = tamper;
    tamper_sensor_->publish_state(tamper);
  }
  if (tamper_deviation_sensor_ != nullptr && !std::isnan(deviation)) {
    uint32_t now = millis();
    bool changed = std::isnan(published_deviation_) || fabsf(deviation - published_deviation_) >= 0.5f;
    if (changed || now - deviation_published_ms_ >= 60000) {
      published_deviation_ = deviation; deviation_published_ms_ = now;
      tamper_deviation_sensor_->publish_state(roundf(deviation * 10.0f) / 10.0f);
    }
  }
  if (lux_dirty && illuminance_sensor_ != nullptr) illuminance_sensor_->publish_state(lux);
  if (!accel_dirty) return;
  if (orientation_sensor_ != nullptr && orient != published_orientation_) {
    published_orientation_ = orient;
    orientation_sensor_->publish_state(orientation_to_string(orient));
  }
  if (tilt_angle_sensor_ != nullptr && (published_angle_ == -1000 || abs(angle - published_angle_) >= 2)) {
    published_angle_ = angle;
    tilt_angle_sensor_->publish_state(angle);
  }
  if (vibration_sensor_ != nullptr && (int) vib != published_vibration_) {
    published_vibration_ = vib;
    vibration_sensor_->publish_state(vib);
  }
}

// ---- OPT3001 ambient light sensor -------------------------------------
static const uint8_t LUX_SENSOR_ADDR = 0x44;

bool AqaraFP2Accel::lux_read_reg16_(uint8_t reg, uint16_t *value) {
  if (lux_dev_ == nullptr) return false;
  uint8_t data[2];
  esp_err_t err = i2c_master_transmit_receive(lux_dev_, &reg, 1, data, 2, 100);
  if (err != ESP_OK) return false;
  *value = (uint16_t)((data[0] << 8) | data[1]);
  return true;
}

bool AqaraFP2Accel::lux_write_reg16_(uint8_t reg, uint16_t value) {
  if (lux_dev_ == nullptr) return false;
  uint8_t buf[3] = {reg, (uint8_t)(value >> 8), (uint8_t)(value & 0xFF)};
  return i2c_master_transmit(lux_dev_, buf, 3, 100) == ESP_OK;
}

void AqaraFP2Accel::lux_init_() {
  if (bus_ == nullptr) return;
  i2c_device_config_t lux_conf = {};
  lux_conf.dev_addr_length = I2C_ADDR_BIT_LEN_7;
  lux_conf.device_address = LUX_SENSOR_ADDR;
  lux_conf.scl_speed_hz = frequency_;
  if (i2c_master_bus_add_device(bus_, &lux_conf, &lux_dev_) != ESP_OK) {
    ESP_LOGW(TAG, "I2C add light sensor failed");
    return;
  }
  uint16_t mfg = 0, dev = 0;
  if (!lux_read_reg16_(0x7E, &mfg)) {
    ESP_LOGI(TAG, "No light sensor answered at 0x%02X", LUX_SENSOR_ADDR);
    return;
  }
  lux_read_reg16_(0x7F, &dev);
  ESP_LOGI(TAG, "Light sensor at 0x%02X: manufacturer 0x%04X device 0x%04X", LUX_SENSOR_ADDR, mfg, dev);
  if (mfg != 0x5449) {  // "TI"
    ESP_LOGW(TAG, "Light sensor is not an OPT3001 (expected mfg 0x5449); not driving it");
    return;
  }
  // Config: automatic full-scale range, 800 ms conversion, continuous mode
  if (!lux_write_reg16_(0x01, 0xCE10)) {
    ESP_LOGW(TAG, "Failed to configure OPT3001");
    return;
  }
  uint16_t cfg = 0;
  lux_read_reg16_(0x01, &cfg);
  ESP_LOGI(TAG, "OPT3001 ambient light sensor enabled (config readback 0x%04X)", cfg);
  lux_present_ = true;
}

void AqaraFP2Accel::lux_read_() {
  uint16_t raw;
  if (!lux_read_reg16_(0x00, &raw)) { ESP_LOGW(TAG, "OPT3001 read failed"); return; }
  static uint8_t n = 0;
  if ((n++ % 30) == 0) {
    uint16_t cfg = 0; lux_read_reg16_(0x01, &cfg);
    ESP_LOGD(TAG, "OPT3001 raw=0x%04X cfg=0x%04X", raw, cfg);
  }
  // lux = 0.01 * 2^E * R   (E = bits 15..12, R = bits 11..0)
  float lux = 0.01f * (float)(1 << (raw >> 12)) * (float)(raw & 0x0FFF);
  if (mutex_ == nullptr) return;
  xSemaphoreTake(mutex_, portMAX_DELAY);
  lux_value_ = lux;
  lux_dirty_ = true;
  xSemaphoreGive(mutex_);
}

// Thread-safe public accessors
int AqaraFP2Accel::get_output_angle_z() const {
  if (mutex_ == nullptr) return 0;
  xSemaphoreTake(mutex_, portMAX_DELAY);
  int value = output_angle_z_;
  xSemaphoreGive(mutex_);
  return value;
}

Orientation AqaraFP2Accel::get_orientation() const {
  if (mutex_ == nullptr) return Orientation::INVALID;
  xSemaphoreTake(mutex_, portMAX_DELAY);
  Orientation value = stable_orientation_;
  xSemaphoreGive(mutex_);
  return value;
}

bool AqaraFP2Accel::is_vibrating() const {
  if (mutex_ == nullptr) return false;
  xSemaphoreTake(mutex_, portMAX_DELAY);
  bool value = acc_state_.is_vibrating;
  xSemaphoreGive(mutex_);
  return value;
}

void AqaraFP2Accel::dump_config() {
  ESP_LOGCONFIG(TAG, "Aqara FP2 Accelerometer:");
  ESP_LOGCONFIG(TAG, "  I2C Port: %d", i2c_port_);
  ESP_LOGCONFIG(TAG, "  SDA Pin: GPIO%d", sda_pin_);
  ESP_LOGCONFIG(TAG, "  SCL Pin: GPIO%d", scl_pin_);
  ESP_LOGCONFIG(TAG, "  Frequency: %d Hz", frequency_);
  ESP_LOGCONFIG(TAG, "  I2C Address: 0x%02X", ACC_SENSOR_ADDR);
  ESP_LOGCONFIG(TAG, "  Update Interval: %d ms", update_interval_ms_);
  ESP_LOGCONFIG(TAG, "  Calibration X: %d", accel_corr_x_);
  ESP_LOGCONFIG(TAG, "  Calibration Y: %d", accel_corr_y_);
  ESP_LOGCONFIG(TAG, "  Calibration Z: %d", accel_corr_z_);
}

bool AqaraFP2Accel::i2c_read_accel_xyz(int16_t *x, int16_t *y, int16_t *z) {
  // Read all 6 registers at once (0x02-0x07)
  // X: 0x02, 0x03
  // Y: 0x04, 0x05
  // Z: 0x06, 0x07
  uint8_t data[6];
  uint8_t reg_addr = 0x02;

  esp_err_t err = acc_dev_ == nullptr ? ESP_ERR_INVALID_STATE
                                      : i2c_master_transmit_receive(acc_dev_, &reg_addr, 1, data, 6, 200);

  if (err != ESP_OK) {
    ESP_LOGW(TAG, "Failed to read accelerometer data: %s", esp_err_to_name(err));
    if (err == ESP_ERR_INVALID_STATE || err == ESP_ERR_TIMEOUT) i2c_master_bus_reset(bus_);
    *x = 0;
    *y = 0;
    *z = 0;
    return false;
  }

  // Parse X axis (registers 0x02, 0x03)
  uint16_t x_raw = (uint16_t)(data[0] >> 4) | ((uint16_t)data[1] << 4);
  if ((x_raw & 0x800) != 0) {
    x_raw |= 0xf000;  // Sign extension
  }
  *x = (int16_t)x_raw;

  // Parse Y axis (registers 0x04, 0x05)
  uint16_t y_raw = (uint16_t)(data[2] >> 4) | ((uint16_t)data[3] << 4);
  if ((y_raw & 0x800) != 0) {
    y_raw |= 0xf000;  // Sign extension
  }
  *y = (int16_t)y_raw;

  // Parse Z axis (registers 0x06, 0x07)
  uint16_t z_raw = (uint16_t)(data[4] >> 4) | ((uint16_t)data[5] << 4);
  if ((z_raw & 0x800) != 0) {
    z_raw |= 0xf000;  // Sign extension
  }
  *z = (int16_t)z_raw;

  return true;
}

bool AqaraFP2Accel::i2c_write_reg(uint8_t reg, uint8_t value) {
  uint8_t write_buf[2] = {reg, value};

  esp_err_t err = acc_dev_ == nullptr ? ESP_ERR_INVALID_STATE
                                      : i2c_master_transmit(acc_dev_, write_buf, 2, 1000);

  if (err != ESP_OK) {
    ESP_LOGW(TAG, "Failed to write register 0x%02X: %s", reg, esp_err_to_name(err));
    return false;
  }

  return true;
}

void AqaraFP2Accel::i2c_init_acc() {
  ESP_LOGI(TAG, "Initializing accelerometer");

  // Write 0x0E to Reg 0x11, 0x40 to Reg 0x0F
  if (!i2c_write_reg(0x11, 0x0e)) {
    ESP_LOGW(TAG, "Failed to write to register 0x11");
  }

  if (!i2c_write_reg(0x0f, 0x40)) {
    ESP_LOGW(TAG, "Failed to write to register 0x0F");
  }
}

bool AqaraFP2Accel::calculate_calibration() {
  ESP_LOGD(TAG, "acc_x=%d y=%d z=%d", acc_x_avg_, acc_y_avg_, acc_z_avg_);

  // Sanity check: If all axes equal, or pairs equal, invalid state
  if (acc_x_avg_ == acc_z_avg_) return false;
  if (acc_z_avg_ == acc_y_avg_) return false;
  if (acc_x_avg_ == acc_y_avg_) return false;

  // Calculate correction
  // Target is 0, 0, -1024 (1G downward)
  accel_corr_x_ = -acc_x_avg_;
  accel_corr_y_ = -acc_y_avg_;
  accel_corr_z_ = -1024 - acc_z_avg_;

  accel_corr_x_ = 0;
  accel_corr_y_ = 0;
  accel_corr_z_ = 0;
  return true;

  // ESP_LOGD(TAG, "correction_x=%d y=%d z=%d", accel_corr_x_, accel_corr_y_, accel_corr_z_);

  // return true;
}

static const double SCALE_FACTOR = 57.2957795;  // 180.0 / PI - radians to degrees

void AqaraFP2Accel::acc_data_deal(int32_t acc_raw_x, int32_t acc_raw_y, int32_t acc_raw_z, int acc_mag_sum) {
    // 1. Calculate Standard Inclination Angles
    // These represent the angle of each axis relative to the horizontal plane.
    // Range: -90 to +90 degrees.
    // atan2(axis, hypot(other_axes)) handles all quadrants and avoids div-by-zero.

    double dx = (double)acc_raw_x;
    double dy = (double)acc_raw_y;
    double dz = (double)acc_raw_z;

    // Angle Y: calculated from X axis (reference: i_angle_y = atan(x / sqrt(y² + z²)))
    int16_t i_angle_y = (int16_t)(atan2(dx, hypot(dy, dz)) * SCALE_FACTOR);

    // Angle X: calculated from Y axis (reference: i_angle_x = atan(y / sqrt(x² + z²)))
    int16_t i_angle_x = (int16_t)(atan2(dy, hypot(dx, dz)) * SCALE_FACTOR);

    // Angle Z: Vertical Tilt
    int16_t i_angle_z = (int16_t)(atan2(dz, hypot(dx, dy)) * SCALE_FACTOR);

    int32_t angle_z_int = (int32_t)i_angle_z;

    // 2. Output Angle Calculation
    // Logic from reference: output_angle_z = 90 - abs(angle_z)
    // Maps vertical (z pointing up/down) to 0 and horizontal to 90.
    int16_t c31 = i_angle_z;
    if (angle_z_int < 1) c31 = -c31; // abs()

    if (mutex_ != nullptr) xSemaphoreTake(mutex_, portMAX_DELAY);
    output_angle_z_ = 90 - c31;
    if (mutex_ != nullptr) xSemaphoreGive(mutex_);

    //ESP_LOGI(TAG, "Angles: %d Ax:%d Ay:%d Az:%d", output_angle_z_, i_angle_x, i_angle_y, i_angle_z);

    // 3. Orientation State Machine
    // We use the exact logic from decompilation, but with clear variable names.
    // Note: In assembly, 'acc_raw_z' register was repurposed to hold 'i_angle_y'.
    // We use proper variables here.

    Orientation curr_orient = Orientation::INVALID;

    // Pre-calculate windows for unsigned comparisons (standard compiler trick for range checks)
    // (val + offset) < width  <=>  -offset <= val < width - offset
    uint16_t u_angle_z = (uint16_t)angle_z_int;
    uint16_t check_y_window = (uint16_t)(i_angle_y + 19);
    uint16_t check_x_window = (uint16_t)(i_angle_x + 19);

    // --- ORIENT_UP ---
    // Z is effectively -1g (<-69 units is <-35 deg), X and Y are flat (+/- 10 deg)
    if (angle_z_int < -69) {
      if (check_y_window < 39 && check_x_window < 39) {
        curr_orient = Orientation::UP;
      }
    }

    // --- ORIENT_UP_TILT & REV ---
    // Z is between -69 and -20 units. Y is flat. X is tilted > 10 deg.
    // (u_angle_z + 69) < 49  =>  -69 <= Z < -20
    else if (((uint16_t)(u_angle_z + 69)) < 49) {
      if (check_y_window < 39) {
        int abs_angle_x = std::abs(i_angle_x);
        if (abs_angle_x > 20) {
          curr_orient = (i_angle_x < 0) ? Orientation::UP_TILT : Orientation::UP_TILT_REV;
        }
      }
    }

    // --- ORIENT_DOWN ---
    // Z is +1g (>70 units / >35 deg), X and Y are flat
    else if (angle_z_int > 70) {
      if (check_y_window < 39 && check_x_window < 39) {
        curr_orient = Orientation::DOWN;
      }
    }

    // --- ORIENT_SIDE & REV ---
    // Z is flat (+/- 10 deg). Y is flat. X is vertical (> +/- 25 deg).
    // (u_angle_z + 20) < 40  =>  -20 <= Z < 20
    else if (((uint16_t)(u_angle_z + 20)) < 40) {
      if (check_y_window < 39) {
        int abs_angle_x = std::abs(i_angle_x);
        if (abs_angle_x > 50) {
          curr_orient = (i_angle_x < 0) ? Orientation::SIDE : Orientation::SIDE_REV;
        }
      }
    }

    // --- ORIENT_DOWN_TILT & REV ---
    // Z is between 20 and 70. Y is flat. X is within reasonable bounds (<70 deg).
    // (u_angle_z - 20) < 51  =>  20 <= Z < 71
    else if (((uint16_t)(u_angle_z - 20)) < 51) {
      if (check_y_window < 39) {
        // Check X upper bound (approx 70 deg)
        if ((uint16_t)(i_angle_x + 69) < 139) {
           curr_orient = (i_angle_x < 0) ? Orientation::DOWN_TILT : Orientation::DOWN_TILT_REV;
        }
      }
    }

    // 4. Debouncing and State Updates
    // (This logic remains identical to your previous correct implementation)
    if (acc_state_.deb_invalid < 30) {
      if (curr_orient == Orientation::INVALID) {
        acc_state_.deb_invalid++;
        curr_orient = stable_orientation_;
      } else {
        acc_state_.deb_invalid = 0;
      }
    }

    Orientation stable_orientation = stable_orientation_;
    if (curr_orient != Orientation::INVALID) {
      // Side debounce specific logic
      if (curr_orient == Orientation::SIDE && stable_orientation_ == Orientation::DOWN_TILT) {
        if (acc_state_.deb_side < 10) {
          acc_state_.deb_side++;
          curr_orient = stable_orientation_;
        }
      } else {
        acc_state_.deb_side = 0;
      }
      stable_orientation = curr_orient;

      if (curr_orient == Orientation::SIDE_REV && stable_orientation_ == Orientation::DOWN_TILT_REV) {
        if (acc_state_.deb_side_rev < 10) {
          acc_state_.deb_side_rev++;
          curr_orient = stable_orientation_;
          stable_orientation = stable_orientation_;  // Reset stable_orientation too (per reference)
        }
      } else {
        acc_state_.deb_side_rev = 0;
      }
    }

    // Commit state
    if (mutex_ != nullptr) xSemaphoreTake(mutex_, portMAX_DELAY);
    Orientation prev_raw = raw_orientation_;
    stable_orientation_ = stable_orientation;
    raw_orientation_ = curr_orient;
    if (mutex_ != nullptr) xSemaphoreGive(mutex_);

    // Log change (helpful for debugging "broken" state)
    if (curr_orient != prev_raw) {
      ESP_LOGI(TAG, "Orientation State: %s -> %s (Ax:%d Ay:%d Az:%d)", orientation_to_string(prev_raw), orientation_to_string(curr_orient), i_angle_x, i_angle_y, i_angle_z);
      // Call orientation callback here
    }

  // Vibration Detection
  // Thread-safe update of vibration state
  if (mutex_ != nullptr) {
    xSemaphoreTake(mutex_, portMAX_DELAY);
  }

  uint32_t delta_sum;
  if (acc_mag_sum < acc_state_.last_acc_sum) {
    delta_sum = acc_state_.last_acc_sum - acc_mag_sum;
  } else {
    delta_sum = acc_mag_sum - acc_state_.last_acc_sum;
  }

  // Vibration counters
  if (delta_sum < 1000) {
    acc_state_.vib_high_cnt = 0;
    acc_state_.vib_low_cnt++;
  } else {
    acc_state_.vib_high_cnt++;
    acc_state_.vib_low_cnt = 0;
  }

  // Vibration Trigger (High delta or sustained high count)
  if (delta_sum > 5000 || acc_state_.vib_high_cnt > 4) {
    acc_state_.is_vibrating = true;
    acc_state_.vib_high_cnt = 0;
  }

  // Vibration Reset (Sustained low count)
  if (acc_state_.vib_low_cnt > 9) {
    acc_state_.is_vibrating = false;
    acc_state_.vib_low_cnt = 0;
  }

  acc_state_.last_acc_sum = acc_mag_sum;

  bool vib_state = acc_state_.is_vibrating;
  accel_dirty_ = true;  // loop() publishes orientation / angle / vibration when they changed

  if (mutex_ != nullptr) {
    xSemaphoreGive(mutex_);
  }

  // Vibration State Change (logging outside mutex to avoid blocking)
  if (prev_vib_state_ != vib_state) {
    ESP_LOGI(TAG, "vibration=%s", vib_state ? "true" : "false");
    prev_vib_state_ = vib_state;
  }
}

void AqaraFP2Accel::read_process_accel() {
  int z, y, x;
  int x2, y2, z2;
  uint8_t sampleIdx;

  // Read all accelerometer axes in a single I2C transaction
  if (!i2c_read_accel_xyz(
      &read_acc_x_buf_[accel_samples_read_],
      &read_acc_y_buf_[accel_samples_read_],
      &read_acc_z_buf_[accel_samples_read_])) {
    // Failed to read, skip this sample
    return;
  }

  accel_samples_read_++;

  // Once buffer is full (10 samples)
  if (accel_samples_read_ > 9) {
    accel_samples_read_ = 0;

    // 1. Calculate Average
    x = 0; y = 0; z = 0;
    for (sampleIdx = 0; sampleIdx < 10; sampleIdx++) {
      x += read_acc_x_buf_[sampleIdx];
      y += read_acc_y_buf_[sampleIdx];
      z += read_acc_z_buf_[sampleIdx];
    }
    acc_x_avg_ = (int16_t)(x / 10);
    acc_y_avg_ = (int16_t)(y / 10);
    acc_z_avg_ = (int16_t)(z / 10);

    // 2. Calculate Variance / Energy (Sum of squared differences)
    x2 = 0; y2 = 0; z2 = 0;
    for (sampleIdx = 0; sampleIdx < 10; sampleIdx++) {
      z = (int)(read_acc_x_buf_[sampleIdx]) - (int)(accel_corr_x_ + acc_x_avg_);
      x2 += (z * z);

      z = (int)(read_acc_y_buf_[sampleIdx]) - (int)(accel_corr_y_ + acc_y_avg_);
      y2 += (z * z);

      z = (int)(read_acc_z_buf_[sampleIdx]) - (int)(accel_corr_z_ + acc_z_avg_);
      z2 += (z * z);
    }

    int energy_sum = (x2 / 10) + (y2 / 10) + (z2 / 10);

    // 2b. Plausibility: the averaged vector must be about 1 g (1024 units).
    //     A corrupted I2C read (the loft unit shows them) lands far outside
    //     and would otherwise fire the vibration flag and skew the tilt.
    {
      int32_t cx = accel_corr_x_ + acc_x_avg_, cy = accel_corr_y_ + acc_y_avg_, cz = accel_corr_z_ + acc_z_avg_;
      float mag = sqrtf((float) cx * cx + (float) cy * cy + (float) cz * cz);
      if (mag < 1024.0f * 0.8f || mag > 1024.0f * 1.2f) {
        dropped_blocks_++;
        if (dropped_blocks_ == 1 || dropped_blocks_ % 50 == 0)
          ESP_LOGW(TAG, "Accel block dropped: |g| = %.0f units (%lu dropped so far)", mag, (unsigned long) dropped_blocks_);
        return;
      }
      good_blocks_++;
      update_tamper_(cx, cy, cz);
    }

    // 3. Process Data
    acc_data_deal(
      (int32_t)(accel_corr_x_ + acc_x_avg_),
      (int32_t)(accel_corr_y_ + acc_y_avg_),
      (int32_t)(accel_corr_z_ + acc_z_avg_),
      energy_sum
    );

    //ESP_LOGI(TAG, "got samples %d %d %d",
    //    (int32_t)(accel_corr_x_ + acc_x_avg_),
    //    (int32_t)(accel_corr_y_ + acc_y_avg_),
    //    (int32_t)(accel_corr_z_ + acc_z_avg_)
    //);
  }
}


// ---- Tamper numbers --------------------------------------------------------
void FP2AccelNumber::setup() {
  pref_ = global_preferences->make_preference<float>(0x7A4B5E10 + kind_);
  float v = initial_;
  float stored;
  if (pref_.load(&stored) && !std::isnan(stored)) v = stored;
  apply_(v);
  this->publish_state(v);
}

void FP2AccelNumber::control(float value) {
  apply_(value);
  pref_.save(&value);
  this->publish_state(value);
}

void FP2AccelNumber::apply_(float value) {
  if (parent_ == nullptr) return;
  if (kind_ == THRESHOLD) parent_->set_tamper_threshold(value);
  else parent_->set_tamper_hold(value);
}

}  // namespace aqara_fp2_accel
}  // namespace esphome
