#pragma once

#include "esphome/core/component.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/text_sensor/text_sensor.h"
#include "esphome/components/binary_sensor/binary_sensor.h"
#include "esphome/components/number/number.h"
#include "esphome/components/button/button.h"
#include "esphome/core/preferences.h"
#include <cstdint>
#include <array>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>
#include <driver/i2c_master.h>

namespace esphome {
namespace aqara_fp2_accel {

static const char *const TAG = "aqara_fp2_accel";

// I2C address for the accelerometer
static const uint8_t ACC_SENSOR_ADDR = 0x27;

// Number of samples to buffer
static const uint8_t ACC_SAMPLE_COUNT = 10;

// Constants for angle calculations
static const double PI = 3.141614159265;
static const double RAD_TO_SCALED_DEG = 180.0 / PI / 0.5;
static const double EPSILON = 0.00001;

// Orientation enumeration
enum class Orientation : uint8_t {
  UP = 0,
  UP_TILT = 1,
  UP_TILT_REV = 2,
  SIDE = 3,
  SIDE_REV = 4,
  DOWN = 5,
  DOWN_TILT = 6,
  DOWN_TILT_REV = 7,
  INVALID = 8
};

// String mapping for logging
static const char *orientation_to_string(Orientation orient) {
  switch (orient) {
    case Orientation::UP: return "UP";
    case Orientation::UP_TILT: return "UP_TILT";
    case Orientation::UP_TILT_REV: return "UP_TILT_REV";
    case Orientation::SIDE: return "SIDE";
    case Orientation::SIDE_REV: return "SIDE_REV";
    case Orientation::DOWN: return "DOWN";
    case Orientation::DOWN_TILT: return "DOWN_TILT";
    case Orientation::DOWN_TILT_REV: return "DOWN_TILT_REV";
    case Orientation::INVALID: return "INVALID";
    default: return "UNKNOWN";
  }
}

// Accelerometer state tracking
struct AccelState {
  uint8_t deb_invalid{0};
  uint8_t deb_side{0};
  uint8_t deb_side_rev{0};
  int32_t last_acc_sum{0};
  uint8_t vib_high_cnt{0};
  uint8_t vib_low_cnt{0};
  bool is_vibrating{false};
};

class AqaraFP2Accel : public Component {
 public:
  void setup() override;
  void loop() override;
  void dump_config() override;

  // Ambient light sensor (TI OPT3001 at 0x44 on the same bus)
  void set_illuminance_sensor(sensor::Sensor *s) { illuminance_sensor_ = s; }
  // Accelerometer outputs as entities: the orientation class the radar is told
  // (UP / SIDE / DOWN_TILT ...), the tilt angle in degrees, and the stock
  // firmware's vibration flag (doubles as a tamper sensor).
  void set_orientation_sensor(text_sensor::TextSensor *s) { orientation_sensor_ = s; }
  void set_tilt_angle_sensor(sensor::Sensor *s) { tilt_angle_sensor_ = s; }
  void set_vibration_sensor(binary_sensor::BinarySensor *s) { vibration_sensor_ = s; }
  // Tamper: the angle between the current gravity vector and a baseline stored
  // in flash. Detected when the deviation stays at or above the threshold for
  // the hold time; clear when it stays below for the hold time. Blocks whose
  // magnitude is not within 20 % of 1 g (corrupted I2C reads) are dropped.
  void set_tamper_sensor(binary_sensor::BinarySensor *s) { tamper_sensor_ = s; }
  void set_tamper_deviation_sensor(sensor::Sensor *s) { tamper_deviation_sensor_ = s; }
  void set_tamper_threshold(float deg) { tamper_threshold_deg_ = deg; }
  void set_tamper_hold(float seconds) { tamper_hold_ms_ = (uint32_t) (seconds * 1000.0f); }
  float get_tamper_threshold() const { return tamper_threshold_deg_; }
  float get_tamper_hold() const { return tamper_hold_ms_ / 1000.0f; }
  void request_tamper_baseline() { baseline_request_ = true; }

  float get_setup_priority() const override { return setup_priority::BUS; }

  // Public accessors for outputs (thread-safe)
  int get_output_angle_z() const;
  Orientation get_orientation() const;
  bool is_vibrating() const;
  bool has_valid_orientation() const { return get_orientation() != Orientation::INVALID; }

  // Calibration
  bool calculate_calibration();

  // Task configuration
  void set_update_interval(uint32_t interval_ms) { update_interval_ms_ = interval_ms; }

 protected:
  // I2C initialization
  bool i2c_init_bus();

  // I2C low level functions (using ESP-IDF driver directly)
  bool i2c_read_accel_xyz(int16_t *x, int16_t *y, int16_t *z);
  bool i2c_write_reg(uint8_t reg, uint8_t value);
  void i2c_init_acc();

  // OPT3001 ambient light sensor
  bool lux_read_reg16_(uint8_t reg, uint16_t *value);
  bool lux_write_reg16_(uint8_t reg, uint16_t value);
  void lux_init_();
  void lux_read_();
  bool lux_present_{false};
  float lux_value_{NAN};
  bool lux_dirty_{false};
  uint32_t lux_last_read_ms_{0};
  sensor::Sensor *illuminance_sensor_{nullptr};
  text_sensor::TextSensor *orientation_sensor_{nullptr};
  sensor::Sensor *tilt_angle_sensor_{nullptr};
  binary_sensor::BinarySensor *vibration_sensor_{nullptr};
  // Set by the sampling task under the mutex, published from loop() on the main thread
  bool accel_dirty_{false};
  // Tamper state (task thread writes under the mutex, loop() publishes)
  binary_sensor::BinarySensor *tamper_sensor_{nullptr};
  sensor::Sensor *tamper_deviation_sensor_{nullptr};
  float tamper_threshold_deg_{3.0f};
  uint32_t tamper_hold_ms_{3000};
  struct TamperBaseline { float x, y, z; uint8_t valid; } baseline_{0, 0, 0, 0};
  ESPPreferenceObject baseline_pref_;
  bool baseline_loaded_{false};
  volatile bool baseline_request_{false};
  bool baseline_save_pending_{false};
  float deviation_deg_{NAN};
  bool tamper_{false};
  uint32_t dev_over_since_{0}, dev_under_since_{0};
  uint32_t good_blocks_{0}, dropped_blocks_{0};
  int published_tamper_{-1};
  float published_deviation_{NAN};
  uint32_t deviation_published_ms_{0};
  void update_tamper_(int32_t ax, int32_t ay, int32_t az);
  Orientation published_orientation_{Orientation::INVALID};
  int published_angle_{-1000};
  int published_vibration_{-1};

  // Data processing
  void read_process_accel();
  void acc_data_deal(int32_t acc_raw_x, int32_t acc_raw_y, int32_t acc_raw_z, int acc_mag_sum);

  // Sample buffers
  std::array<int16_t, ACC_SAMPLE_COUNT> read_acc_x_buf_{};
  std::array<int16_t, ACC_SAMPLE_COUNT> read_acc_y_buf_{};
  std::array<int16_t, ACC_SAMPLE_COUNT> read_acc_z_buf_{};
  uint8_t accel_samples_read_{0};

  // Averaged values
  int16_t acc_x_avg_{0};
  int16_t acc_y_avg_{0};
  int16_t acc_z_avg_{0};

  // Factory calibration corrections
  int16_t accel_corr_x_{0};
  int16_t accel_corr_y_{0};
  int16_t accel_corr_z_{0};

  // State tracking
  AccelState acc_state_{};
  Orientation stable_orientation_{Orientation::INVALID};
  Orientation raw_orientation_{Orientation::INVALID};
  bool prev_vib_state_{false};
  int output_angle_z_{0};

  // FreeRTOS task management
  TaskHandle_t task_handle_{nullptr};
  SemaphoreHandle_t mutex_{nullptr};
  uint32_t update_interval_ms_{100};
  bool task_running_{false};

  // ESP-IDF I2C configuration (new i2c_master driver, IDF >= 5.2 / required by IDF 6)
  i2c_port_t i2c_port_{I2C_NUM_0};
  i2c_master_bus_handle_t bus_{nullptr};
  i2c_master_dev_handle_t acc_dev_{nullptr};
  i2c_master_dev_handle_t lux_dev_{nullptr};
  uint8_t sda_pin_{33};
  uint8_t scl_pin_{32};
  uint32_t frequency_{100000};  // 100 kHz: the FP2 board sticks at 400 kHz (tigercraft4 245955e)
  bool i2c_initialized_{false};

  // Static task function
  static void accel_task_(void *param);

  // Thread-safe task loop
  void task_loop_();
};


// Runtime-tunable tamper numbers (threshold in degrees, hold in seconds),
// restored from flash so a value set in Home Assistant survives a reboot.
class FP2AccelNumber : public number::Number, public Component {
 public:
  enum Kind : uint8_t { THRESHOLD = 0, HOLD = 1 };
  void set_parent(AqaraFP2Accel *parent, uint8_t kind, float initial) { parent_ = parent; kind_ = (Kind) kind; initial_ = initial; }
  void setup() override;
 protected:
  void control(float value) override;
  void apply_(float value);
  AqaraFP2Accel *parent_{nullptr};
  Kind kind_{THRESHOLD};
  float initial_{3.0f};
  ESPPreferenceObject pref_;
};

// Re-captures the tamper baseline from the current gravity vector.
class FP2AccelBaselineButton : public button::Button, public Component {
 public:
  void set_parent(AqaraFP2Accel *parent) { parent_ = parent; }
 protected:
  void press_action() override { if (parent_ != nullptr) parent_->request_tamper_baseline(); }
  AqaraFP2Accel *parent_{nullptr};
};

}  // namespace aqara_fp2_accel
}  // namespace esphome
