#ifndef MB_MPU9250_HPP_
#define MB_MPU9250_HPP_

#include <cmath>

#include "driver/i2c.h"
#include "esp_err.h"

namespace mb {

class MPU9250 {
public:
  struct Raw {
    int16_t ax, ay, az;
    int16_t gx, gy, gz;
  };

  struct SI {
    float ax, ay, az;   // m/s²
    float gx, gy, gz;   // rad/s
  };

  struct AxisMap {
    uint8_t idx;   // 0,1 or 2
    int8_t  sign;  // +1 or -1
  };

  /** ctor: specify I2C port and SDA/SCL pins */
  explicit MPU9250(i2c_port_t port = I2C_NUM_0, gpio_num_t sda_pin = GPIO_NUM_21,
          gpio_num_t scl_pin = GPIO_NUM_22, uint32_t clk_hz = 400000, AxisMap axisMap[3] = nullptr);

  /** Initialize I2C bus and wake sensor */
  esp_err_t begin() const;

  /** Read raw accel+gyro. Returns ESP_OK or error code */
  esp_err_t readRaw(Raw &out);
  esp_err_t readSI(SI &out);

private:
  esp_err_t writeReg(uint8_t reg, uint8_t data) const;
  esp_err_t readRegs(uint8_t startReg, uint8_t *buf, size_t len);

  const char* tag_;
  i2c_port_t port_;
  gpio_num_t sda_, scl_;
  uint32_t clkHz_;
  static constexpr uint8_t kAddr_ = 0x68;
  static constexpr uint8_t kPwrMgmt1_ = 0x6B;
  static constexpr uint8_t kAccelXoutH_ = 0x3B;
  AxisMap axisMap_[3];

  // Sensitivity conversion (default ±2 g, ±250 °/s)
  static constexpr float ACCEL_LSB_PER_G = 16384.0f;
  static constexpr float GYRO_LSB_PER_DPS = 131.0f;  // 32768/250
  static constexpr float G_TO_MS2 = 9.80665f;
  static constexpr float DPS_TO_RADS = (M_PI / 180.0f);
};

}  // namespace mb

#endif // MB_MPU9250_HPP_
