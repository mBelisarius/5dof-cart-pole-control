#include "mpu9250.hpp"

#include <cstring>

#include "driver/i2c.h"
#include "esp_log.h"

namespace mb {

static const MPU9250::AxisMap DEF_ID[3] = {
  {0, +1},  // X ← raw.ax
  {1, +1},  // Y ← raw.ay
  {2, +1}   // Z ← raw.az
};

MPU9250::MPU9250(const i2c_port_t port, const gpio_num_t sda_pin,
                 const gpio_num_t scl_pin, const uint32_t clk_hz, AxisMap axisMap[3])
    : tag_("mpu9250"),
      port_(port),
      sda_(sda_pin),
      scl_(scl_pin),
      clkHz_(clk_hz) {
  if (axisMap) {
    std::memcpy(axisMap_, axisMap, sizeof(axisMap_));
  } else {
    std::memcpy(axisMap_, DEF_ID, sizeof(axisMap_));
  }
}

esp_err_t MPU9250::begin() const {
  // configure I2C master
  i2c_config_t config = {};
  config.mode = I2C_MODE_MASTER;
  config.sda_io_num = sda_;
  config.sda_pullup_en = GPIO_PULLUP_ENABLE;
  config.scl_io_num = scl_;
  config.scl_pullup_en = GPIO_PULLUP_ENABLE;
  config.master.clk_speed = clkHz_;

  esp_err_t ret = i2c_param_config(port_, &config);
  if (ret != ESP_OK) {
    ESP_LOGE(tag_, "i2c_param_config failed: %s", esp_err_to_name(ret));
    return ret;
  }
  ret = i2c_driver_install(port_, I2C_MODE_MASTER, 0, 0, 0);
  if (ret != ESP_OK) {
    ESP_LOGE(tag_, "i2c_driver_install failed: %s", esp_err_to_name(ret));
    return ret;
  }

  // wake up MPU9250
  ret = writeReg(kPwrMgmt1_, 0x00);
  if (ret == ESP_OK) {
    vTaskDelay(pdMS_TO_TICKS(100));
    ESP_LOGI(tag_, "MPU9250 awake");
  } else {
    ESP_LOGE(tag_, "Failed to wake MPU9250: %s", esp_err_to_name(ret));
  }
  return ret;
}

esp_err_t MPU9250::writeReg(const uint8_t reg, const uint8_t data) const {
  i2c_cmd_handle_t cmd = i2c_cmd_link_create();
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, (kAddr_ << 1) | I2C_MASTER_WRITE, true);
  i2c_master_write_byte(cmd, reg, true);
  i2c_master_write_byte(cmd, data, true);
  i2c_master_stop(cmd);

  esp_err_t ret = i2c_master_cmd_begin(port_, cmd, pdMS_TO_TICKS(1000));
  i2c_cmd_link_delete(cmd);
  return ret;
}

esp_err_t MPU9250::readRegs(const uint8_t startReg, uint8_t *buf,
                            const size_t len) {
  i2c_cmd_handle_t cmd = i2c_cmd_link_create();
  // set read address
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, (kAddr_ << 1) | I2C_MASTER_WRITE, true);
  i2c_master_write_byte(cmd, startReg, true);
  // read block
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, (kAddr_ << 1) | I2C_MASTER_READ, true);
  if (len > 1) {
    i2c_master_read(cmd, buf, len - 1, I2C_MASTER_ACK);
  }
  i2c_master_read_byte(cmd, buf + len - 1, I2C_MASTER_NACK);
  i2c_master_stop(cmd);
  esp_err_t ret = i2c_master_cmd_begin(port_, cmd, pdMS_TO_TICKS(1000));
  i2c_cmd_link_delete(cmd);
  return ret;
}

esp_err_t MPU9250::readRaw(Raw &out) {
  uint8_t buf[14];
  esp_err_t ret = readRegs(kAccelXoutH_, buf, sizeof(buf));
  if (ret != ESP_OK) return ret;

  int16_t tmp[6];
  tmp[0] = static_cast<int16_t>((buf[0] << 8) | buf[1]);   // ax
  tmp[1] = static_cast<int16_t>((buf[2] << 8) | buf[3]);   // ay
  tmp[2] = static_cast<int16_t>((buf[4] << 8) | buf[5]);   // az
  tmp[3] = static_cast<int16_t>((buf[8] << 8) | buf[9]);   // gx
  tmp[4] = static_cast<int16_t>((buf[10] << 8) | buf[11]); // gy
  tmp[5] = static_cast<int16_t>((buf[12] << 8) | buf[13]); // gz

  // remap accel
  out.ax = axisMap_[0].sign * tmp[axisMap_[0].idx];
  out.ay = axisMap_[1].sign * tmp[axisMap_[1].idx];
  out.az = axisMap_[2].sign * tmp[axisMap_[2].idx];
  // remap gyro
  out.gx = axisMap_[0].sign * tmp[3 + axisMap_[0].idx];
  out.gy = axisMap_[1].sign * tmp[3 + axisMap_[1].idx];
  out.gz = axisMap_[2].sign * tmp[3 + axisMap_[2].idx];

  return ESP_OK;
}

esp_err_t MPU9250::readSI(SI &out) {
  Raw raw;
  esp_err_t ret = readRaw(raw);
  if (ret != ESP_OK) return ret;

  // accel: LSB → g → m/s²
  out.ax = static_cast<float>(raw.ax) / ACCEL_LSB_PER_G * G_TO_MS2;
  out.ay = static_cast<float>(raw.ay) / ACCEL_LSB_PER_G * G_TO_MS2;
  out.az = static_cast<float>(raw.az) / ACCEL_LSB_PER_G * G_TO_MS2;

  // gyro: LSB → °/s → rad/s
  out.gx = static_cast<float>(raw.gx) / GYRO_LSB_PER_DPS * DPS_TO_RADS;
  out.gy = static_cast<float>(raw.gy) / GYRO_LSB_PER_DPS * DPS_TO_RADS;
  out.gz = static_cast<float>(raw.gz) / GYRO_LSB_PER_DPS * DPS_TO_RADS;

  return ESP_OK;
}

}  // namespace mb
