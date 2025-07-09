#include "driver/i2c.h"
#include "esp_err.h"
#include "esp_event.h"
#include "esp_http_client.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "firebase.hpp"
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/task.h"
#include "mpu9250.hpp"
#include "nvs_flash.h"
#include "wifi.hpp"

const char* WifiSsid = CONFIG_WIFI_SSID;
const char* WifiPass = CONFIG_WIFI_PASSWORD;
const char* FirebaseHost = CONFIG_FIREBASE_HOST;
const char* FirebaseApiKey = CONFIG_FIREBASE_API_KEY;

mb::Firebase* pRtdb = nullptr;
mb::MPU9250* pMpu = nullptr;

int initializeNvs() {
  esp_err_t err = nvs_flash_init();
  if (err == ESP_ERR_NVS_NO_FREE_PAGES ||
      err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
    ESP_ERROR_CHECK(nvs_flash_erase());
    err = nvs_flash_init();
  }
  return err;
}

int initializeWifi() {
  mb::Wifi::SetSsid(WifiSsid);
  mb::Wifi::SetPass(WifiPass);
  mb::Wifi::Connect();
  return ESP_OK;
}

void processData() {
  const char* tag = "app_process_data";

  // Read data
  mb::MPU9250::SI data;
  if (pMpu->readSI(data) == ESP_OK) {
    ESP_LOGI(tag,
             "Accel [m/s²]: X=%.2f  Y=%.2f  Z=%.2f | "
             "Gyro  [rad/s]: X=%.3f  Y=%.3f  Z=%.3f",
             data.ax, data.ay, data.az, data.gx, data.gy, data.gz);
  } else {
    ESP_LOGE(tag, "Read failed");
  }

  // Send data
  char payload[128];
  int len = snprintf(payload, sizeof(payload),
                     "{"
                     "\"accel\":{"
                     "\"x\":%.2f,"
                     "\"y\":%.2f,"
                     "\"z\":%.2f"
                     "},"
                     "\"gyro\":{"
                     "\"x\":%.3f,"
                     "\"y\":%.3f,"
                     "\"z\":%.3f"
                     "}"
                     "}",
                     data.ax, data.ay, data.az, data.gx, data.gy, data.gz);
  if (len < 0 || len >= static_cast<int>(sizeof(payload))) {
    ESP_LOGE(tag, "JSON payload truncated");
  } else {
    pRtdb->Send("test", payload);
  }
}

extern "C" void app_main(void) {
  const char* tag = "app";
  ESP_ERROR_CHECK(initializeNvs());
  ESP_ERROR_CHECK(initializeWifi());

  // Firebase RTDB setup
  pRtdb = new mb::Firebase("RTDB", FirebaseApiKey, FirebaseHost);
  ESP_ERROR_CHECK(pRtdb->SignInAnonymously());

  // MPU9250 setup
  mb::MPU9250::AxisMap MpuAxisMap[3] = {
      {1, -1},  // new X = +raw.ay
      {2, -1},  // new Y = +raw.ax
      {0, -1}   // new Z = -raw.az
  };
  pMpu =
      new mb::MPU9250(I2C_NUM_0, GPIO_NUM_21, GPIO_NUM_22, 400000, MpuAxisMap);
  if (pMpu->begin() != ESP_OK) {
    ESP_LOGE(tag, "MPU init failed");
    return;
  }

  xTaskCreatePinnedToCore(
      [](void*) {
        while (true) processData();
        vTaskDelete(nullptr);  // Clean exit
      },
      "main_task_worker",
      8192,  // stack size in bytes
      nullptr,
      5,  // priority
      nullptr,
      1);  // core 1 (or 0)
}
