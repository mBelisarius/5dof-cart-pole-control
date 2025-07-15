#include <cJSON.h>

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
  if (pMpu->readSI(data) != ESP_OK)
    ESP_LOGE(tag, "Read failed");

  // Send data

}

void sendRawData(const char* path, const ImuRawData& data) {
  cJSON* payload = cJSON_CreateObject();
  cJSON_AddNumberToObject(payload, "timestamp", esp_log_timestamp());

  cJSON* xddJson = cJSON_CreateObject();
  cJSON_AddNumberToObject(xddJson, "x", data.ax);
  cJSON_AddNumberToObject(xddJson, "y", data.ay);
  cJSON_AddNumberToObject(xddJson, "z", data.az);
  cJSON_AddItemToObject(payload, "xdd", xddJson);

  cJSON* gdJson = cJSON_CreateObject();
  cJSON_AddNumberToObject(gdJson, "x", data.gx);
  cJSON_AddNumberToObject(gdJson, "y", data.gy);
  cJSON_AddNumberToObject(gdJson, "z", data.gz);
  cJSON_AddItemToObject(payload, "gd", gdJson);

  pRtdb->Send("raw", cJSON_Print(payload));
}

void sendImuData(const char* path, const ImuData& data) {
  cJSON* payload = cJSON_CreateObject();
  cJSON_AddNumberToObject(payload, "timestamp", esp_log_timestamp());

  cJSON* xJson = cJSON_CreateObject();
  cJSON_AddNumberToObject(xJson, "x", data.ax);
  cJSON_AddNumberToObject(xJson, "y", data.ay);
  cJSON_AddNumberToObject(xJson, "z", data.az);
  cJSON_AddItemToObject(payload, "xdd", xJson);

  cJSON* xdJson = cJSON_CreateObject();
  cJSON_AddNumberToObject(xdJson, "x", data.ax);
  cJSON_AddNumberToObject(xdJson, "y", data.ay);
  cJSON_AddNumberToObject(xdJson, "z", data.az);
  cJSON_AddItemToObject(payload, "xdd", xdJson);

  cJSON* xddJson = cJSON_CreateObject();
  cJSON_AddNumberToObject(xddJson, "x", data.ax);
  cJSON_AddNumberToObject(xddJson, "y", data.ay);
  cJSON_AddNumberToObject(xddJson, "z", data.az);
  cJSON_AddItemToObject(payload, "xdd", xddJson);

  cJSON* gJson = cJSON_CreateObject();
  cJSON_AddNumberToObject(gJson, "x", data.gx);
  cJSON_AddNumberToObject(gJson, "y", data.gy);
  cJSON_AddNumberToObject(gJson, "z", data.gz);
  cJSON_AddItemToObject(payload, "gd", gJson);

  cJSON* gdJson = cJSON_CreateObject();
  cJSON_AddNumberToObject(gdJson, "x", data.gx);
  cJSON_AddNumberToObject(gdJson, "y", data.gy);
  cJSON_AddNumberToObject(gdJson, "z", data.gz);
  cJSON_AddItemToObject(payload, "gd", gdJson);

  cJSON* gddJson = cJSON_CreateObject();
  cJSON_AddNumberToObject(gddJson, "x", data.gx);
  cJSON_AddNumberToObject(gddJson, "y", data.gy);
  cJSON_AddNumberToObject(gddJson, "z", data.gz);
  cJSON_AddItemToObject(payload, "gd", gddJson);

  pRtdb->Send("imu", cJSON_Print(payload));
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
      {1, -1},  // new X = -raw.ay
      {2, -1},  // new Y = -raw.az
      {0, -1}   // new Z = -raw.ax
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
