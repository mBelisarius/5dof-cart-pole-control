#include "wifi.hpp"

#include <cstring>

#include "esp_err.h"
#include "esp_event.h"
#include "esp_http_client.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/task.h"

namespace mb {

const char *Wifi::tag_ = "WIFI";
std::unique_ptr<char[]> Wifi::ssid_ = nullptr;
std::unique_ptr<char[]> Wifi::pass_ = nullptr;
EventGroupHandle_t Wifi::wifiEventGroup_ = nullptr;

void Wifi::SetSsid(const char *ssid) {
  if (ssid) {
    ssid_ = std::make_unique<char[]>(strlen(ssid) + 1);
    std::strcpy(ssid_.get(), ssid);
  }
}

void Wifi::SetPass(const char *pass) {
  if (pass) {
    pass_ = std::make_unique<char[]>(strlen(pass) + 1);
    std::strcpy(pass_.get(), pass);
  }
}

void Wifi::Connect() {
  static bool initialized = false;
  if (initialized) return;
  initialized = true;

  ESP_LOGI(tag_, "Connecting to Wi-Fi...");

  ESP_ERROR_CHECK(esp_netif_init());
  ESP_ERROR_CHECK(esp_event_loop_create_default());
  esp_netif_create_default_wifi_sta();

  wifiEventGroup_ = xEventGroupCreate();

  wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
  ESP_ERROR_CHECK(esp_wifi_init(&cfg));

  esp_event_handler_instance_t instance_any_id;
  esp_event_handler_instance_t instance_got_ip;

  ESP_ERROR_CHECK(esp_event_handler_instance_register(
      WIFI_EVENT, ESP_EVENT_ANY_ID, &Wifi::EventHandler, nullptr,
      &instance_any_id));
  ESP_ERROR_CHECK(esp_event_handler_instance_register(
      IP_EVENT, IP_EVENT_STA_GOT_IP, &Wifi::EventHandler, nullptr,
      &instance_got_ip));

  wifi_config_t wifi_config = {};
  strncpy(reinterpret_cast<char *>(wifi_config.sta.ssid), ssid_.get(),
          sizeof(wifi_config.sta.ssid));
  strncpy(reinterpret_cast<char *>(wifi_config.sta.password), pass_.get(),
          sizeof(wifi_config.sta.password));
  wifi_config.sta.threshold.authmode = WIFI_AUTH_WPA2_PSK;

  ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
  ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
  ESP_ERROR_CHECK(esp_wifi_start());

  xEventGroupWaitBits(wifiEventGroup_, wifi_connected_bit_, false, true,
                      portMAX_DELAY);

  ESP_LOGI(tag_, "Wi-Fi connected.");
}

void Wifi::EventHandler(void *arg, esp_event_base_t event_base,
                        int32_t event_id, void *event_data) {
  if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
    esp_wifi_connect();
  } else if (event_base == WIFI_EVENT &&
             event_id == WIFI_EVENT_STA_DISCONNECTED) {
    ESP_LOGW(tag_, "Disconnected. Reconnecting...");
    esp_wifi_connect();
  } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
    xEventGroupSetBits(wifiEventGroup_, wifi_connected_bit_);
  }
}

}  // namespace mb
