#ifndef MB_WIFI_HPP_
#define MB_WIFI_HPP_

#include <memory>

#include "esp_event.h"
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"

namespace mb {

class Wifi {
 public:
  Wifi() = delete;

  static void SetSsid(const char* ssid);

  static void SetPass(const char* pass);

  static void Connect();

  static void EventHandler(void* arg, esp_event_base_t event_base,
                           int32_t event_id, void* event_data);

 private:
  static const char* tag_;
  static constexpr int wifi_connected_bit_ = BIT0;

  static std::unique_ptr<char[]> ssid_;
  static std::unique_ptr<char[]> pass_;

  static EventGroupHandle_t wifiEventGroup_;
};

}  // namespace mb

#endif  // MB_WIFI_HPP_
