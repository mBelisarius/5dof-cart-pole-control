#include "http.hpp"

#include <esp_log.h>

namespace mb {

const char* Http::tag_ = "HTTP";
uint32_t Http::bufferSize_ = 4096;
std::unique_ptr<char[]> Http::buffer_ =
    std::make_unique<char[]>(Http::bufferSize_);
uint32_t Http::outputLen_ = 0;

uint32_t Http::GetBufferSize() { return bufferSize_; }

void Http::SetBufferSize(const uint32_t bufferSize) {
  bufferSize_ = bufferSize;
  buffer_ = std::make_unique<char[]>(bufferSize_);
  ESP_LOGI(tag_, "HTTP buffer size set to %lu.", bufferSize);
}

char* Http::GetBuffer() { return buffer_.get(); }

void Http::ClearBuffer() {
  memset(buffer_.get(), 0, Http::bufferSize_);
  Http::outputLen_ = 0;
  ESP_LOGI(tag_, "HTTP buffer cleared.");
}

uint32_t Http::GetOutputLen() { return outputLen_; }

esp_err_t Http::HttpEventHandler(esp_http_client_event_t* evt) {
  const char* httpTag = "HTTP_CLIENT";

  switch (evt->event_id) {
    case HTTP_EVENT_ERROR:
      ESP_LOGD(httpTag, "HTTP_EVENT_ERROR");
      break;
    case HTTP_EVENT_ON_CONNECTED:
      ESP_LOGD(httpTag, "HTTP_EVENT_ON_CONNECTED");
      memset(evt->user_data, 0, Http::bufferSize_);
      Http::outputLen_ = 0;
      break;
    case HTTP_EVENT_HEADER_SENT:
      ESP_LOGD(httpTag, "HTTP_EVENT_HEADER_SENT");
      break;
    case HTTP_EVENT_ON_HEADER:
      ESP_LOGD(httpTag, "HTTP_EVENT_ON_HEADER, key=%s, value=%s",
               evt->header_key, evt->header_value);
      break;
    case HTTP_EVENT_ON_FINISH:
      ESP_LOGD(httpTag, "HTTP_EVENT_ON_FINISH");
      Http::outputLen_ = 0;
      break;
    case HTTP_EVENT_ON_DATA:
      ESP_LOGD(httpTag, "HTTP_EVENT_ON_DATA, len=%d", evt->data_len);
      memcpy(evt->user_data + Http::outputLen_, evt->data, evt->data_len);
      Http::outputLen_ += evt->data_len;
      break;
    case HTTP_EVENT_DISCONNECTED:
      ESP_LOGD(httpTag, "HTTP_EVENT_DISCONNECTED");
      break;
    case HTTP_EVENT_REDIRECT:
      ESP_LOGD(httpTag, "HTTP_EVENT_REDIRECT");
      break;
  }

  return ESP_OK;
}

}  // namespace mb
