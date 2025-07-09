#include "firebase.hpp"

#include <esp_http_client.h>
#include <esp_log.h>

#include <cstdlib>
#include <cstring>
#include <memory>

#include "cJSON.h"
#include "esp_crt_bundle.h"
#include "http.hpp"

namespace mb {

Firebase::Firebase(const char* tag, const char* apiKey, const char* host)
    : client_{nullptr} {
  tag_ = tag;
  idToken_[0] = '\0';

  if (!apiKey || strlen(apiKey) > 64) {
    ESP_LOGE(tag_, "API key not set.");
    return;
  }
  apiKey_ = apiKey;

  if (!host || strlen(host) > 64) {
    ESP_LOGE(tag_, "Host not set.");
    return;
  }

  // Remove "https://" prefix if present
  const char* clean_host = host;
  constexpr char prefix[] = "https://";
  constexpr size_t prefix_len = sizeof(prefix) - 1;
  if (strncmp(host, prefix, prefix_len) == 0) {
    clean_host = host + prefix_len;
  }
  strncpy(host_, clean_host, sizeof(host_) - 2);  // leave space for '/'
  host_[sizeof(host_) - 2] = '\0';                // ensure null-termination

  // Append '/' if missing
  if (size_t len = strlen(host_); host_[len - 1] != '/') {
    host_[len] = '/';
    host_[len + 1] = '\0';
  }

  char url[256];
  snprintf(url, sizeof(url),
           "https://identitytoolkit.googleapis.com/v1/accounts:signUp?key=%s",
           apiKey_);

  esp_http_client_config_t config = {};
  config.url = url;
  config.event_handler = Http::HttpEventHandler;
  config.user_data = Http::GetBuffer();
  config.buffer_size_tx = Http::GetBufferSize();
  config.buffer_size = Http::GetBufferSize();
  config.crt_bundle_attach = esp_crt_bundle_attach;

  client_ = esp_http_client_init(&config);
  ESP_LOGD(tag_, "HTTP Client Initialized.");
}

http_ret_t Firebase::PerformRequest(const char* url,
                                    const esp_http_client_method_t method,
                                    const char* postField) {
  ESP_ERROR_CHECK(esp_http_client_set_url(client_, url));
  ESP_ERROR_CHECK(esp_http_client_set_method(client_, method));
  ESP_ERROR_CHECK(
      esp_http_client_set_header(client_, "Content-Type", "application/json"));

  if (postField && (method == HTTP_METHOD_POST || method == HTTP_METHOD_PUT))
    ESP_ERROR_CHECK(
        esp_http_client_set_post_field(client_, postField, strlen(postField)));

  const esp_err_t err = esp_http_client_perform(client_);
  const int status = esp_http_client_get_status_code(client_);

  if (err != ESP_OK || status != 200) {
    ESP_LOGE(
        tag_,
        "Error while performing request esp_err_t code=0x%x | status_code=%d",
        (int)err, status);
    ESP_LOGE(tag_, "request: url=%s \nmethod=%d \npost_field=%s", url, method,
             postField);
    ESP_LOGE(tag_, "response=\n%s", Http::GetBuffer());
  }

  return http_ret_t{err, status};
}

int Firebase::SignInAnonymously() {
  const auto payload = R"({"returnSecureToken": true})";

  char url[256];
  snprintf(url, sizeof(url),
           "https://identitytoolkit.googleapis.com/v1/accounts:signUp?key=%s",
           apiKey_);

  auto [err, status] = PerformRequest(url, HTTP_METHOD_POST, payload);
  if (err != ESP_OK || status != 200) {
    ESP_LOGE(tag_, "Error while performing anonymously sign-in request.");
    return ESP_FAIL;
  }

  const char* data = Http::GetBuffer();
  if (!data) {
    ESP_LOGE(tag_, "No response data.");
    return ESP_FAIL;
  }

  cJSON* json = cJSON_Parse(data);
  if (!json) {
    ESP_LOGE(tag_, "Failed to parse JSON.");
    return ESP_FAIL;
  }

  auto token = cJSON_GetObjectItem(json, "idToken");
  if (token && cJSON_IsString(token)) {
    strncpy(idToken_, token->valuestring, sizeof(idToken_) - 1);
  }

  isConnected_ = true;
  ESP_LOGI(tag_, "Anonymous sign-in successful.");
  cJSON_Delete(json);
  return ESP_OK;
}

void Firebase::Send(const char* path, const char* json) {
  if (!isConnected_) {
    ESP_LOGE(tag_, "Host or ID token not set.");
    return;
  }

  char url[64 + strlen(host_) + strlen(path) + strlen(idToken_) + 1];
  snprintf(url, sizeof(url), "https://%s%s.json?auth=%s", host_, path,
           idToken_);

  auto [err, status] = PerformRequest(url, HTTP_METHOD_PUT, json);
  if (err != ESP_OK || status != 200) {
    ESP_LOGE(tag_, "Error while performing request.");
    return;
  }

  ESP_LOGI(tag_, "Data sent; HTTP status = %lu", status);
}

void Firebase::Read(const char* path) {
  if (!isConnected_) {
    ESP_LOGE(tag_, "Host or ID token not set.");
    return;
  }

  char url[64 + strlen(host_) + strlen(path) + strlen(idToken_) + 1];
  snprintf(url, sizeof(url), "https://%s%s.json?auth=%s", host_, path,
           idToken_);

  auto [err, status] = PerformRequest(url, HTTP_METHOD_GET, nullptr);
  if (err != ESP_OK || status != 200) {
    ESP_LOGE(tag_, "Error while performing request.");
    return;
  }

  const char* data = Http::GetBuffer();
  if (!data || strlen(data) == 0) {
    ESP_LOGE(tag_, "No response data.");
    return;
  }

  ESP_LOGI(tag_, "Received data: %s", data);
}

}  // namespace mb
