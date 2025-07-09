#ifndef MB_HTTP_HPP_
#define MB_HTTP_HPP_

#include <esp_http_client.h>

#include <memory>

namespace mb {

struct http_ret_t {
  esp_err_t err;
  int32_t status;
};

class Http {
 public:
  Http() = delete;

  static uint32_t GetBufferSize();

  static void SetBufferSize(const uint32_t bufferSize);

  static char* GetBuffer();

  static void ClearBuffer();

  static uint32_t GetOutputLen();

  static esp_err_t HttpEventHandler(esp_http_client_event_t* evt);

 private:
  static const char* tag_;
  static uint32_t bufferSize_;
  static std::unique_ptr<char[]> buffer_;
  static uint32_t outputLen_;
};

}  // namespace mb

#endif  // MB_HTTP_HPP_
