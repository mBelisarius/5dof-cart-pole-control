#ifndef MB_FIREBASE_HPP_
#define MB_FIREBASE_HPP_

#include <esp_http_client.h>
#include <http.hpp>

namespace mb {

class Firebase {
 public:
  Firebase(const char* tag, const char* apiKey, const char* host);

  ~Firebase() = default;

  http_ret_t PerformRequest(const char* url,
                                    const esp_http_client_method_t method,
                                    const char* postField);

  int SignInAnonymously();

  void Send(const char* path, const char* json);
  void Read(const char* path);

 private:
  const char* tag_{};
  const char* apiKey_{};
  char host_[64]{};
  char idToken_[1024]{};  // Cached JWT from sign-in
  esp_http_client_handle_t client_;
  bool isConnected_{false};
};

}  // namespace mb

#endif  // MB_FIREBASE_HPP_
