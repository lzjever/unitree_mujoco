#pragma once

#include <atomic>
#include <chrono>
#include <string>
#include <thread>

#include "reference_frame_cache.h"

namespace agentic_ref
{

struct ReferenceFrameClientConfig
{
  std::string url = "http://127.0.0.1:8083/_sim/reference_frame";
  double poll_hz = 25.0;
  int timeout_ms = 15;
};

namespace internal
{

enum class HttpParseStatus
{
  kIncomplete,
  kComplete,
  kInvalid,
};

HttpParseStatus ExtractHttpBody(const std::string& response, bool eof, std::string* body);

}  // namespace internal

class ReferenceFrameClient
{
public:
  ReferenceFrameClient(ReferenceFrameClientConfig config, ReferenceFrameCache* cache);
  ~ReferenceFrameClient();

  ReferenceFrameClient(const ReferenceFrameClient&) = delete;
  ReferenceFrameClient& operator=(const ReferenceFrameClient&) = delete;

  void Start();
  void Stop();

private:
  void Run();
  bool FetchOnce(std::string* body);

  ReferenceFrameClientConfig config_;
  ReferenceFrameCache* cache_ = nullptr;
  std::thread thread_;
  std::atomic_bool stop_{false};

  std::string host_ = "127.0.0.1";
  int port_ = 8083;
  std::string path_ = "/_sim/reference_frame";
  bool url_valid_ = true;
};

}  // namespace agentic_ref
