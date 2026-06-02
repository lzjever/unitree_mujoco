#pragma once

#include <array>
#include <chrono>
#include <mutex>
#include <optional>
#include <string>

namespace agentic_ref
{

constexpr int kBodyCount = 27;
constexpr const char* kSchema = "ET1REF1";
constexpr const char* kBodyOrder = "et1_27_v1";

struct ReferenceFrame
{
  bool active = false;
  std::string schema;
  std::string body_order;
  std::string id;
  int frame = 0;
  int frames = 0;
  double time_s = 0.0;
  double fps = 0.0;
  double stale_ms = 0.0;
  std::array<std::array<double, 3>, kBodyCount> p{};
  std::array<std::array<double, 4>, kBodyCount> q{};
  std::array<bool, 2> c{};
  std::array<double, 3> com{};
  std::array<double, 3> comv{};
  std::chrono::steady_clock::time_point received_at = std::chrono::steady_clock::now();
};

struct ParseResult
{
  bool ok = false;
  bool active = false;
  std::string error;
  ReferenceFrame frame;
};

ParseResult ParseReferenceFrameJson(const std::string& json);

class ReferenceFrameCache
{
public:
  void StoreInactive();
  void StoreFrame(const ReferenceFrame& frame);
  std::optional<ReferenceFrame> LatestFresh(
      std::chrono::milliseconds max_local_age,
      double max_packet_stale_ms) const;

private:
  mutable std::mutex mutex_;
  ReferenceFrame latest_;
  bool has_latest_ = false;
};

}  // namespace agentic_ref
