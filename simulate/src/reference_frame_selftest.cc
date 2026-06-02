#include <iostream>
#include <limits>
#include <sstream>
#include <string>
#include <thread>

#include <mujoco/mujoco.h>

#include "ghost_scene_overlay.h"
#include "reference_frame_cache.h"
#include "reference_frame_client.h"

namespace
{

std::string ActiveJson(const std::string& schema = agentic_ref::kSchema,
                       const std::string& body_order = agentic_ref::kBodyOrder,
                       double stale_ms = 10.0)
{
  std::ostringstream out;
  out << "{\"ok\":true,\"active\":true,\"schema\":\"" << schema
      << "\",\"body_order\":\"" << body_order
      << "\",\"id\":\"selftest\",\"frame\":1,\"frames\":2,\"time_s\":0.04,"
      << "\"fps\":25,\"stale_ms\":" << stale_ms << ",\"p\":[";
  for (int i = 0; i < agentic_ref::kBodyCount; ++i)
  {
    if (i) out << ',';
    out << '[' << 0.01 * i << ',' << 0.02 * i << ',' << 1.0 + 0.01 * i << ']';
  }
  out << "],\"q\":[";
  for (int i = 0; i < agentic_ref::kBodyCount; ++i)
  {
    if (i) out << ',';
    out << "[1,0,0,0]";
  }
  out << "],\"c\":[true,0],\"com\":[0.1,0.2,1.1],\"comv\":[0,0,0]}";
  return out.str();
}

bool Expect(bool condition, const char* message)
{
  if (!condition)
  {
    std::cerr << "FAIL: " << message << std::endl;
    return false;
  }
  return true;
}

std::string ReplaceRequired(std::string text, const std::string& from, const std::string& to)
{
  std::size_t pos = text.find(from);
  if (pos != std::string::npos)
  {
    text.replace(pos, from.size(), to);
  }
  return text;
}

}  // namespace

int main()
{
  bool pass = true;

  auto inactive = agentic_ref::ParseReferenceFrameJson("{\"ok\":true,\"active\":false}");
  pass &= Expect(inactive.ok && !inactive.active, "inactive frame parses");

  auto active = agentic_ref::ParseReferenceFrameJson(ActiveJson());
  pass &= Expect(active.ok && active.active, "active frame parses");
  pass &= Expect(active.frame.p[26][2] > 1.0, "body positions parsed");
  pass &= Expect(active.frame.c[0] && !active.frame.c[1], "contacts parsed");

  auto bad_schema = agentic_ref::ParseReferenceFrameJson(ActiveJson("BAD"));
  pass &= Expect(!bad_schema.ok, "schema mismatch rejected");

  auto bad_order = agentic_ref::ParseReferenceFrameJson(ActiveJson(agentic_ref::kSchema, "other"));
  pass &= Expect(!bad_order.ok, "body_order mismatch rejected");

  pass &= Expect(!agentic_ref::ParseReferenceFrameJson(
                     ReplaceRequired(ActiveJson(), "\"frame\":1", "\"frame\":NaN")).ok,
                 "non-finite frame rejected");
  pass &= Expect(!agentic_ref::ParseReferenceFrameJson(
                     ReplaceRequired(ActiveJson(), "\"frames\":2", "\"frames\":Inf")).ok,
                 "non-finite frames rejected");
  pass &= Expect(!agentic_ref::ParseReferenceFrameJson(
                     ReplaceRequired(ActiveJson(), "\"time_s\":0.04", "\"time_s\":nan")).ok,
                 "non-finite time rejected");
  pass &= Expect(!agentic_ref::ParseReferenceFrameJson(
                     ReplaceRequired(ActiveJson(), "\"fps\":25", "\"fps\":inf")).ok,
                 "non-finite fps rejected");
  pass &= Expect(!agentic_ref::ParseReferenceFrameJson(
                     ReplaceRequired(ActiveJson(), "\"stale_ms\":10", "\"stale_ms\":-inf")).ok,
                 "non-finite stale rejected");
  pass &= Expect(!agentic_ref::ParseReferenceFrameJson(
                     ReplaceRequired(ActiveJson(), "[0,0,1]", "[nan,0,1]")).ok,
                 "non-finite p rejected");
  pass &= Expect(!agentic_ref::ParseReferenceFrameJson(
                     ReplaceRequired(ActiveJson(), "[1,0,0,0]", "[1,inf,0,0]")).ok,
                 "non-finite q rejected");
  pass &= Expect(!agentic_ref::ParseReferenceFrameJson(
                     ReplaceRequired(ActiveJson(), "\"com\":[0.1,0.2,1.1]",
                                     "\"com\":[0.1,nan,1.1]")).ok,
                 "non-finite com rejected");
  pass &= Expect(!agentic_ref::ParseReferenceFrameJson(
                     ReplaceRequired(ActiveJson(), "\"comv\":[0,0,0]",
                                     "\"comv\":[0,inf,0]")).ok,
                 "non-finite comv rejected");

  const std::string http_body = "{\"ok\":true,\"active\":false}";
  std::string http_response =
      "HTTP/1.1 200 OK\r\nContent-Length: " + std::to_string(http_body.size()) +
      "\r\nConnection: keep-alive\r\n\r\n" + http_body +
      "HTTP/1.1 500 unused\r\n\r\n";
  std::string parsed_body;
  pass &= Expect(agentic_ref::internal::ExtractHttpBody(http_response, false, &parsed_body) ==
                     agentic_ref::internal::HttpParseStatus::kComplete &&
                 parsed_body == http_body,
                 "Content-Length keep-alive response parses without EOF");
  parsed_body.clear();
  pass &= Expect(agentic_ref::internal::ExtractHttpBody(
                     "HTTP/1.1 200 OK\r\nContent-Length: 5\r\n\r\nab", false, &parsed_body) ==
                     agentic_ref::internal::HttpParseStatus::kIncomplete,
                 "partial Content-Length response remains incomplete");
  parsed_body.clear();
  pass &= Expect(agentic_ref::internal::ExtractHttpBody(
                     "HTTP/1.0 200 OK\r\n\r\n" + http_body, true, &parsed_body) ==
                     agentic_ref::internal::HttpParseStatus::kComplete &&
                 parsed_body == http_body,
                 "close-delimited response remains supported");

  agentic_ref::ReferenceFrameCache cache;
  cache.StoreInactive();
  pass &= Expect(!cache.LatestFresh(std::chrono::milliseconds(250), 250.0).has_value(),
                 "inactive cache hides frame");
  cache.StoreFrame(active.frame);
  pass &= Expect(cache.LatestFresh(std::chrono::milliseconds(250), 250.0).has_value(),
                 "fresh cache returns frame");
  pass &= Expect(cache.LatestFresh(std::chrono::milliseconds(10), 250.0).has_value(),
                 "fresh cached frame remains visible without StoreInactive");
  std::this_thread::sleep_for(std::chrono::milliseconds(3));
  pass &= Expect(!cache.LatestFresh(std::chrono::milliseconds(1), 250.0).has_value(),
                 "local stale timeout hides cached frame");

  auto stale = agentic_ref::ParseReferenceFrameJson(ActiveJson(agentic_ref::kSchema,
                                                              agentic_ref::kBodyOrder, 300.0));
  cache.StoreFrame(stale.frame);
  pass &= Expect(!cache.LatestFresh(std::chrono::milliseconds(250), 250.0).has_value(),
                 "packet stale hides frame");

  mjvScene scene;
  mjv_defaultScene(&scene);
  mjv_makeScene(nullptr, &scene, 100);
  std::size_t added = agentic_ref::AppendGhostOverlay(active.frame, &scene);
  pass &= Expect(added == 56 && scene.ngeom == 56, "overlay expected geom count");
  mjv_freeScene(&scene);

  mjv_defaultScene(&scene);
  mjv_makeScene(nullptr, &scene, 10);
  added = agentic_ref::AppendGhostOverlay(active.frame, &scene);
  pass &= Expect(added == 10 && scene.ngeom == 10, "overlay respects maxgeom cap");
  mjv_freeScene(&scene);

  auto defensive = active.frame;
  defensive.p[1] = defensive.p[0];
  defensive.p[2][0] = std::numeric_limits<double>::infinity();
  defensive.com[1] = std::numeric_limits<double>::quiet_NaN();
  mjv_defaultScene(&scene);
  mjv_makeScene(nullptr, &scene, 100);
  added = agentic_ref::AppendGhostOverlay(defensive, &scene);
  pass &= Expect(added == 51 && scene.ngeom == 51,
                 "overlay skips non-finite and degenerate geoms");
  mjv_freeScene(&scene);

  if (pass)
  {
    std::cout << "reference_frame_selftest: PASS" << std::endl;
    return 0;
  }
  return 1;
}
