#include <cmath>
#include <filesystem>
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

#ifndef UNITREE_MUJOCO_REPO_DIR
#define UNITREE_MUJOCO_REPO_DIR "."
#endif

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

bool ExpectSizeEq(std::size_t actual, std::size_t expected, const char* message)
{
  if (actual != expected)
  {
    std::cerr << "FAIL: " << message << " expected " << expected
              << ", got " << actual << std::endl;
    return false;
  }
  return true;
}

bool Near(double lhs, double rhs, double eps = 1e-9)
{
  return std::fabs(lhs - rhs) <= eps;
}

bool NearVec3(const std::array<double, 3>& lhs, const std::array<double, 3>& rhs,
              double eps = 1e-9)
{
  return Near(lhs[0], rhs[0], eps) && Near(lhs[1], rhs[1], eps) &&
         Near(lhs[2], rhs[2], eps);
}

std::array<double, 4> YawQuat(double yaw)
{
  return {std::cos(0.5 * yaw), 0.0, 0.0, std::sin(0.5 * yaw)};
}

double YawFromQuat(const std::array<double, 4>& q)
{
  return std::atan2(2.0 * (q[0] * q[3] + q[1] * q[2]),
                    1.0 - 2.0 * (q[2] * q[2] + q[3] * q[3]));
}

std::array<double, 3> ApplyYawTranslation(const std::array<double, 3>& p,
                                          double yaw,
                                          const std::array<double, 3>& t)
{
  const double c = std::cos(yaw);
  const double s = std::sin(yaw);
  return {
      c * p[0] - s * p[1] + t[0],
      s * p[0] + c * p[1] + t[1],
      p[2] + t[2],
  };
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

  pass &= Expect(agentic_ref::kEt1BodyNames.size() == agentic_ref::kBodyCount,
                 "body mapping has expected length");
  const std::array<const char*, agentic_ref::kBodyCount> expected_body_names = {{
      "pelvis_link",
      "left_hip_pitch_link",
      "left_hip_roll_link",
      "left_hip_yaw_link",
      "left_knee_link",
      "left_ankle_pitch_link",
      "left_ankle_roll_link",
      "right_hip_pitch_link",
      "right_hip_roll_link",
      "right_hip_yaw_link",
      "right_knee_link",
      "right_ankle_pitch_link",
      "right_ankle_roll_link",
      "waist_roll_link",
      "waist_yaw_link",
      "left_shoulder_pitch_link",
      "left_shoulder_roll_link",
      "left_shoulder_yaw_link",
      "left_elbow_link",
      "left_wrist_roll_link",
      "right_shoulder_pitch_link",
      "right_shoulder_roll_link",
      "right_shoulder_yaw_link",
      "right_elbow_link",
      "right_wrist_roll_link",
      "head_pitch_link",
      "head_yaw_link",
  }};
  for (int i = 0; i < agentic_ref::kBodyCount; ++i)
  {
    pass &= Expect(std::string(agentic_ref::kEt1BodyNames[i]) == expected_body_names[i],
                   "body mapping name matches fixed et1_27_v1 order");
  }

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

  const std::filesystem::path robot_xml =
      std::filesystem::path(UNITREE_MUJOCO_REPO_DIR) /
      "unitree_robots" / "tdf_ET1" / "tdf_ET1.xml";
  char load_error[1024] = "";
  mjModel* et1_model = mj_loadXML(robot_xml.c_str(), nullptr, load_error, sizeof(load_error));
  pass &= Expect(et1_model != nullptr, "tdf_ET1.xml loads");
  if (et1_model)
  {
    for (int i = 0; i < agentic_ref::kBodyCount; ++i)
    {
      const int body_id = mj_name2id(et1_model, mjOBJ_BODY, agentic_ref::kEt1BodyNames[i]);
      pass &= Expect(body_id >= 0, "mapped ET1 body resolves in tdf_ET1.xml");
    }

    const auto visual_geoms = agentic_ref::BuildEt1VisualMeshGeoms(et1_model);
    pass &= ExpectSizeEq(visual_geoms.size(), 40,
                         "visual mesh geom count for mapped ET1 bodies");

    mjv_defaultScene(&scene);
    mjv_makeScene(et1_model, &scene, 200);
    added = agentic_ref::AppendGhostOverlay(active.frame, &scene, {}, et1_model);
    bool found_mesh = false;
    bool found_transparent_mesh = false;
    for (int i = 0; i < scene.ngeom; ++i)
    {
      if (scene.geoms[i].type == mjGEOM_MESH)
      {
        found_mesh = true;
        if (scene.geoms[i].rgba[3] < 1.0f && scene.geoms[i].transparent)
        {
          found_transparent_mesh = true;
        }
      }
    }
    pass &= Expect(added > 56, "full model ghost appends mesh geoms");
    pass &= Expect(found_mesh, "full model ghost includes at least one mesh geom");
    pass &= Expect(found_transparent_mesh, "full model ghost mesh is transparent");
    mjv_freeScene(&scene);

    mjv_defaultScene(&scene);
    mjv_makeScene(et1_model, &scene, 7);
    added = agentic_ref::AppendGhostOverlay(active.frame, &scene, {}, et1_model);
    pass &= Expect(added == 7 && scene.ngeom == 7, "full model overlay respects maxgeom cap");
    mjv_freeScene(&scene);

    mj_deleteModel(et1_model);
  }

  const double ref_yaw = 0.25;
  const double live_yaw = 1.10;
  auto align_frame = active.frame;
  align_frame.p[0] = {1.0, -2.0, 0.7};
  align_frame.q[0] = YawQuat(ref_yaw);
  const std::array<double, 3> live_root = {-0.5, 3.0, 1.2};
  const auto live_quat = YawQuat(live_yaw);
  const auto alignment = agentic_ref::ComputeYawTranslationAlignment(
      align_frame.p[0], align_frame.q[0], live_root, live_quat);
  const auto aligned_frame = agentic_ref::TransformReferenceFrame(align_frame, alignment);
  pass &= Expect(alignment.enabled, "yaw+translation alignment produced an offset");
  pass &= Expect(NearVec3(aligned_frame.p[0], live_root, 1e-8),
                 "yaw+translation alignment maps ref root to live root");
  pass &= Expect(Near(YawFromQuat(aligned_frame.q[0]), live_yaw, 1e-8),
                 "yaw+translation alignment maps ref root yaw to live yaw");

  agentic_ref::GhostFrameTransform manual_transform;
  manual_transform.enabled = true;
  manual_transform.yaw = 0.5;
  manual_transform.translation = {0.25, -0.5, 0.75};
  const auto transformed_frame =
      agentic_ref::TransformReferenceFrame(active.frame, manual_transform);
  pass &= Expect(NearVec3(transformed_frame.com,
                         ApplyYawTranslation(active.frame.com,
                                             manual_transform.yaw,
                                             manual_transform.translation)),
                 "transform applies to COM");

  mjv_defaultScene(&scene);
  mjv_makeScene(nullptr, &scene, 100);
  added = agentic_ref::AppendGhostOverlay(active.frame, &scene, {},
                                          nullptr, manual_transform);
  pass &= Expect(added == 56 && scene.ngeom == 56, "transformed skeleton overlay count");
  const int com_geom_index = 26 + agentic_ref::kBodyCount;
  pass &= Expect(Near(scene.geoms[com_geom_index].pos[0], transformed_frame.com[0], 1e-6) &&
                 Near(scene.geoms[com_geom_index].pos[1], transformed_frame.com[1], 1e-6) &&
                 Near(scene.geoms[com_geom_index].pos[2], transformed_frame.com[2], 1e-6),
                 "transform applies to rendered COM marker");
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
