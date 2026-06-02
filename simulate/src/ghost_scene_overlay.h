#pragma once

#include <cstddef>
#include <array>
#include <vector>

#include <mujoco/mujoco.h>

#include "reference_frame_cache.h"

namespace agentic_ref
{

struct GhostOverlayOptions
{
  float body_radius = 0.035f;
  float limb_radius = 0.018f;
  float com_radius = 0.055f;
  float foot_radius = 0.055f;
};

inline constexpr std::array<const char*, kBodyCount> kEt1BodyNames = {{
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

struct GhostFrameTransform
{
  bool enabled = false;
  double yaw = 0.0;
  std::array<double, 3> translation{};
};

struct GhostMeshGeom
{
  int body_index = -1;
  int body_id = -1;
  int geom_id = -1;
  int dataid = -1;
  int texcoord = 0;
  float rgba[4] = {1.0f, 1.0f, 1.0f, 1.0f};
  float rbound = 0.0f;
  mjtNum size[3] = {0, 0, 0};
  mjtNum local_pos[3] = {0, 0, 0};
  mjtNum local_quat[4] = {1, 0, 0, 0};
};

std::vector<GhostMeshGeom> BuildEt1VisualMeshGeoms(const mjModel* model);

GhostFrameTransform ComputeYawTranslationAlignment(
    const std::array<double, 3>& ref_pos,
    const std::array<double, 4>& ref_quat,
    const std::array<double, 3>& live_pos,
    const std::array<double, 4>& live_quat);

ReferenceFrame TransformReferenceFrame(const ReferenceFrame& frame,
                                       const GhostFrameTransform& transform);

std::size_t AppendGhostOverlay(const ReferenceFrame& frame, mjvScene* scene,
                               const GhostOverlayOptions& options = {},
                               const mjModel* model = nullptr,
                               const GhostFrameTransform& transform = {});

}  // namespace agentic_ref
