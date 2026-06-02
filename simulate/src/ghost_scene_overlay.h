#pragma once

#include <cstddef>

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

std::size_t AppendGhostOverlay(const ReferenceFrame& frame, mjvScene* scene,
                               const GhostOverlayOptions& options = {});

}  // namespace agentic_ref
