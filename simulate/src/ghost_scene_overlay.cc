#include "ghost_scene_overlay.h"

#include <array>
#include <cmath>

namespace agentic_ref
{
namespace
{

constexpr std::array<std::pair<int, int>, 26> kEdges = {{
    {0, 1}, {1, 2}, {2, 3}, {3, 4}, {4, 5}, {5, 6},
    {0, 7}, {7, 8}, {8, 9}, {9, 10}, {10, 11}, {11, 12},
    {0, 13}, {13, 14},
    {14, 15}, {15, 16}, {16, 17}, {17, 18}, {18, 19},
    {14, 20}, {20, 21}, {21, 22}, {22, 23}, {23, 24},
    {14, 25}, {25, 26},
}};

bool HasGeomRoom(const mjvScene* scene)
{
  return scene && scene->geoms && scene->ngeom < scene->maxgeom;
}

bool IsFiniteVec3(const std::array<double, 3>& v)
{
  return std::isfinite(v[0]) && std::isfinite(v[1]) && std::isfinite(v[2]);
}

bool IsUsableRadius(float radius)
{
  return std::isfinite(radius) && radius > 0.0f;
}

bool IsUsableCapsule(const std::array<double, 3>& from,
                     const std::array<double, 3>& to,
                     float radius)
{
  if (!IsFiniteVec3(from) || !IsFiniteVec3(to) || !IsUsableRadius(radius))
  {
    return false;
  }
  const double dx = to[0] - from[0];
  const double dy = to[1] - from[1];
  const double dz = to[2] - from[2];
  const double length_sq = dx * dx + dy * dy + dz * dz;
  return std::isfinite(length_sq) && length_sq > 1e-12;
}

mjvGeom* NextGeom(mjvScene* scene, std::size_t* added)
{
  if (!HasGeomRoom(scene))
  {
    return nullptr;
  }
  ++(*added);
  return &scene->geoms[scene->ngeom++];
}

void AddSphere(mjvScene* scene, const std::array<double, 3>& pos, float radius,
               const float rgba[4], std::size_t* added)
{
  if (!IsFiniteVec3(pos) || !IsUsableRadius(radius))
  {
    return;
  }
  mjvGeom* geom = NextGeom(scene, added);
  if (!geom)
  {
    return;
  }
  const mjtNum size[3] = {radius, radius, radius};
  const mjtNum p[3] = {pos[0], pos[1], pos[2]};
  mjv_initGeom(geom, mjGEOM_SPHERE, size, p, nullptr, rgba);
}

void AddCapsule(mjvScene* scene, const std::array<double, 3>& from,
                const std::array<double, 3>& to, float radius,
                const float rgba[4], std::size_t* added)
{
  if (!IsUsableCapsule(from, to, radius))
  {
    return;
  }
  mjvGeom* geom = NextGeom(scene, added);
  if (!geom)
  {
    return;
  }
  mjv_initGeom(geom, mjGEOM_CAPSULE, nullptr, nullptr, nullptr, rgba);
  const mjtNum a[3] = {from[0], from[1], from[2]};
  const mjtNum b[3] = {to[0], to[1], to[2]};
  mjv_connector(geom, mjGEOM_CAPSULE, radius, a, b);
}

}  // namespace

std::size_t AppendGhostOverlay(const ReferenceFrame& frame, mjvScene* scene,
                               const GhostOverlayOptions& options)
{
  if (!frame.active || frame.schema != kSchema || frame.body_order != kBodyOrder || !scene)
  {
    return 0;
  }

  std::size_t added = 0;
  const float joint_rgba[4] = {0.1f, 0.85f, 1.0f, 0.36f};
  const float limb_rgba[4] = {0.1f, 0.85f, 1.0f, 0.24f};
  const float com_rgba[4] = {1.0f, 0.85f, 0.05f, 0.55f};
  const float foot_idle_rgba[4] = {0.2f, 0.9f, 0.35f, 0.22f};
  const float foot_contact_rgba[4] = {1.0f, 0.25f, 0.1f, 0.58f};

  for (const auto& edge : kEdges)
  {
    AddCapsule(scene, frame.p[edge.first], frame.p[edge.second],
               options.limb_radius, limb_rgba, &added);
  }

  for (const auto& p : frame.p)
  {
    AddSphere(scene, p, options.body_radius, joint_rgba, &added);
  }

  AddSphere(scene, frame.com, options.com_radius, com_rgba, &added);

  const float* left_rgba = frame.c[0] ? foot_contact_rgba : foot_idle_rgba;
  const float* right_rgba = frame.c[1] ? foot_contact_rgba : foot_idle_rgba;
  AddSphere(scene, frame.p[6], options.foot_radius, left_rgba, &added);
  AddSphere(scene, frame.p[12], options.foot_radius, right_rgba, &added);

  return added;
}

}  // namespace agentic_ref
