#include "ghost_scene_overlay.h"

#include <array>
#include <cmath>
#include <mutex>

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

constexpr float kGhostModelAlpha = 0.22f;

bool HasGeomRoom(const mjvScene* scene)
{
  return scene && scene->geoms && scene->ngeom < scene->maxgeom;
}

bool IsFiniteVec3(const std::array<double, 3>& v)
{
  return std::isfinite(v[0]) && std::isfinite(v[1]) && std::isfinite(v[2]);
}

bool IsFiniteQuat(const std::array<double, 4>& q)
{
  return std::isfinite(q[0]) && std::isfinite(q[1]) &&
         std::isfinite(q[2]) && std::isfinite(q[3]);
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

double QuatNorm(const std::array<double, 4>& q)
{
  return std::sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
}

std::array<double, 4> NormalizeQuat(const std::array<double, 4>& q)
{
  const double norm = QuatNorm(q);
  if (!std::isfinite(norm) || norm <= 1e-12)
  {
    return {1.0, 0.0, 0.0, 0.0};
  }
  return {q[0] / norm, q[1] / norm, q[2] / norm, q[3] / norm};
}

double YawFromQuat(const std::array<double, 4>& q_in)
{
  const auto q = NormalizeQuat(q_in);
  const double w = q[0];
  const double x = q[1];
  const double y = q[2];
  const double z = q[3];
  return std::atan2(2.0 * (w * z + x * y),
                    1.0 - 2.0 * (y * y + z * z));
}

std::array<double, 4> YawQuat(double yaw)
{
  return {std::cos(0.5 * yaw), 0.0, 0.0, std::sin(0.5 * yaw)};
}

std::array<double, 3> TransformPoint(const std::array<double, 3>& p,
                                     const GhostFrameTransform& transform)
{
  if (!transform.enabled)
  {
    return p;
  }
  const double c = std::cos(transform.yaw);
  const double s = std::sin(transform.yaw);
  return {
      c * p[0] - s * p[1] + transform.translation[0],
      s * p[0] + c * p[1] + transform.translation[1],
      p[2] + transform.translation[2],
  };
}

std::array<double, 4> TransformQuat(const std::array<double, 4>& q_in,
                                    const GhostFrameTransform& transform)
{
  const auto q = NormalizeQuat(q_in);
  if (!transform.enabled)
  {
    return q;
  }
  const auto yaw_q = YawQuat(transform.yaw);
  mjtNum out[4];
  const mjtNum lhs[4] = {yaw_q[0], yaw_q[1], yaw_q[2], yaw_q[3]};
  const mjtNum rhs[4] = {q[0], q[1], q[2], q[3]};
  mju_mulQuat(out, lhs, rhs);
  return NormalizeQuat({out[0], out[1], out[2], out[3]});
}

std::array<double, 3> RotateYawOnly(const std::array<double, 3>& p, double yaw)
{
  const double c = std::cos(yaw);
  const double s = std::sin(yaw);
  return {c * p[0] - s * p[1], s * p[0] + c * p[1], p[2]};
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

std::vector<GhostMeshGeom> BuildEt1VisualMeshGeomsUncached(const mjModel* model)
{
  std::vector<GhostMeshGeom> geoms;
  if (!model)
  {
    return geoms;
  }

  std::vector<int> body_to_ref_index(model->nbody, -1);
  for (int i = 0; i < kBodyCount; ++i)
  {
    const int body_id = mj_name2id(model, mjOBJ_BODY, kEt1BodyNames[i]);
    if (body_id >= 0 && body_id < model->nbody)
    {
      body_to_ref_index[body_id] = i;
    }
  }

  for (int geom_id = 0; geom_id < model->ngeom; ++geom_id)
  {
    if (model->geom_type[geom_id] != mjGEOM_MESH ||
        model->geom_group[geom_id] != 1)
    {
      continue;
    }
    const int body_id = model->geom_bodyid[geom_id];
    if (body_id < 0 || body_id >= model->nbody)
    {
      continue;
    }
    const int body_index = body_to_ref_index[body_id];
    const int mesh_id = model->geom_dataid[geom_id];
    if (body_index < 0 || mesh_id < 0)
    {
      continue;
    }

    GhostMeshGeom geom;
    geom.body_index = body_index;
    geom.body_id = body_id;
    geom.geom_id = geom_id;
    geom.dataid = 2 * mesh_id;
    geom.texcoord = model->mesh_texcoordadr[mesh_id] >= 0 ? 1 : 0;
    geom.rbound = static_cast<float>(model->geom_rbound[geom_id]);
    for (int i = 0; i < 3; ++i)
    {
      geom.size[i] = model->geom_size[3 * geom_id + i];
      geom.local_pos[i] = model->geom_pos[3 * geom_id + i];
    }
    for (int i = 0; i < 4; ++i)
    {
      geom.local_quat[i] = model->geom_quat[4 * geom_id + i];
      geom.rgba[i] = model->geom_rgba[4 * geom_id + i];
    }
    geoms.push_back(geom);
  }
  return geoms;
}

std::vector<GhostMeshGeom> CachedEt1VisualMeshGeoms(const mjModel* model)
{
  struct Cache
  {
    std::mutex mutex;
    const mjModel* model = nullptr;
    int nbody = -1;
    int ngeom = -1;
    std::vector<GhostMeshGeom> geoms;
  };
  static Cache cache;

  std::lock_guard<std::mutex> lock(cache.mutex);
  if (model != cache.model ||
      (model && (model->nbody != cache.nbody || model->ngeom != cache.ngeom)))
  {
    cache.model = model;
    cache.nbody = model ? model->nbody : -1;
    cache.ngeom = model ? model->ngeom : -1;
    cache.geoms = BuildEt1VisualMeshGeomsUncached(model);
  }
  return cache.geoms;
}

void AddMeshGeom(mjvScene* scene, const GhostMeshGeom& desc,
                 const ReferenceFrame& frame, std::size_t* added)
{
  if (desc.body_index < 0 || desc.body_index >= kBodyCount ||
      !IsFiniteVec3(frame.p[desc.body_index]) ||
      !IsFiniteQuat(frame.q[desc.body_index]))
  {
    return;
  }

  mjtNum body_pos[3] = {
      frame.p[desc.body_index][0],
      frame.p[desc.body_index][1],
      frame.p[desc.body_index][2],
  };
  const auto body_quat_norm = NormalizeQuat(frame.q[desc.body_index]);
  mjtNum body_quat[4] = {
      body_quat_norm[0],
      body_quat_norm[1],
      body_quat_norm[2],
      body_quat_norm[3],
  };
  mjtNum world_pos[3];
  mjtNum world_quat[4];
  mju_mulPose(world_pos, world_quat,
              body_pos, body_quat, desc.local_pos, desc.local_quat);

  if (!std::isfinite(world_pos[0]) || !std::isfinite(world_pos[1]) ||
      !std::isfinite(world_pos[2]) || !std::isfinite(world_quat[0]) ||
      !std::isfinite(world_quat[1]) || !std::isfinite(world_quat[2]) ||
      !std::isfinite(world_quat[3]))
  {
    return;
  }

  mjvGeom* geom = NextGeom(scene, added);
  if (!geom)
  {
    return;
  }

  mjtNum mat[9];
  mju_quat2Mat(mat, world_quat);
  float rgba[4] = {
      desc.rgba[0],
      desc.rgba[1],
      desc.rgba[2],
      kGhostModelAlpha,
  };
  mjv_initGeom(geom, mjGEOM_MESH, desc.size, world_pos, mat, rgba);
  geom->dataid = desc.dataid;
  geom->objtype = mjOBJ_GEOM;
  geom->objid = desc.geom_id;
  geom->category = mjCAT_DECOR;
  geom->matid = -1;
  geom->texcoord = desc.texcoord;
  geom->segid = -1;
  geom->modelrbound = desc.rbound;
  geom->transparent = 1;
}

std::size_t AppendFullModelGhost(const ReferenceFrame& frame, mjvScene* scene,
                                 const mjModel* model)
{
  if (!model || !scene)
  {
    return 0;
  }

  std::size_t added = 0;
  const auto mesh_geoms = CachedEt1VisualMeshGeoms(model);
  for (const auto& geom : mesh_geoms)
  {
    AddMeshGeom(scene, geom, frame, &added);
  }
  return added;
}

}  // namespace

std::vector<GhostMeshGeom> BuildEt1VisualMeshGeoms(const mjModel* model)
{
  return BuildEt1VisualMeshGeomsUncached(model);
}

GhostFrameTransform ComputeYawTranslationAlignment(
    const std::array<double, 3>& ref_pos,
    const std::array<double, 4>& ref_quat,
    const std::array<double, 3>& live_pos,
    const std::array<double, 4>& live_quat)
{
  GhostFrameTransform transform;
  if (!IsFiniteVec3(ref_pos) || !IsFiniteQuat(ref_quat) ||
      !IsFiniteVec3(live_pos) || !IsFiniteQuat(live_quat))
  {
    return transform;
  }

  transform.enabled = true;
  transform.yaw = YawFromQuat(live_quat) - YawFromQuat(ref_quat);
  const auto rotated_ref = RotateYawOnly(ref_pos, transform.yaw);
  transform.translation = {
      live_pos[0] - rotated_ref[0],
      live_pos[1] - rotated_ref[1],
      live_pos[2] - rotated_ref[2],
  };
  return transform;
}

ReferenceFrame TransformReferenceFrame(const ReferenceFrame& frame,
                                       const GhostFrameTransform& transform)
{
  if (!transform.enabled)
  {
    return frame;
  }

  ReferenceFrame out = frame;
  for (int i = 0; i < kBodyCount; ++i)
  {
    out.p[i] = TransformPoint(frame.p[i], transform);
    out.q[i] = TransformQuat(frame.q[i], transform);
  }
  out.com = TransformPoint(frame.com, transform);
  return out;
}

std::size_t AppendGhostOverlay(const ReferenceFrame& frame, mjvScene* scene,
                               const GhostOverlayOptions& options,
                               const mjModel* model,
                               const GhostFrameTransform& transform)
{
  if (!frame.active || frame.schema != kSchema || frame.body_order != kBodyOrder || !scene)
  {
    return 0;
  }

  std::size_t added = 0;
  const ReferenceFrame draw_frame = TransformReferenceFrame(frame, transform);
  const float joint_rgba[4] = {0.1f, 0.85f, 1.0f, 0.36f};
  const float limb_rgba[4] = {0.1f, 0.85f, 1.0f, 0.24f};
  const float com_rgba[4] = {1.0f, 0.85f, 0.05f, 0.55f};
  const float foot_idle_rgba[4] = {0.2f, 0.9f, 0.35f, 0.22f};
  const float foot_contact_rgba[4] = {1.0f, 0.25f, 0.1f, 0.58f};

  added += AppendFullModelGhost(draw_frame, scene, model);

  for (const auto& edge : kEdges)
  {
    AddCapsule(scene, draw_frame.p[edge.first], draw_frame.p[edge.second],
               options.limb_radius, limb_rgba, &added);
  }

  for (const auto& p : draw_frame.p)
  {
    AddSphere(scene, p, options.body_radius, joint_rgba, &added);
  }

  AddSphere(scene, draw_frame.com, options.com_radius, com_rgba, &added);

  const float* left_rgba = frame.c[0] ? foot_contact_rgba : foot_idle_rgba;
  const float* right_rgba = frame.c[1] ? foot_contact_rgba : foot_idle_rgba;
  AddSphere(scene, draw_frame.p[6], options.foot_radius, left_rgba, &added);
  AddSphere(scene, draw_frame.p[12], options.foot_radius, right_rgba, &added);

  return added;
}

}  // namespace agentic_ref
