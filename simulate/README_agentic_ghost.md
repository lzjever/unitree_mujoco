# Agentic Reference Ghost Viewer

`unitree_mujoco_agentic_ghost` is an opt-in simulator binary that renders a
reference ET1 ghost from the tracker hidden endpoint:

```bash
GET http://127.0.0.1:8083/_sim/reference_frame
```

The original `unitree_mujoco` target does not compile or expose the ghost
viewer code path. Use only `unitree_mujoco_agentic_ghost` for this overlay.

## Build

```bash
cmake --build build --target unitree_mujoco
cmake --build build --target unitree_mujoco_agentic_ghost
cmake --build build --target reference_frame_selftest
./build/reference_frame_selftest
```

## Run

```bash
./build/unitree_mujoco_agentic_ghost
```

The ghost target defaults to these compiled-in settings:

```yaml
ghost_ref_url: "http://127.0.0.1:8083/_sim/reference_frame"
ghost_ref_poll_hz: 25.0
ghost_ref_timeout_ms: 15
ghost_ref_stale_ms: 250
```

`unitree_mujoco_agentic_ghost` also accepts `--ghost_ref_enable 0`,
`--ghost_ref_url`, `--ghost_ref_poll_hz`, `--ghost_ref_timeout_ms`, and
`--ghost_ref_stale_ms`. These options are intentionally not present on the
original `unitree_mujoco` binary.

## Behavior

The overlay uses the packet `body_order` `et1_27_v1` directly and does not
write MuJoCo body indices, `qpos`, or `qvel`. For `tdf_ET1.xml`, it maps the
fixed `et1_27_v1` body order to ET1 visual mesh geoms and appends a full
semi-transparent robot model to `mjvScene` immediately before rendering. It
also keeps the original skeleton capsules, joint spheres, COM marker, and foot
contact markers as auxiliary guides.

Press `G` to align the reference root to the live `pelvis_link` using yaw plus
translation. The offset is applied to all later reference body poses, COM,
skeleton guides, and model ghost rendering. Press `G` again to update the
offset. Press `Shift+G` or `Ctrl+G` to clear it.

The default 25 Hz polling rate is low-risk for local visualization. Use 50 or
60 Hz when the reference should visually track a 50 Hz `.trk` more closely.
The viewer keeps only the latest frame; it does not queue, interpolate, or
replay missed frames. Explicit inactive packets hide the ghost immediately.
HTTP, transport, parse, schema, or body-order failures keep the last good frame
visible until `ghost_ref_stale_ms` expires, then hide it.
