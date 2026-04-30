# Phase 0c — System Audit Checklist

One row per in-tree system under [gz-sim/src/systems/](../../src/systems/).
Each row is advanced by the automated audit
([0c-audit.py](../../tools/0c-audit.py)) and a manual review.

Status values:
- `pending` — not yet reviewed
- `auto-green` — automated audit found no offending patterns
- `auto-flag` — automated audit flagged a potential issue, needs manual look
- `manual-green` — manual review confirmed archetype-safe
- `fixed` — had issues, fix merged
- `deferred` — issue found, fix tracked separately

Run `python3 tools/0c-audit.py --update-checklist` to regenerate the
`auto-*` columns.

| System | Status | Notes |
|---|---|---|
| ackermann_steering | auto-green | |
| acoustic_comms | auto-green | |
| advanced_lift_drag | auto-green | |
| air_pressure | auto-green | |
| air_speed | auto-green | |
| altimeter | auto-green | |
| apply_joint_force | auto-green | |
| apply_link_wrench | auto-green | |
| battery_plugin | auto-green | |
| breadcrumbs | auto-green | |
| buoyancy | auto-green | |
| buoyancy_engine | auto-green | |
| camera_video_recorder | auto-green | |
| collada_world_exporter | auto-green | |
| comms_endpoint | auto-green | |
| contact | auto-green | |
| detachable_joint | auto-green | |
| diff_drive | auto-green | Manually verified: no cross-phase cached pointers. |
| drive_to_pose_controller | auto-green | |
| dvl | auto-green | |
| elevator | auto-green | |
| entity_semantics | auto-green | |
| environmental_sensor_system | auto-green | |
| environment_preload | auto-green | |
| follow_actor | auto-green | |
| force_torque | auto-green | |
| free_space_explorer | auto-green | |
| hydrodynamics | auto-green | |
| imu | auto-green | |
| joint_controller | auto-green | Manually verified. |
| joint_position_controller | auto-green | |
| joint_state_publisher | auto-green | |
| joint_trajectory_controller | auto-green | |
| kinetic_energy_monitor | auto-green | |
| label | auto-green | |
| lens_flare | auto-green | |
| lift_drag | auto-green | |
| log | auto-green | |
| log_video_recorder | auto-green | |
| logical_audio_sensor_plugin | auto-green | |
| logical_camera | auto-green | |
| magnetometer | auto-green | |
| mecanum_drive | auto-green | |
| mesh_uri | auto-green | |
| model_photo_shoot | auto-green | |
| multicopter_control | auto-green | |
| multicopter_motor_model | auto-green | |
| navsat | auto-green | |
| odometry_publisher | auto-green | Manually verified. |
| optical_tactile_plugin | auto-green | |
| particle_emitter | auto-green | |
| performer_detector | auto-green | |
| physics | manual-green | Audited 2026-04-29: all `components::*` reads are local callback parameters or local vars inside `Each` callbacks; no member pointer caching. Member fields are gz-physics handles (EnginePtr/WorldPtr/ModelPtr/etc.), not gz-sim component pointers. |
| pose_publisher | auto-green | Manually verified. |
| projector | auto-green | |
| rate_control | auto-green | |
| reset_system | auto-green | |
| rf_comms | auto-green | |
| rgbd_camera_controller | auto-green | |
| scene_broadcaster | manual-green | Audited 2026-04-29: every `Component<T>(e)` is a local var inside Configure or an Each callback parameter; no member-cached component pointers. Serialization round-trip uses State/SetState messages, not retained pointers. |
| sensors | manual-green | Audited 2026-04-29: only member pointers are gz-sensors `CameraSensor*` (gz-rendering objects, not gz-sim components) and `EventManager*`; component reads are all local-scope. Render-thread interactions go through gz-sensors not the ECM. |
| shader_param | auto-green | |
| spacecraft_thruster_model | auto-green | |
| spherical_coordinates | auto-green | |
| tethered_drone | auto-green | |
| thruster | auto-green | |
| touch_plugin | auto-green | |
| tracked_vehicle | auto-green | |
| triggered_publisher | auto-green | |
| user_commands | auto-green | |
| velocity_control | auto-green | |
| wheel_slip | auto-green | |
| wind_effects | auto-green | |

## Aggregate status

| Bucket | Count |
|---|---|
| `auto-green` | 72 |
| `manual-green` | 7 (diff_drive, joint_controller, odometry_publisher, pose_publisher, physics, sensors, scene_broadcaster) |

The three §6 "heavy ECM user" systems were audited 2026-04-29 — all
clean. No system in the in-tree set retains component pointers across
phase boundaries.
