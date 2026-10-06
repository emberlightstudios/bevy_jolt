# bevy_jolt

Safe Bevy integration for Jolt Physics. Raw FFI lives in `jolt_sys`; this
crate owns world lifetime, the safe body API, and the Bevy schedule wiring.

## The one rule: two schedules

- Everything in `FixedUpdate` runs **before** the sim steps. Write drives here.
- Everything in `JoltStep` runs **after**. The step plus all readbacks live
  here, slotted between `FixedUpdate` and `FixedPostUpdate`.
- By `FixedPostUpdate`, the sim has always stepped. Read results here or later.

You never order against the step. The schedules run in order, so no
per-system `.before()` / `.after()` can get it wrong. (Tests drive schedules
by hand, so they run `FixedUpdate` then `JoltStep` explicitly.)

## Bodies: describe, bake, sync

Add three things to an entity; the plugin creates the Jolt body and keeps it
in sync:

```rust
commands.spawn((
    Transform::from_xyz(0.0, 5.0, 0.0), // spawn pose
    JoltBody::dynamic(0),                // motion + layer + material
    JoltShape::sphere(0.5),              // geometry (shared via Arc)
));
```

- `JoltBody::dynamic(0)` / `::fixed(0)` / `::kinematic(0)` — motion type +
  collision layer. Builder methods: `with_density`, `with_gravity`,
  `with_friction` (0 ice – 1+ rubber), `with_restitution` (0 dead – 1
  superball), `with_ccd` (fast bullets/swords).
- Bake inserts `JoltBodyId` (read it, never write it) plus
  `PreviousBodyTransform` (render interpolation state).
- Post-step, the sync copies pose into `Transform` and writes measured
  velocities (below). Sleeping bodies are skipped — frozen means frozen.

Despawn the entity and the Jolt body is destroyed. Joints on it cascade first
(constraint removals are synchronous, the solver never sees a dead endpoint).

## Shapes

| Constructor | Notes |
|---|---|
| `box_shape(half)` / `sphere(r)` / `capsule(hh, r)` / `cylinder(hh, r)` | Primitives, any motion |
| `tapered_cylinder(hh, top, bottom)` / `tapered_capsule(...)` | Primitives, any motion |
| `compound(parts)` | Box/sphere/capsule parts, ≤16, any motion |
| `hull(points)` | Shrink-wrap from a point soup, any motion. Dents filled |
| `mesh(verts, tris)` | Exact triangles, **static only** (bake panics otherwise). Single-sided: wind faces toward the player |
| `heightfield(heights, width, cell)` | `width²` heights on a grid, **static only**. Grid lookup beats tree walk at scale; edges are cliffs |
| `plane(normal, constant)` | Infinite ground |

`JoltShape` holds an `Arc<PhysicsShape>`: the component and the debug table
share one allocation. Terrain-sized geometry lives once, not twice.

## Driving bodies

**Fire-and-forget triggers** (no archetype churn, missing bodies skipped):

| Trigger | Effect |
|---|---|
| `JoltImpulse::linear(e, v)` / `.angular(e, v)` | Instant kick, framerate-independent (`impulse / mass = Δv`) |
| `JoltSetVelocity::linear(e, v)` / `.stop(e)` | Velocity overwrite (zero stops that axis) |
| `JoltTeleport { position, rotation }` | Pose move, keeps momentum unless stopped after |
| `JoltSleep { body_entity }` | Freeze in place. Still solid, wakes on contact |
| `JoltWake { body_entity }` | Rejoin next step, velocities intact |
| `JoltSetMotion { body_entity, motion }` | Live static/kinematic/dynamic flip. Static is unwakeable; flips `JoltBody.motion` too, never half-synced |

**Held components** (attach once, write fields per frame, remove to stop):

| Component | Effect |
|---|---|
| `JoltLinearForce` / `JoltAngularForce` | Continuous push, scaled by dt internally. Use for held shoves |
| `JoltLinearVelocity` / `JoltAngularVelocity` | Overwrite every tick. Overrules gravity/friction — for driven bodies only, never for natural motion |
| `JoltKinematicTarget { position, rotation }` | Kinematic pose drive via `MoveKinematic`: Jolt derives velocity so it shoves dynamics aside |

Linear and angular halves are separate components so driving movement never
wipes out spin. Impulse ≠ one-frame force: forces scale with dt, impulses
don't.

## Reading bodies

- `JoltMeasuredLinearVelocity` / `JoltMeasuredAngularVelocity` — written by
  the crate every tick, **read-only for game code**. Tells what the body is
  actually doing (vs drive components, which say what you asked for).
  Inserted at bake for non-static bodies.
- `JoltSleeping` — marker the activation drain maintains. Sync filters it
  out; debug greys it. React with `Added<JoltSleeping>` /
  `Removed<JoltSleeping>` — no polling, no callbacks in the API.
- `body_snapshot(id)` — one-off position + linear + angular read.
- Never drive from a measured value back into a drive component: the stale
  write fights the sim (set 5 m/s, hit a wall, read 1, drive 1 forever).

## Characters (two kinds, different components)

- `JoltCharacter` + `JoltCharacterVelocity` — virtual character
  (move-and-slide, stairs, stick-to-floor). Velocity is a *request* blended
  with gravity, not an overwrite. Ground state in `JoltCharacterGround`.
  Jump is yours: set upward velocity when grounded.
- `JoltRigidCharacter` + `JoltRigidCharacterVelocity` — a real capsule body
  in the solver: pushes and gets pushed.

Keep the velocity types separate: same field name, different machinery
(solver overwrite vs move-and-slide). Shared components with `With`/`Without`
filters would make misuse silent instead of a compile error.

## Joints, motors, vehicles, ragdolls, soft bodies

- `JoltJoint` + `JointKind` (fixed/distance/hinge/slider/cone/swing-twist/
  six-dof/point/pulley/gear/rack-pinion/path) with `JointSpace::World` or
  `LocalToBodyCom` (center-of-mass relative, NOT Transform origin).
- `JoltMotorDrive(v)` + `JointMotor` — per-tick motor targets, driven by
  `apply_jolt_motor_drives`. Ping-pong by rewriting the value.
- Vehicles (`JoltVehicleDrive`, tracked/wheeled), ragdolls (`JoltRagdoll`),
  soft bodies (`JoltSoftBodyConfig`, cloth/balloon/skinned), buoyancy
  (`JoltWater` resource + `JoltBuoyant` tag; flat surface, per-tick push).
- Sensors: `JoltSensor` at bake (overlaps report, nothing pushes).
- Contacts: `JoltContactAdded` / `JoltContactRemoved` entity events after
  the step. Sleeping sensors still report — sleep skips integration, not
  detection.

## Queries (reads, not writes)

`cast_ray_all`, `collide_point_all`, `overlap_shape_all`, `cast_shape_all`
via `JoltPhysicsWorld`, plus `QueryProbe` shapes. Soft-body vertex reads
(`soft_vertices`, `soft_volume`) are also reads — the only examples that
may touch the world directly. Everything that *moves* something goes
through components.

## Collision layers and gravity

- `CollisionLayers`: declare teams up front, wire who-hits-who. Layer 0
  everywhere in examples; real games isolate world/character/sensor teams.
- Restitution combines by **max** (floor 0 + ball 0.9 bounces at 0.9);
  friction by **geometric mean** (either side 0 kills grip).
- `JoltStartupGravity` / world gravity: global pull, per-body
  `gravity_factor` scales it (0 floats, 2 double-pulls).

## Debug outlines (`JoltDebugPlugin`)

Green = awake, grey = sleeping (from the marker, no per-frame poll).
Boxes/spheres as solids; capsules/cylinders/tapered as line outlines; hulls
as computed wireframe faces (brute-force triples, debug counts only); meshes
and heightfields as wireframes. Characters draw their capsules (they never
enter the body table).

## Tests and examples

- Tests run single-threaded (`--test-threads=1`): parallel `JoltWorld`s
  crash on Jolt's shared job plumbing. The harness advances `Time<Fixed>`
  by one timestep per tick — `run_schedule(FixedUpdate)` alone integrates
  delta = 0.
- Examples are visual proofs with console assertions, not fixtures:
  `bodies_*` for shapes and drives, `constraint_*` per joint,
  `character_walk` for both character types, `bodies_terrain` for
  heightfields, `spatial_queries` for query shapes.
- Dropped-ball pattern: spawn high, tick N times, assert rest pose. Copy
  `tests/hull_mesh.rs` for new shapes, `tests/sleep_motion.rs` for
  triggers.

## What this crate is not

A game framework. No player controller, no input, no health, no inventory.
Game-specific logic lives in game crates (or examples, when demo-specific).
Keep it unopinionated: if different games would implement it differently, it
doesn't belong here.
