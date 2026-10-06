# bevy_jolt

Safe Bevy integration for Jolt Physics. Raw FFI lives in `jolt_sys`; this
crate owns world lifetime, the safe body API, and the Bevy schedule wiring.

## Schedules: where your game code goes

Put game logic in `PreFixedUpdate`, `FixedUpdate`, and/or `PostFixedUpdate`.
The crate guarantees this ordering every tick:

1. Your `FixedUpdate` systems are guaranteed to finish before the simulation steps. Nothing physics-side runs
   yet.
2. `JoltStep` runs next: change-detection pushes first (motion, driven
   velocities), then the sim steps, then all readbacks (poses, measured
   velocities, contacts, sleep markers).
3. Your `PostFixedUpdate` systems run last. The step and every readback are already
   done, so what you read is this tick's result.

So: write drives in `FixedUpdate`, read results in `PostFixedUpdate` (or
later). No manual ordering needed — the schedules already run in that
sequence, so a write can never be missed by ordering. (Tests drive schedules
by hand, so they run `FixedUpdate` then `JoltStep` explicitly — see the
`tick()` helper in `tests/`.)

## Bodies: describe, bake, sync

Add three things to an entity; the plugin creates the Jolt body and keeps it
in sync:

```rust
commands.spawn((
    Transform::from_xyz(0.0, 5.0, 0.0), // spawn pose
    JoltBody::dynamic(0),                // motion type + collision layer
    JoltShape::sphere(0.5),              // geometry (shared via Arc)
));
```

- `JoltBody::dynamic(0)` / `::fixed(0)` / `::kinematic(0)` — motion type +
  collision layer. Builder methods: `with_density`, `with_gravity` (per-body
  gravity multiplier: 0 floats, 2 double-pulls), `with_friction` (0 ice –
  1+ rubber), `with_restitution` (0 dead – 1 superball), `with_ccd` (fast
  bullets/swords).
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
| `compound(parts)` | Box/sphere/capsule parts, at most 16, any motion. Empty rejects at bake |
| `hull(points)` | Convex shrink-wrap from a point soup, any motion. Dents filled — rocks, crates, wreckage |
| `mesh(verts, tris)` | Exact-triangle static scenery, **static only** (bake panics otherwise). Archways, stairs, rubble |
| `heightfield(heights, width, cell)` | `width × width` heights on a grid centered on spawn, **static only** (bake panics otherwise). Cheaper than the equivalent mesh at scale |
| `plane(normal, constant)` | Infinite ground |

`JoltShape` holds an `Arc<PhysicsShape>` filed into the world's debug table
at bake (`file_shape`): the component and the drawer share one allocation.
Terrain-sized geometry lives once, not twice.

## Driving bodies

**Fire-and-forget triggers** (no archetype churn, missing bodies skipped):

| Trigger | Effect |
|---|---|
| `JoltImpulse::linear(e, v)` / `.angular(e, v)` | Instant kick, framerate-independent (`impulse / mass = Δv`) |
| `JoltTeleport { body_entity, target_position, target_rotation }` | Pose move that wakes the body; momentum is preserved unless you overwrite velocity afterwards |

Motion type (`JoltBody.motion`) is a plain component write — no trigger.
Static is solid ground that sleeps forever, kinematic is driven by you via
`JoltKinematicTarget`, dynamic is moved by physics. Write it in `FixedUpdate`
and the sync at the start of the next `JoltStep` pushes it to Jolt before the
sim steps. Only writing back to kinematic/dynamic wakes a staticked body —
writing `sleeping = false` alone won't.

Sleep (`JoltSleeping.sleeping`) is a plain component write too — no trigger.
`true` freezes the body where it stands (still solid, wakes on contact, for
dormant crowds); `false` rejoins next step with velocities intact. Jolt's own
transitions (settling asleep, waking on contact) land in the same field via
the post-step drain, so what you read is always the truth. Both operations
are idempotent, so the drain's echo costs one extra no-op push per flip, then
stops — it cannot loop. Derefs to bool: `if **sleeping` reads it directly.

There is deliberately no velocity trigger: write `JoltLinearVelocity` /
`JoltAngularVelocity` and change detection pushes it at the start of the next
`JoltStep`, before the sim steps (see below). Triggers are for things a
component can't say — kicks, teleports — not for values you own already.

**Held components** (attach once, remove to stop):

| Component | Effect |
|---|---|
| `JoltLinearForce` / `JoltAngularForce` | Continuous push, scaled by dt internally. Re-applied every tick while present — use for held shoves |
| `JoltKinematicTarget { target_position, target_rotation }` | Per-tick destination for a kinematic body — elevator, moving platform, sliding door, patrol. Any entity with `JoltBody::kinematic`, not just characters: write where it should be this tick and the crate moves it there with `MoveKinematic`, deriving velocity from the delta so the platform shoves dynamics aside instead of teleporting through them. Not for teleports (`JoltTeleport` does that), not for dynamics (write velocity there). Set the fields each frame while it runs; remove to stop |

Velocity (`JoltLinearVelocity` / `JoltAngularVelocity`) is not in this table
on purpose: it lives on every non-static body already, and writes are
one-shot requests gated by change detection — see the next section.
Impulse ≠ one-frame force: forces scale with dt, impulses don't.

## Velocity: one component, read and write

- `JoltLinearVelocity` / `JoltAngularVelocity` — a single component per axis
  that is both the drive and the readout. Present on every non-static body
  from bake, zeroed. What you read is always the truth: what the sim says
  the body is doing right now.
- **Writing drives, once.** At the start of the next `JoltStep`, before the
  sim steps, the pre-step system pushes only `Changed` values to Jolt.
  Untouched bodies are never written, so sleep survives and natural motion
  stays natural. A blocked request is forgotten, not retried: write again (or
  hold with `set_if_neq`) to keep pushing.
- **The writeback doesn't re-drive.** After the step the sync writes the
  measured result back into the same component, bypassing change detection —
  so the update never looks like a new drive request. Game code watching
  `Changed<JoltLinearVelocity>` sees only real writes, never the crate's
  own readback.
- Drive with `set_if_neq` (or write only when the target changes). A plain
  write every frame re-drives every frame: the correct pattern for a motor,
  the wrong one for a nudge.
- Never drive from the readback into a fresh write: the stale value fights
  the sim (set 5 m/s, hit a wall, read 1, drive 1 forever).
- `body_snapshot(id)` — one-off position + linear + angular read for code
  that doesn't want the component at all.
- Linear and angular stay separate components so driving movement never
  wipes out spin (and vice versa).

## Sleep and observation

- `JoltSleeping` — sleep state the activation drain maintains. The sync skips
  sleeping bodies; debug greys them. Derefs to bool, so `if **sleeping`
  reads the state directly. Field writes, not marker add/remove — frequent
  flips never move the entity between archetypes. React with
  `Changed<JoltSleeping>` — no polling, no callbacks in the API.

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
- `JoltMotorDrive(target_velocity)` on an entity with `JointMotor` — per-tick
  motor speed targets, applied by `apply_jolt_motor_drives`. Ping-pong by
  rewriting the value.
- Vehicles (`JoltVehicleDrive`, tracked/wheeled), ragdolls (`JoltRagdoll`),
  soft bodies (`JoltSoftBodyConfig`, cloth/balloon/skinned), buoyancy
  (`JoltWater` resource + `JoltBuoyant` tag; flat surface, per-tick push).
- Sensors: `JoltSensor` flags the Jolt body so overlaps report without
  pushing. Add it before bake and it lands then; add it after and a retry
  marker (`PendingSensor`) applies it next tick, before the step, so the
  first overlap is never missed.
- Contacts: `JoltContactAdded` / `JoltContactRemoved` entity events after
  the step. Sleeping sensors still report — sleep skips integration, not
  detection.

## Queries (reads, not writes)

`cast_ray_all`, `collide_point_all`, `overlap_shape_all`, `cast_shape_all`
on `JoltPhysicsWorld`, plus `QueryProbe` shapes — all pure reads, safe from
any schedule. Soft-body vertex reads (`soft_vertices`, `soft_volume`) are
reads too. Everything that *moves* something goes through components,
triggers, or the body's own config — never ad-hoc world calls from game
systems (see the soft-body examples for the one place vertex code touches
the world, and keep it there).

## Collision layers and gravity

- `CollisionLayers`: declare teams up front, wire who-hits-who. Layer 0
  everywhere in examples; real games isolate world/character/sensor teams.
- Restitution combines by **max** (floor 0 + ball 0.9 bounces at 0.9);
  friction by **geometric mean** (either side 0 kills grip).
- World gravity flows one way: `JoltPlugin` builder → `JoltStartupGravity`
  resource → world at creation, then `JoltPhysicsWorld::set_gravity` for
  runtime changes. Per-body `with_gravity(factor)` scales it (0 ignores
  gravity, 1 normal, 2 double pull).

## Debug outlines (`JoltDebugPlugin`)

Green = awake, grey = sleeping. The color comes from the `JoltSleeping`
state the activation drain maintains (see "Reading bodies"), not from a
per-frame Jolt poll. Boxes/spheres draw as solids; capsules/cylinders/tapered
as line outlines; hulls as computed wireframe faces (brute-force triples,
debug counts only); meshes and heightfields as wireframes. Virtual characters
draw their capsules from their own state (they are not Jolt bodies and never
enter the body table); rigid characters are real bodies and draw like any
other.

## Tests and examples

- Tests run single-threaded (`--test-threads=1`): parallel `JoltWorld`s
  crash on Jolt's shared job plumbing. Each test drives time by hand — the
  `tick()` helper in `tests/` advances `Time<Fixed>` one timestep, then runs
  `FixedUpdate` followed by `JoltStep`. Running `FixedUpdate` alone is not
  enough: without the `JoltStep` run the sim never steps (delta stays 0).
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
