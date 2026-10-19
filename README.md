# bevy_jolt

Safe Bevy integration for Jolt Physics. Raw FFI lives in `jolt_sys`; this
crate owns world lifetime, the safe body API, and the Bevy schedule wiring.

## Schedules: where your game code goes

Put game logic in `FixedPreUpdate`, `FixedUpdate`, and/or `FixedPostUpdate`.
The crate guarantees this ordering every tick:

1. Your `FixedUpdate` systems are guaranteed to finish before the simulation steps. Nothing physics-side runs
   yet.
2. `JoltStep` runs next: change-detection pushes first (motion, sleep, driven
   velocities), then the sim steps, then all readbacks (poses, measured
   velocities, contacts, sleep states).
3. Your `FixedPostUpdate` systems run last. The step and every readback are already
   done, so what you read is this tick's result.

So: write drives in `FixedUpdate`, read results in `FixedPostUpdate` (or
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
  collision layer only. Tuning rides sibling components, each read at bake
  and pushed live on `Changed` writes: `JoltDensity::new(10.0)`,
  `JoltGravity::new(0.0)` (0 floats, 2 double-pulls), `JoltFriction::new(0.0)`
  (0 ice – 1+ rubber), `JoltRestitution::new(0.9)` (0 dead – 1 superball),
  `JoltCcd::enabled()` (fast bullets/swords), `JoltDamping::new(0.1, 0.1)`.
- Bake inserts `JoltBodyId` (read it, never write it) plus
  `PreviousBodyTransform` (render interpolation state).
- Post-step, the sync copies pose into `Transform` and writes measured
  velocities (below). Sleeping bodies are skipped — frozen means frozen.

Despawn the entity and the Jolt body is destroyed — and so is removing the
component that created it. Every Jolt object (body, character, joint, soft body,
vehicle, ragdoll) is freed when its spec component leaves the entity, not only
when the entity despawns: cleanup hangs off component *removal*. Without that,
`remove::<JoltBody>()` on a live entity left a body in the world with nothing
referring to it, and a re-add baked a second one — an insert fires no removal,
so the id was overwritten and the first body leaked. Joints on a destroyed body
cascade first (constraint removals are synchronous, the solver never sees a dead
endpoint).
An id that names no live body — stale, or a double destroy — is refused with a
panic rather than handed to Jolt, which would free whatever body now owns that
slot.

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
  in the solver: pushes and gets pushed. `capsule_center_height` lifts the
  capsule so its bottom cap rests on the entity origin (feet at y = 0): the
  Jolt `Character` centers its capsule on its position, character roots hold
  meshes and skeletons at the origin. Edit the capsule size in place and the
  stance swap re-anchors the feet — no sinking, no floating.

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
  soft bodies (`JoltSoftBodyConfig`, cloth/balloon/skinned; their shared
  settings are freed with the body), buoyancy (`JoltWater` resource + `JoltBuoyant` tag; flat surface, per-tick push).
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
  runtime changes. Per-body `JoltGravity::new(factor)` scales it (0 ignores
  gravity, 1 normal, 2 double pull).

## Time scale (slow motion)

`JoltPhysicsWorld::set_time_scale` scales how much sim time each fixed tick
advances (`1` = real time, `0.15` = slow motion, `0` = paused). The tick
itself stays fixed: every tick still steps once, just with a smaller delta
(`tick × scale`). See the `bodies_slow_mo` example.

Limits worth knowing:

- **Determinism:** a scaled run is only deterministic for a fixed scale
  history. Same inputs at 1x and 0.15x take different solver paths (smaller
  steps integrate slightly differently), so replays must record the scale
  alongside inputs — replaying inputs alone at a different scale diverges.
- **Stability at extremes:** very small scales take very small steps, which
  is stable but wastes solver work per unit of visible motion; very large
  scales (2x+) take oversized steps that can tunnel fast bodies — prefer
  `JoltCcd` on those, or cap the scale.
- **Driven things stay consistent:** kinematic targets, buoyancy, and
  character movers all consume the same scaled delta, so nothing drifts out
  of sync with the world step.


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

## Body-id → entity lookup

**Fixed 2026-10-18.** Resolving a Jolt body id to a Bevy `Entity` used to be a
linear scan of `Query<(Entity, &JoltBodyId)>` repeated once per event, in four
places:

- `src/body_forces.rs` — `sync_sleep_states`, up to `2 * MAX_ACTIVATION_EVENTS`
  (64) lookups per tick.
- `src/contact_events.rs` — `drain_contact_events`, **two** ids per pair over
  up to `2 * MAX_CONTACT_EVENTS` (128), so as many as 256 full scans every tick.
- `src/debug_draw.rs` — twice: once per body in the shape table, and again per
  vehicle's wheels. That one was O(bodies²) in a system running every frame.

`body_sync::body_entity_index` now builds the map once per system invocation
and every lookup is O(1). It is deliberately *not* cached on `JoltWorld`: bodies
are created in four places (body bake, ragdoll parts, vehicle chassis, soft
body) and `remove_jolt_ragdoll` leaves part entities holding ids that are
already dead, so a persistent index would need lifecycle rules more subtle than
the lookup it replaces. A per-invocation map costs one pass and cannot go stale.

If you add a fifth place that needs this lookup, use the helper rather than
re-scanning. Regression coverage: `tests/body_event_index.rs`.

Joint teardown uses the same discipline in the other direction: `JointBodyIndex`
maps a body to the joints linking it, built where the constraint is created, so
despawning a body unlinks its joints without walking every joint in the world.

## What this crate is not

A game framework. No player controller, no input, no health, no inventory.
Game-specific logic lives in game crates (or examples, when demo-specific).
Keep it unopinionated: if different games would implement it differently, it
doesn't belong here.
