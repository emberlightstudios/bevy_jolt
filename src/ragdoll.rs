//! Ragdolls through Jolt's `RagdollSettings`: a part list baked in one shot
//! with mass stabilization, constraint priorities, and parent-child
//! no-collide handled inside Jolt.
//!
//! A [`JoltRagdoll`] names the parts (shape + pose + how each links to its
//! parent). The plugin builds the settings, stabilizes, creates bodies +
//! constraints in one call, then spawns one part entity per body (pose sync
//! moves them like any other body). Despawn the ragdoll entity and the whole
//! ragdoll — bodies and constraints — goes with it.

use bevy::prelude::*;

use crate::body_sync::{JoltBodyId, JoltMotion, PreviousBodyTransform};
use crate::plugin::JoltPhysicsWorld;

/// One ragdoll segment: shape + spawn pose + link to the parent.
/// `parent_part` is the index into [`JoltRagdoll::parts`] this hangs from
/// (`None` = root). Parents must come first (index < child).
#[derive(Clone, Debug)]
pub struct RagdollPart {
    pub shape: RagdollShape,
    /// Body origin in world space at spawn.
    pub part_position: Vec3,
    pub part_rotation: Quat,
    pub parent_part: Option<usize>,
    /// Joint to the parent. Root ignores this.
    pub joint: RagdollJoint,
}

/// Shape of one ragdoll part. Same semantics as `JoltShape` for these three:
///
/// - capsule: cylinder half height (without caps) + radius.
/// - box: half extents.
/// - sphere: radius.
#[derive(Clone, Copy, Debug)]
pub enum RagdollShape {
    Capsule {
        cylinder_half_height: f32,
        radius: f32,
    },
    Box {
        half_extents: Vec3,
    },
    Sphere {
        radius: f32,
    },
}

/// Joint between a part and its parent, about `anchor` (world space).
/// Frames derive at bake from the seated part rotations (per side, like
/// avian's `local_basis2`), so the seated pose reads zero. Hinges flex
/// about body-local Z (normal Y); swing-twist runs twist along body-local
/// Y (plane X). `SwingTwistFramed` uses explicit per-side twist/plane axes
/// (each side's own seated rotation carries them to world) for joints where
/// neither body's long axis is the anatomical axis — e.g. the ankle, whose
/// cone must open down the foot (ankle → toe), not along the shin or the
/// foot box's measured Y.
#[derive(Clone, Copy, Debug)]
pub enum RagdollJoint {
    /// Rotation about body-local Z within [`min`, `max`] (elbows, knees).
    Hinge { anchor: Vec3, min: f32, max: f32 },
    /// Cone swing about body-local Y plus bounded twist (everything else).
    SwingTwist {
        anchor: Vec3,
        normal_half_cone: f32,
        plane_half_cone: f32,
        twist_min: f32,
        twist_max: f32,
    },
    /// Cone swing about an explicit twist axis (in each side's seated local
    /// frame) plus bounded twist. Same semantics as `SwingTwist`, but the
    /// cone opens along `twist_axis` instead of body-local Y.
    SwingTwistFramed {
        anchor: Vec3,
        /// Twist axis in the parent's seated local frame.
        parent_twist: Vec3,
        /// Plane axis in the parent's seated local frame.
        parent_plane: Vec3,
        /// Twist axis in the child's seated local frame.
        child_twist: Vec3,
        /// Plane axis in the child's seated local frame.
        child_plane: Vec3,
        normal_half_cone: f32,
        plane_half_cone: f32,
        twist_min: f32,
        twist_max: f32,
    },
}

/// Which motor axis to run on a ragdoll joint. Hinges take `Hinge`;
/// swing-twist joints take `Twist` or `Swing`.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum RagdollDriveAxis {
    /// Hinge flex about body-local Z (elbows, knees).
    Hinge,
    /// Swing-twist spin about body-local Y.
    Twist,
    /// Swing-twist sweep about body-local X.
    Swing,
}

/// Ragdoll spec: the part list plus shared physics tuning. Spawn with a
#[derive(Component, Clone, Debug)]
pub struct JoltRagdoll {
    pub parts: Vec<RagdollPart>,
    pub object_layer: u16,
    pub density_kg_per_m3: f32,
    /// Motion every body bakes as. Kinematic = follow bones (hitboxes);
    /// dynamic = simulate. Flips later cost nothing.
    pub motion: JoltMotion,
}

/// Part bodies of one baked ragdoll, in spec order. Read poses from these.
#[derive(Component, Clone, Debug)]
pub struct JoltRagdollParts {
    pub part_entities: Vec<Entity>,
    body_ids: Vec<u32>,
}

/// Marker on ragdoll part entities: their `JoltBodyId` is owned by the
/// ragdoll teardown, so `despawn_jolt_body` must skip them (double destroy
/// crashes). The whole ragdoll — bodies and constraints — dies with the
/// `JoltRagdollHandle`.
#[derive(Component, Clone, Copy, Debug)]
pub struct RagdollPartBody;

/// Live ragdoll handle: 1-based id into the world's ragdoll registry. Plain
/// `u32` — no pointers, no unsafe impls, safe to copy. Removing this
/// tears the whole ragdoll (bodies + constraints) out of the system.
#[derive(Component, Clone, Copy, Debug)]
pub struct JoltRagdollHandle {
    pub(crate) ragdoll_id: u32,
}

impl JoltRagdollHandle {
    /// The registry id, for world calls (`ragdoll_set_motion`, ...).
    pub fn id(self) -> u32 {
        self.ragdoll_id
    }
}

/// Bakes a [`JoltRagdoll`] in one shot: settings builder → stabilize →
///
/// create → part entities. Runs on add; panics on invalid specs or failed
/// creates because a half-built ragdoll is never useful.
pub fn bake_jolt_ragdoll(
    trigger: On<Add, JoltRagdoll>,
    ragdoll_query: Query<(&JoltRagdoll, Option<&Transform>)>,
    mut commands: Commands,
    mut physics_world: ResMut<JoltPhysicsWorld>,
) {
    let ragdoll_entity = trigger.event().entity;
    let Ok((ragdoll, ragdoll_transform)) = ragdoll_query.get(ragdoll_entity) else {
        panic!("JoltRagdoll gone on {ragdoll_entity:?} before bake ran");
    };
    if ragdoll_transform.is_none() {
        // Part entities parent under the spec entity, so the spec needs a
        // Transform: without it every part warns (Bevy B0004).
        commands.entity(ragdoll_entity).insert(Transform::IDENTITY);
    }
    assert!(!ragdoll.parts.is_empty(), "ragdoll needs at least one part");
    for (part_index, ragdoll_part) in ragdoll.parts.iter().enumerate() {
        if let Some(parent_index) = ragdoll_part.parent_part {
            assert!(
                parent_index < part_index,
                "ragdoll part {part_index} names parent {parent_index}: parents must come first"
            );
        }
    }
    let world = &mut **physics_world;
    let build = world.ragdoll_build_create();
    // Seated rotations per part (parents-first, so the parent's frame is
    // always known): joint frames derive per side from these.
    let mut seated_rotations = Vec::with_capacity(ragdoll.parts.len());
    for ragdoll_part in &ragdoll.parts {
        let (shape_kind, dim_x, dim_y, dim_z) = match ragdoll_part.shape {
            RagdollShape::Capsule {
                cylinder_half_height,
                radius,
            } => (0, cylinder_half_height, radius, 0.0),
            RagdollShape::Box { half_extents } => {
                (1, half_extents.x, half_extents.y, half_extents.z)
            }
            RagdollShape::Sphere { radius } => (2, radius, 0.0, 0.0),
        };
        let part_index = world.ragdoll_build_add_part(
            build,
            ragdoll_part.parent_part.map_or(-1, |parent| parent as i32),
            shape_kind,
            dim_x,
            dim_y,
            dim_z,
            ragdoll_part.part_position,
            ragdoll_part.part_rotation,
            ragdoll.object_layer,
            ragdoll.density_kg_per_m3,
            ragdoll.motion,
        );
        assert!(part_index >= 0, "ragdoll part shape invalid");
        seated_rotations.push(ragdoll_part.part_rotation);
        if let Some(parent_index) = ragdoll_part.parent_part {
            // Per-side frames: local axes through each body's own seated
            // rotation, so the seated pose reads zero (avian local_basis2).
            let parent_rotation = seated_rotations[parent_index];
            let child_rotation = ragdoll_part.part_rotation;
            let axis = |rotation: Quat, local: Vec3| rotation * local;
            let joint_ok = match ragdoll_part.joint {
                RagdollJoint::Hinge { anchor, min, max } => world.ragdoll_build_set_hinge(
                    build,
                    part_index,
                    anchor,
                    axis(parent_rotation, Vec3::Z),
                    axis(parent_rotation, Vec3::Y),
                    axis(child_rotation, Vec3::Z),
                    axis(child_rotation, Vec3::Y),
                    min,
                    max,
                ),
                RagdollJoint::SwingTwist {
                    anchor,
                    normal_half_cone,
                    plane_half_cone,
                    twist_min,
                    twist_max,
                } => world.ragdoll_build_set_swing_twist(
                    build,
                    part_index,
                    anchor,
                    axis(parent_rotation, Vec3::Y),
                    axis(parent_rotation, Vec3::X),
                    axis(child_rotation, Vec3::Y),
                    axis(child_rotation, Vec3::X),
                    normal_half_cone,
                    plane_half_cone,
                    twist_min,
                    twist_max,
                ),
                RagdollJoint::SwingTwistFramed {
                    anchor,
                    parent_twist,
                    parent_plane,
                    child_twist,
                    child_plane,
                    normal_half_cone,
                    plane_half_cone,
                    twist_min,
                    twist_max,
                } => world.ragdoll_build_set_swing_twist(
                    build,
                    part_index,
                    anchor,
                    axis(parent_rotation, parent_twist),
                    axis(parent_rotation, parent_plane),
                    axis(child_rotation, child_twist),
                    axis(child_rotation, child_plane),
                    normal_half_cone,
                    plane_half_cone,
                    twist_min,
                    twist_max,
                ),
            };
            assert!(joint_ok, "ragdoll joint failed on part {part_index}");
        }
    }
    assert!(
        world.ragdoll_build_stabilize(build),
        "ragdoll stabilization failed"
    );
    world.ragdoll_build_finalize(build);
    let ragdoll_id = world.ragdoll_create(build);
    world.ragdoll_build_destroy(build);
    assert_ne!(ragdoll_id, 0, "ragdoll create failed on {ragdoll_entity:?}");
    let body_count = world.ragdoll_body_count(ragdoll_id) as usize;
    assert_eq!(
        body_count,
        ragdoll.parts.len(),
        "ragdoll came back with {body_count} bodies for {} parts",
        ragdoll.parts.len()
    );
    let mut body_ids = vec![0u32; body_count];
    let written = world.ragdoll_body_ids(ragdoll_id, &mut body_ids);
    assert_eq!(written as usize, body_count, "ragdoll body ids short");
    let mut part_entities = Vec::with_capacity(body_count);
    for (part_index, body_id_raw) in body_ids.iter().enumerate() {
        let part_pose = Transform {
            translation: ragdoll.parts[part_index].part_position,
            rotation: ragdoll.parts[part_index].part_rotation,
            ..default()
        };
        // File the outline for the debug visualizer: ragdoll bodies never pass
        // through the per-body bake that files them otherwise.
        let outline = match ragdoll.parts[part_index].shape {
            RagdollShape::Capsule {
                cylinder_half_height,
                radius,
            } => crate::physics_world::PhysicsShape::Capsule {
                capsule_half_height: cylinder_half_height,
                capsule_radius: radius,
            },
            RagdollShape::Box { half_extents } => {
                crate::physics_world::PhysicsShape::Box { half_extents }
            }
            RagdollShape::Sphere { radius } => crate::physics_world::PhysicsShape::Sphere {
                sphere_radius: radius,
            },
        };
        world.file_shape(*body_id_raw, std::sync::Arc::new(outline));
        let part_entity = commands
            .spawn((
                part_pose,
                JoltBodyId {
                    body_id_raw: *body_id_raw,
                },
                RagdollPartBody,
                crate::body_forces::JoltSleeping { sleeping: false },
                PreviousBodyTransform {
                    previous_position: part_pose.translation,
                    previous_rotation: part_pose.rotation,
                },
                ChildOf(ragdoll_entity),
            ))
            .id();
        part_entities.push(part_entity);
    }
    commands.entity(ragdoll_entity).insert((
        JoltRagdollParts {
            part_entities,
            body_ids,
        },
        JoltRagdollHandle { ragdoll_id },
    ));
}
/// Tears the whole ragdoll (bodies + constraints) out of Jolt when the
/// handle leaves. Part entities keep their `JoltBodyId`s but the ids are
/// dead — the ragdoll entity's despawn takes the parts with it.
pub fn despawn_jolt_ragdoll(
    trigger: On<Remove, JoltRagdollHandle>,
    handle_query: Query<(&JoltRagdollHandle, &JoltRagdollParts)>,
    mut physics_world: ResMut<JoltPhysicsWorld>,
) {
    let trigger_entity = trigger.event().entity;
    let Ok((handle, baked)) = handle_query.get(trigger_entity) else {
        panic!("JoltRagdollHandle gone on {trigger_entity:?} before despawn ran");
    };
    for body_id_raw in &baked.body_ids {
        physics_world.unfile_shape(*body_id_raw);
    }
    physics_world.ragdoll_destroy(handle.ragdoll_id);
}
