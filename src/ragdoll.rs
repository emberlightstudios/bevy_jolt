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

/// Joint between a part and its parent, about `anchor` (world space),
/// measured from the seated spawn pose (identical frames on both sides
/// read zero).
#[derive(Clone, Copy, Debug)]
pub enum RagdollJoint {
 /// Rotation about `hinge_axis` within [`min`, `max`] (elbows, knees).
 Hinge {
 anchor: Vec3,
 hinge_axis: Dir3,
 normal_axis: Dir3,
 min: f32,
 max: f32,
 },
 /// Cone swing about `twist_axis` plus bounded twist (everything else).
 SwingTwist {
 anchor: Vec3,
 twist_axis: Dir3,
 plane_axis: Dir3,
 normal_half_cone: f32,
 plane_half_cone: f32,
 twist_min: f32,
 twist_max: f32,
 },
}
/// Ragdoll spec: the part list plus shared physics tuning. Spawn with a
/// `JoltRagdoll`; the plugin expands it into part bodies + joints.
#[derive(Component, Clone, Debug)]
pub struct JoltRagdoll {
 pub parts: Vec<RagdollPart>,
 pub object_layer: u16,
 pub density_kg_per_m3: f32,
 /// Motion every body bakes as. Kinematic = follow bones (hitboxes);
 /// dynamic = simulate. Flips later cost nothing.
 pub motion: JoltMotion,
}

impl JoltRagdoll {
    /// A small humanoid: hips root, torso, head, two arms (upper + lower),
    /// two legs (upper + lower). Offsets hang down -y from the spawn.
 pub fn humanoid(object_layer: u16) -> Self {
 // Offsets first: anchors are midpoints between parent/child positions.
 let offsets = [
 Vec3::new(0.0, 0.9, 0.0),
 Vec3::new(0.0, 1.3, 0.0),
 Vec3::new(0.0, 1.7, 0.0),
 Vec3::new(-0.25, 1.35, 0.0),
 Vec3::new(-0.25, 1.05, 0.0),
 Vec3::new(0.25, 1.35, 0.0),
 Vec3::new(0.25, 1.05, 0.0),
 Vec3::new(-0.12, 0.5, 0.0),
 Vec3::new(-0.12, 0.1, 0.0),
 Vec3::new(0.12, 0.5, 0.0),
 Vec3::new(0.12, 0.1, 0.0),
 ];
 let parents = [
 None,
 Some(0),
 Some(1),
 Some(1),
 Some(3),
 Some(1),
 Some(5),
 Some(0),
 Some(7),
 Some(0),
 Some(9),
 ];
 let anchor_for = |part: usize| match parents[part] {
 Some(parent) => (offsets[parent] + offsets[part]) * 0.5,
 None => offsets[part],
 };
 let swing = |part: usize, cone: f32| RagdollJoint::SwingTwist {
 anchor: anchor_for(part),
 twist_axis: Dir3::Y,
 plane_axis: Dir3::X,
 normal_half_cone: cone,
 plane_half_cone: cone,
 twist_min: -cone,
 twist_max: cone,
 };
 let capsule_part = |part: usize, half_height: f32, radius: f32, cone: f32| RagdollPart {
 shape: RagdollShape::Capsule {
 cylinder_half_height: half_height,
 radius,
 },
 part_position: offsets[part],
 part_rotation: Quat::IDENTITY,
 parent_part: parents[part],
 joint: swing(part, cone),
 };
 let limb_cone = 0.6;
 Self {
 parts: vec![
 capsule_part(0, 0.15, 0.15, 0.0),
 capsule_part(1, 0.2, 0.14, 0.5),
 capsule_part(2, 0.1, 0.12, 0.5),
 capsule_part(3, 0.12, 0.06, limb_cone),
 capsule_part(4, 0.12, 0.05, limb_cone),
 capsule_part(5, 0.12, 0.06, limb_cone),
 capsule_part(6, 0.12, 0.05, limb_cone),
 capsule_part(7, 0.2, 0.08, limb_cone),
 capsule_part(8, 0.2, 0.07, limb_cone),
 capsule_part(9, 0.2, 0.08, limb_cone),
 capsule_part(10, 0.2, 0.07, limb_cone),
 ],
 object_layer,
 density_kg_per_m3: 1000.0,
 motion: JoltMotion::Dynamic,
 }
}
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

/// Live ragdoll handle: owns the Jolt bodies + constraints. Removing this
/// tears the whole ragdoll out of the system.
#[derive(Component, Clone, Copy, Debug)]
pub struct JoltRagdollHandle {
 pub(crate) ragdoll_raw: *mut jolt_sys::BJoltRagdoll,
}

impl JoltRagdollHandle {
 /// The opaque Jolt handle, for world calls (`ragdoll_set_motion`, ...).
 pub fn raw(self) -> *mut jolt_sys::BJoltRagdoll {
 self.ragdoll_raw
 }
}
// The handle is an opaque Jolt pointer, only touched from the physics thread.
unsafe impl Send for JoltRagdollHandle {}
unsafe impl Sync for JoltRagdollHandle {}

/// Bakes a [`JoltRagdoll`] in one shot: settings builder → stabilize →
///
/// create → part entities. Runs on add; panics on invalid specs or failed
/// creates because a half-built ragdoll is never useful.
pub fn bake_jolt_ragdoll(
    trigger: On<Add, JoltRagdoll>,
    ragdoll_query: Query<&JoltRagdoll>,
    mut commands: Commands,
    mut physics_world: ResMut<JoltPhysicsWorld>,
) {
    let ragdoll_entity = trigger.event().entity;
    let Ok(ragdoll) = ragdoll_query.get(ragdoll_entity) else {
        panic!("JoltRagdoll gone on {ragdoll_entity:?} before bake ran");
    };
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
 if ragdoll_part.parent_part.is_some() {
 let joint_ok = match ragdoll_part.joint {
 RagdollJoint::Hinge {
 anchor,
 hinge_axis,
 normal_axis,
 min,
 max,
 } => world.ragdoll_build_set_hinge(build, part_index, anchor, hinge_axis, normal_axis, min, max),
 RagdollJoint::SwingTwist {
 anchor,
 twist_axis,
 plane_axis,
 normal_half_cone,
 plane_half_cone,
 twist_min,
 twist_max,
 } => world.ragdoll_build_set_swing_twist(
 build, part_index, anchor, twist_axis, plane_axis, normal_half_cone, plane_half_cone,
 twist_min, twist_max,
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
    let handle = world.ragdoll_create(build);
    world.ragdoll_build_destroy(build);
    let Some(handle) = handle else {
        panic!("ragdoll create failed on {ragdoll_entity:?}");
    };
    let body_count = world.ragdoll_body_count(handle) as usize;
    assert_eq!(
        body_count,
        ragdoll.parts.len(),
        "ragdoll came back with {body_count} bodies for {} parts",
        ragdoll.parts.len()
    );
    let mut body_ids = vec![0u32; body_count];
    let written = world.ragdoll_body_ids(handle, &mut body_ids);
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
 RagdollShape::Box { half_extents } => crate::physics_world::PhysicsShape::Box { half_extents },
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
 JoltRagdollHandle {
 ragdoll_raw: handle,
 },
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
 physics_world.ragdoll_destroy(handle.ragdoll_raw);
}
