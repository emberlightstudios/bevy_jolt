//! Ragdolls: a chain of capsule bodies pinned by joints, posed by physics.
//!
//! A [`JoltRagdoll`] names the parts (capsule + offset from the spawn
//! origin) and how each links to its parent. The plugin bakes one dynamic
//! body per part plus one joint per link, then behaves like any other set
//! of bodies: [`sync_body_transforms`](crate::body_sync::sync_body_transforms)
//! moves the part entities, the joints hold them together. Knock it over
//! with an impulse; read poses from the part `Transform`s. Despawn the
//! ragdoll entity and every part + joint goes with it.
//!
//! Parent-child parts never collide (group filter), so stacked capsules in a
//! limb don't fight each other. Non-adjacent parts still collide: the body
//! folds on itself instead of passing through.

use bevy::prelude::*;

use crate::body_sync::{JoltBody, JoltBodyId, JoltShape};
use crate::joint_sync::{JointKind, JoltJoint};
use crate::physics_world::JointSpace;
use crate::plugin::JoltPhysicsWorld;

/// One ragdoll segment: a capsule body spawned at an offset from the
/// ragdoll origin. `parent_part` is the index into [`JoltRagdoll::parts`]
/// this hangs from (`None` = root, usually the hips).
#[derive(Clone, Debug)]
pub struct RagdollPart {
    pub capsule_half_height: f32,
    pub capsule_radius: f32,
    pub part_offset: Vec3,
    pub parent_part: Option<usize>,
    /// Swing limit of the link to the parent (radians, 0 = locked). Root
    /// ignores this.
    pub swing_half_cone_angle: f32,
}

/// Ragdoll spec: the part list plus shared physics tuning. Spawn with a
/// `Transform` (the origin the offsets hang from); the plugin expands it
/// into part bodies + joints as children.
#[derive(Component, Clone, Debug)]
pub struct JoltRagdoll {
    pub parts: Vec<RagdollPart>,
    pub object_layer: u16,
    pub density_kg_per_m3: f32,
    /// Linear damping bled from every part (Jolt default 0.05). Higher
    /// settles faster; the humanoid default stills joint crawl in seconds.
    pub linear_damping: f32,
    /// Angular damping bled from every part. Same scale as linear.
    pub angular_damping: f32,
}
impl JoltRagdoll {
    /// A small humanoid: hips root, torso, head, two arms (upper + lower),
    /// two legs (upper + lower). Offsets hang down -y from the spawn.
    pub fn humanoid(object_layer: u16) -> Self {
        let limb_cone = 0.6;
        Self {
            parts: vec![
                RagdollPart {
                    capsule_half_height: 0.15,
                    capsule_radius: 0.15,
                    part_offset: Vec3::new(0.0, 0.9, 0.0),
                    parent_part: None,
                    swing_half_cone_angle: 0.0,
                },
                RagdollPart {
                    capsule_half_height: 0.2,
                    capsule_radius: 0.14,
                    part_offset: Vec3::new(0.0, 1.3, 0.0),
                    parent_part: Some(0),
                    swing_half_cone_angle: 0.5,
                },
                RagdollPart {
                    capsule_half_height: 0.1,
                    capsule_radius: 0.12,
                    part_offset: Vec3::new(0.0, 1.7, 0.0),
                    parent_part: Some(1),
                    swing_half_cone_angle: 0.5,
                },
                RagdollPart {
                    capsule_half_height: 0.12,
                    capsule_radius: 0.06,
                    part_offset: Vec3::new(-0.25, 1.35, 0.0),
                    parent_part: Some(1),
                    swing_half_cone_angle: limb_cone,
                },
                RagdollPart {
                    capsule_half_height: 0.12,
                    capsule_radius: 0.05,
                    part_offset: Vec3::new(-0.25, 1.05, 0.0),
                    parent_part: Some(3),
                    swing_half_cone_angle: limb_cone,
                },
                RagdollPart {
                    capsule_half_height: 0.12,
                    capsule_radius: 0.06,
                    part_offset: Vec3::new(0.25, 1.35, 0.0),
                    parent_part: Some(1),
                    swing_half_cone_angle: limb_cone,
                },
                RagdollPart {
                    capsule_half_height: 0.12,
                    capsule_radius: 0.05,
                    part_offset: Vec3::new(0.25, 1.05, 0.0),
                    parent_part: Some(5),
                    swing_half_cone_angle: limb_cone,
                },
                RagdollPart {
                    capsule_half_height: 0.2,
                    capsule_radius: 0.08,
                    part_offset: Vec3::new(-0.12, 0.5, 0.0),
                    parent_part: Some(0),
                    swing_half_cone_angle: limb_cone,
                },
                RagdollPart {
                    capsule_half_height: 0.2,
                    capsule_radius: 0.07,
                    part_offset: Vec3::new(-0.12, 0.1, 0.0),
                    parent_part: Some(7),
                    swing_half_cone_angle: limb_cone,
                },
                RagdollPart {
                    capsule_half_height: 0.2,
                    capsule_radius: 0.08,
                    part_offset: Vec3::new(0.12, 0.5, 0.0),
                    parent_part: Some(0),
                    swing_half_cone_angle: limb_cone,
                },
                RagdollPart {
                    capsule_half_height: 0.2,
                    capsule_radius: 0.07,
                    part_offset: Vec3::new(0.12, 0.1, 0.0),
                    parent_part: Some(9),
                    swing_half_cone_angle: limb_cone,
                },
            ],
            object_layer,
            density_kg_per_m3: 1000.0,
            linear_damping: 3.0,
            angular_damping: 3.0,
        }
    }
}

/// Part bodies of one baked ragdoll, in spec order. Read poses from these.
#[derive(Component, Clone, Debug)]
pub struct JoltRagdollParts {
    pub part_entities: Vec<Entity>,
}

/// Expands [`JoltRagdoll`] into part bodies on add. Links (joints +
/// no-collide pairs) go through [`bake_ragdoll_links`], which polls until
/// every part owns a [`JoltBodyId`]: entity commands buffer, so ids don't
/// exist inside this observer. Parts spawn as children (despawn cascades).
pub fn bake_jolt_ragdoll(
    trigger: On<Add, JoltRagdoll>,
    ragdoll_query: Query<(&JoltRagdoll, &Transform)>,
    mut commands: Commands,
) {
    let ragdoll_entity = trigger.event().entity;
    let Ok((ragdoll, ragdoll_pose)) = ragdoll_query.get(ragdoll_entity) else {
        panic!("JoltRagdoll gone on {ragdoll_entity:?} before bake ran");
    };
    assert!(
        !ragdoll.parts.is_empty(),
        "ragdoll needs at least one part"
    );
    for (part_index, ragdoll_part) in ragdoll.parts.iter().enumerate() {
        if let Some(parent_index) = ragdoll_part.parent_part {
            assert!(
                parent_index < part_index,
                "ragdoll part {part_index} names parent {parent_index}: parents must come first"
            );
        }
    }
    let ragdoll_origin = ragdoll_pose.translation;
    let mut part_entities = Vec::with_capacity(ragdoll.parts.len());
    for ragdoll_part in &ragdoll.parts {
        let part_entity = commands
            .spawn((
                Transform::from_translation(ragdoll_origin + ragdoll_part.part_offset),
                JoltBody::dynamic(ragdoll.object_layer),
                crate::body_forces::JoltDensity::new(ragdoll.density_kg_per_m3),
                JoltShape::capsule(
                    ragdoll_part.capsule_half_height,
                    ragdoll_part.capsule_radius,
                ),
                crate::body_forces::JoltDamping::new(
                    ragdoll.linear_damping,
                    ragdoll.angular_damping,
                ),
                ChildOf(ragdoll_entity),
            ))
            .id();
        part_entities.push(part_entity);
    }
    // Links wait for `bake_ragdoll_links`: commands buffer, so part bodies
    // own no JoltBodyId until their own bake observers flush.
    commands.entity(ragdoll_entity).insert(JoltRagdollParts {
        part_entities: part_entities.clone(),
    });
    commands.entity(ragdoll_entity).insert(RagdollLinksPending {
        part_entities,
        part_offsets: ragdoll.parts.iter().map(|part| part.part_offset).collect(),
        parent_indices: ragdoll.parts.iter().map(|part| part.parent_part).collect(),
        swing_angles: ragdoll
            .parts
            .iter()
            .map(|part| part.swing_half_cone_angle)
            .collect(),
    });
}

/// Unbaked link data: everything [`bake_ragdoll_links`] needs once part
/// bodies own ids. Removed after the links file.
#[derive(Component, Clone, Debug)]
pub struct RagdollLinksPending {
    part_entities: Vec<Entity>,
    part_offsets: Vec<Vec3>,
    parent_indices: Vec<Option<usize>>,
    swing_angles: Vec<f32>,
}

/// Files joints + no-collide pairs once every part owns a [`JoltBodyId`].
/// Runs before the joint creator so links bake the same tick bodies land.
pub fn bake_ragdoll_links(
    mut pending_query: Query<(Entity, &RagdollLinksPending, &Transform)>,
    body_ids: Query<&JoltBodyId>,
    mut commands: Commands,
    mut physics_world: ResMut<JoltPhysicsWorld>,
) {
    for (ragdoll_entity, pending_links, ragdoll_pose) in &mut pending_query {
        if pending_links
            .part_entities
            .iter()
            .any(|part_entity| body_ids.get(*part_entity).is_err())
        {
            continue;
        }
        let ragdoll_origin = ragdoll_pose.translation;
        for (part_index, part_entity) in pending_links.part_entities.iter().enumerate() {
            let Some(parent_index) = pending_links.parent_indices[part_index] else {
                continue;
            };
            let parent_id = body_ids
                .get(pending_links.part_entities[parent_index])
                .expect("ragdoll parent baked above")
                .body_id_raw;
            let part_id = body_ids
                .get(*part_entity)
                .expect("ragdoll part baked above")
                .body_id_raw;
            // Parent-child capsules share a group pair so stacked limbs never
            // collide; non-adjacent parts still collide and fold.
            physics_world.set_bodies_no_collide(parent_id, part_id);
            let joint_position = ragdoll_origin
                + (pending_links.part_offsets[parent_index]
                    + pending_links.part_offsets[part_index])
                    * 0.5;
            commands.spawn(JoltJoint {
                body_a: pending_links.part_entities[parent_index],
                body_b: *part_entity,
                joint_space: JointSpace::World,
                kind: JointKind::Cone {
                    constraint_point: joint_position,
                    twist_axis1: Dir3::Y,
                    twist_axis2: Dir3::Y,
                    half_cone_angle: pending_links.swing_angles[part_index],
                },
            });
        }
        commands.entity(ragdoll_entity).remove::<RagdollLinksPending>();
    }
}
