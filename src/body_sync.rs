//! Entity ↔ physics body sync: spawn, transform copy, despawn.
//!
//! Game code spawns an entity with [`JoltBody`] (motion + layer) plus
//! [`JoltShape`] (geometry) plus `Transform` (spawn pose). The
//! [`JoltPlugin`](crate::JoltPlugin) creates the Jolt body when the
//! component is added, copies position + rotation into `Transform` after each
//! physics tick, and destroys the Jolt body when the entity leaves.

use bevy::prelude::*;

use crate::physics_world::PhysicsShape;
use crate::plugin::JoltPhysicsWorld;

/// Rigid-body descriptor: motion type + collision layer + density. Add
/// alongside a [`JoltShape`] and a `Transform`; the plugin creates the Jolt
/// body and keeps them in sync. Mirrors Jolt's `Body`: motion, layer, and
/// density live on the body, geometry lives on the (shareable) shape.
/// Density is kg/m³ (water ≈ 1000); mass = density × shape volume, baked
/// at creation.
#[derive(Component, Clone, Copy, Debug)]
pub struct JoltBody {
    pub motion: JoltMotion,
    pub object_layer: u16,
    pub density_kg_per_m3: f32,
}

/// Jolt motion type: static never moves, kinematic moves by velocity and
/// pushes dynamics without responding to forces, dynamic fully simulates.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum JoltMotion {
    Static,
    Kinematic,
    Dynamic,
}

impl JoltBody {
    /// Default density: water-like, matches Jolt's own default.
    pub const DEFAULT_DENSITY: f32 = 1000.0;

    pub fn dynamic(object_layer: u16) -> Self {
        Self {
            motion: JoltMotion::Dynamic,
            object_layer,
            density_kg_per_m3: Self::DEFAULT_DENSITY,
        }
    }

    pub fn fixed(object_layer: u16) -> Self {
        Self {
            motion: JoltMotion::Static,
            object_layer,
            density_kg_per_m3: Self::DEFAULT_DENSITY,
        }
    }

    pub fn kinematic(object_layer: u16) -> Self {
        Self {
            motion: JoltMotion::Kinematic,
            object_layer,
            density_kg_per_m3: Self::DEFAULT_DENSITY,
        }
    }

    /// Override density in kg/m³. Must be positive and finite.
    pub fn with_density(mut self, density_kg_per_m3: f32) -> Self {
        assert!(
            density_kg_per_m3.is_finite() && density_kg_per_m3 > 0.0,
            "density must be positive, got {}",
            density_kg_per_m3
        );
        self.density_kg_per_m3 = density_kg_per_m3;
        self
    }
}

/// Pure collision geometry for one physics body. Add alongside [`JoltBody`].
/// Mirrors Jolt's `Shape`: immutable geometry with no motion of its own.
#[derive(Component, Clone, Copy, Debug)]
pub struct JoltShape(pub PhysicsShape);

impl JoltShape {
    pub fn box_shape(half_extents: Vec3) -> Self {
        Self(PhysicsShape::Box { half_extents })
    }

    pub fn sphere(sphere_radius: f32) -> Self {
        Self(PhysicsShape::Sphere { sphere_radius })
    }

    pub fn capsule(capsule_half_height: f32, capsule_radius: f32) -> Self {
        Self(PhysicsShape::Capsule {
            capsule_half_height,
            capsule_radius,
        })
    }

    pub fn cylinder(cylinder_half_height: f32, cylinder_radius: f32) -> Self {
        Self(PhysicsShape::Cylinder {
            cylinder_half_height,
            cylinder_radius,
        })
    }

    pub fn tapered_cylinder(
        tapered_half_height: f32,
        top_radius: f32,
        bottom_radius: f32,
    ) -> Self {
        Self(PhysicsShape::TaperedCylinder {
            tapered_half_height,
            top_radius,
            bottom_radius,
        })
    }

    pub fn tapered_capsule(
        tapered_half_height: f32,
        top_radius: f32,
        bottom_radius: f32,
    ) -> Self {
        Self(PhysicsShape::TaperedCapsule {
            tapered_half_height,
            top_radius,
            bottom_radius,
        })
    }

    pub fn plane(surface_normal: Vec3, plane_constant: f32) -> Self {
        Self(PhysicsShape::Plane {
            surface_normal,
            plane_constant,
        })
    }
}

/// Jolt body id owned by an entity. Inserted by the spawn observer;
/// game code reads it but never writes it.
#[derive(Component, Clone, Copy, Debug)]
pub struct JoltBodyId {
    pub body_id_raw: u32,
}

/// Physics transform from the previous Fixed tick. Written by the tick sync,
/// read by the render interpolation. Lets meshes glide between ticks instead
/// of snapping at tick rate.
#[derive(Component, Clone, Copy, Debug)]
pub struct PreviousBodyTransform {
    pub previous_position: Vec3,
    pub previous_rotation: Quat,
}
/// Creates the Jolt body when [`JoltBody`] is added: reads the entity's
/// `Transform` as the spawn pose and stores the resulting [`JoltBodyId`].
pub fn spawn_jolt_body(
    trigger: On<Add, JoltBody>,
    mut commands: Commands,
    body_query: Query<(&JoltBody, &JoltShape, &Transform)>,
    mut physics_world: ResMut<JoltPhysicsWorld>,
) {
    let trigger_entity = trigger.event().entity;
    let Ok((body, shape, spawn_transform)) = body_query.get(trigger_entity) else {
        panic!(
            "JoltBody added without a JoltShape + Transform on {:?}: shape and spawn pose are required",
            trigger_entity
        );
    };

    let spawn_position = spawn_transform.translation;
    let body_id_raw = match shape.0 {
        PhysicsShape::Box { half_extents } => physics_world.create_box(
            half_extents,
            spawn_position,
            body.object_layer,
            body.motion,
            body.density_kg_per_m3,
        ),
        PhysicsShape::Sphere { sphere_radius } => physics_world.create_sphere(
            sphere_radius,
            spawn_position,
            body.object_layer,
            body.density_kg_per_m3,
        ),
        PhysicsShape::Capsule {
            capsule_half_height,
            capsule_radius,
        } => physics_world.create_capsule(
            capsule_half_height,
            capsule_radius,
            spawn_position,
            body.object_layer,
            body.density_kg_per_m3,
        ),
        PhysicsShape::Cylinder {
            cylinder_half_height,
            cylinder_radius,
        } => physics_world.create_cylinder(
            cylinder_half_height,
            cylinder_radius,
            spawn_position,
            body.object_layer,
            body.density_kg_per_m3,
        ),
        PhysicsShape::TaperedCylinder {
            tapered_half_height,
            top_radius,
            bottom_radius,
        } => physics_world.create_tapered_cylinder(
            tapered_half_height,
            top_radius,
            bottom_radius,
            spawn_position,
            body.object_layer,
            body.density_kg_per_m3,
        ),
        PhysicsShape::TaperedCapsule {
            tapered_half_height,
            top_radius,
            bottom_radius,
        } => physics_world.create_tapered_capsule(
            tapered_half_height,
            top_radius,
            bottom_radius,
            spawn_position,
            body.object_layer,
            body.density_kg_per_m3,
        ),
        PhysicsShape::Plane {
            surface_normal,
            plane_constant,
        } => physics_world.create_plane(
            surface_normal,
            plane_constant,
            50.0,
            body.object_layer,
        ),
    };
    commands.entity(trigger_entity).insert((
        JoltBodyId { body_id_raw },
        PreviousBodyTransform {
            previous_position: spawn_transform.translation,
            previous_rotation: spawn_transform.rotation,
        },
    ));
}

/// Copies each body's physics transform into its entity's `Transform` and
/// remembers the previous tick's pose. Runs after the physics step in the
/// same Fixed tick; the render interpolation blends between the two.
pub fn sync_body_transforms(
    mut body_query: Query<(&JoltBodyId, &mut Transform, &mut PreviousBodyTransform)>,
    physics_world: Res<JoltPhysicsWorld>,
) {
    for (body_id, mut entity_transform, mut previous_transform) in body_query.iter_mut() {
        previous_transform.previous_position = entity_transform.translation;
        previous_transform.previous_rotation = entity_transform.rotation;
        let (body_position, body_rotation) =
            physics_world.body_full_transform(body_id.body_id_raw);
        entity_transform.translation = body_position;
        entity_transform.rotation = body_rotation;
    }
}

/// Blends each entity's `Transform` between the previous and current tick's
/// physics pose using the Fixed overstep fraction. Reads the tick pose from
/// Jolt (never from the blended `Transform`), so repeated frames can't
/// accumulate error: the same tick always blends to the same picture.
/// Static bodies skip the blend; their pose never changes.
pub fn interpolate_body_transforms(
    fixed_time: Res<Time<Fixed>>,
    mut body_query: Query<(
        &JoltBody,
        &JoltBodyId,
        &mut Transform,
        &PreviousBodyTransform,
    )>,
    physics_world: Res<JoltPhysicsWorld>,
) {
    let blend_factor = fixed_time.overstep_fraction().clamp(0.0, 1.0);
    for (body, body_id, mut entity_transform, previous_transform) in
        body_query.iter_mut()
    {
        // Only static bodies skip interpolation; kinematic bodies move via
        // MoveKinematic and need the same render blending as dynamics.
        if body.motion == JoltMotion::Static {
            continue;
        }
        let (tick_position, tick_rotation) =
            physics_world.body_full_transform(body_id.body_id_raw);
        entity_transform.translation = previous_transform
            .previous_position
            .lerp(tick_position, blend_factor);
        entity_transform.rotation = previous_transform
            .previous_rotation
            .slerp(tick_rotation, blend_factor);
    }
}

/// Destroys the Jolt body when its entity is despawned.
pub fn despawn_jolt_body(
    trigger: On<Remove, JoltBodyId>,
    body_query: Query<&JoltBodyId>,
    mut physics_world: ResMut<JoltPhysicsWorld>,
) {
    let trigger_entity = trigger.event().entity;
    let Ok(body_id) = body_query.get(trigger_entity) else {
        panic!(
            "JoltBodyId gone on {:?} before despawn ran",
            trigger_entity
        );
    };
    physics_world.remove_and_destroy_body(body_id.body_id_raw);
}
