use bevy::prelude::*;

use crate::physics_world::{CompoundPart, PhysicsShape};
use crate::plugin::JoltPhysicsWorld;

/// Rigid-body descriptor: motion type + collision layer. Add alongside a
/// [`JoltShape`] and a `Transform`; the plugin creates the Jolt body and
/// keeps them in sync. Tuning lives on sibling components
/// ([`JoltDensity`](crate::body_forces::JoltDensity),
/// [`JoltGravity`](crate::body_forces::JoltGravity),
/// [`JoltFriction`](crate::body_forces::JoltFriction),
/// [`JoltRestitution`](crate::body_forces::JoltRestitution),
/// [`JoltCcd`](crate::body_forces::JoltCcd),
/// [`JoltDamping`](crate::body_forces::JoltDamping)): each is read at bake
/// when present and pushed live on `Changed` writes. Geometry lives on the
/// (shareable) shape.
#[derive(Component, Clone, Copy, Debug)]
pub struct JoltBody {
    pub motion: JoltMotion,
    pub object_layer: u16,
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
    pub fn dynamic(object_layer: u16) -> Self {
        Self {
            motion: JoltMotion::Dynamic,
            object_layer,
        }
    }

    pub fn fixed(object_layer: u16) -> Self {
        Self {
            motion: JoltMotion::Static,
            object_layer,
        }
    }

    pub fn kinematic(object_layer: u16) -> Self {
        Self {
            motion: JoltMotion::Kinematic,
            object_layer,
        }
    }
}

/// Pure collision geometry for one physics body. Add alongside [`JoltBody`].
/// Mirrors Jolt's `Shape`: immutable geometry with no motion of its own.
/// Shared ownership: the component files the same allocation the debug
/// table reads, so terrain-sized geometry lives once, not twice.
#[derive(Component, Clone, Debug)]
pub struct JoltShape(pub std::sync::Arc<PhysicsShape>);

impl JoltShape {
    pub fn box_shape(half_extents: Vec3) -> Self {
        Self(std::sync::Arc::new(PhysicsShape::Box { half_extents }))
    }

    pub fn sphere(sphere_radius: f32) -> Self {
        Self(std::sync::Arc::new(PhysicsShape::Sphere {
            sphere_radius,
        }))
    }

    pub fn capsule(capsule_half_height: f32, capsule_radius: f32) -> Self {
        Self(std::sync::Arc::new(PhysicsShape::Capsule {
            capsule_half_height,
            capsule_radius,
        }))
    }

    pub fn cylinder(cylinder_half_height: f32, cylinder_radius: f32) -> Self {
        Self(std::sync::Arc::new(PhysicsShape::Cylinder {
            cylinder_half_height,
            cylinder_radius,
        }))
    }

    pub fn tapered_cylinder(
        tapered_half_height: f32,
        top_radius: f32,
        bottom_radius: f32,
    ) -> Self {
        Self(std::sync::Arc::new(PhysicsShape::TaperedCylinder {
            tapered_half_height,
            top_radius,
            bottom_radius,
        }))
    }

    pub fn tapered_capsule(
        tapered_half_height: f32,
        top_radius: f32,
        bottom_radius: f32,
    ) -> Self {
        Self(std::sync::Arc::new(PhysicsShape::TaperedCapsule {
            tapered_half_height,
            top_radius,
            bottom_radius,
        }))
    }

    pub fn plane(surface_normal: Vec3, plane_constant: f32) -> Self {
        Self(std::sync::Arc::new(PhysicsShape::Plane {
            surface_normal,
            plane_constant,
        }))
    }

    /// One body from box/sphere/capsule parts (a table = top + legs, a
    /// hammer = head + handle). At most 16 parts; empty rejects at bake.
    pub fn compound(compound_parts: Vec<CompoundPart>) -> Self {
        Self(std::sync::Arc::new(PhysicsShape::Compound {
            compound_parts,
        }))
    }

    /// Shrink-wrapped convex lump from a point soup. Any motion: dynamics
    /// tumble it. Rocks, crates, wreckage.
    pub fn hull(hull_points: Vec<Vec3>) -> Self {
        Self(std::sync::Arc::new(PhysicsShape::Hull { hull_points }))
    }

    /// Exact-triangle static scenery. Static only: bake panics otherwise.
    /// Archways, stairs meshes, rubble.
    pub fn mesh(mesh_vertices: Vec<Vec3>, mesh_triangles: Vec<[u32; 3]>) -> Self {
        Self(std::sync::Arc::new(PhysicsShape::Mesh {
            mesh_vertices,
            mesh_triangles,
        }))
    }

    /// Terrain grid: `grid_width * grid_width` heights, `cell_size` meters
    /// apart, centered on the spawn. Static only: bake panics otherwise.
    /// Cheaper than the equivalent mesh at scale.
    pub fn heightfield(field_heights: Vec<f32>, field_width: u32, field_cell: f32) -> Self {
        Self(std::sync::Arc::new(PhysicsShape::Heightfield {
            field_heights,
            field_width,
            field_cell,
        }))
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
/// Tuning components present at spawn
/// ([`JoltDensity`](crate::body_forces::JoltDensity),
/// [`JoltGravity`](crate::body_forces::JoltGravity),
/// [`JoltFriction`](crate::body_forces::JoltFriction),
/// [`JoltRestitution`](crate::body_forces::JoltRestitution),
/// [`JoltCcd`](crate::body_forces::JoltCcd),
/// [`JoltDamping`](crate::body_forces::JoltDamping)) land on the body the
/// same tick: density + gravity ride the creation call, the rest apply right
/// after. Later writes flow through the pre-step `Changed` pushes.
pub fn spawn_jolt_body(
    trigger: On<Add, JoltBody>,
    mut commands: Commands,
    body_query: Query<(
        &JoltBody,
        &JoltShape,
        &Transform,
        Option<&crate::body_forces::JoltDensity>,
        Option<&crate::body_forces::JoltGravity>,
        Option<&crate::body_forces::JoltFriction>,
        Option<&crate::body_forces::JoltRestitution>,
        Option<&crate::body_forces::JoltCcd>,
        Option<&crate::body_forces::JoltDamping>,
    )>,
    mut physics_world: ResMut<JoltPhysicsWorld>,
) {
    /// Jolt's own creation defaults: creation takes no friction-style args,
    /// so these only apply when the matching component is absent.
    const DEFAULT_DENSITY: f32 = 1000.0;
    const DEFAULT_GRAVITY: f32 = 1.0;
    const DEFAULT_FRICTION: f32 = 0.2;
    const DEFAULT_RESTITUTION: f32 = 0.0;

    let trigger_entity = trigger.event().entity;
    let Ok((
        body,
        shape,
        spawn_transform,
        spawn_density,
        spawn_gravity,
        spawn_friction,
        spawn_restitution,
        spawn_ccd,
        spawn_damping,
    )) = body_query.get(trigger_entity) else {
        panic!(
            "JoltBody added without a JoltShape + Transform on {:?}: shape and spawn pose are required",
            trigger_entity
        );
    };
    let spawn_density = spawn_density.map_or(DEFAULT_DENSITY, |density| {
        assert!(
            density.density_kg_per_m3.is_finite() && density.density_kg_per_m3 > 0.0,
            "density must be positive, got {}",
            density.density_kg_per_m3
        );
        density.density_kg_per_m3
    });
    let spawn_gravity = spawn_gravity.map_or(DEFAULT_GRAVITY, |gravity| {
        assert!(
            gravity.gravity_factor.is_finite() && gravity.gravity_factor >= 0.0,
            "gravity factor must be non-negative, got {}",
            gravity.gravity_factor
        );
        gravity.gravity_factor
    });
    let spawn_friction = spawn_friction.map_or(DEFAULT_FRICTION, |friction| {
        assert!(
            friction.friction.is_finite() && friction.friction >= 0.0,
            "friction must be non-negative, got {}",
            friction.friction
        );
        friction.friction
    });
    let spawn_restitution = spawn_restitution.map_or(DEFAULT_RESTITUTION, |restitution| {
        assert!(
            restitution.restitution.is_finite() && (0.0..=1.0).contains(&restitution.restitution),
            "restitution must be 0..=1, got {}",
            restitution.restitution
        );
        restitution.restitution
    });

    let spawn_position = spawn_transform.translation;
    let body_id_raw = match shape.0.as_ref() {
        PhysicsShape::Box { half_extents } => physics_world.create_box(
            *half_extents,
            spawn_position,
            body.object_layer,
            body.motion,
            spawn_density,
            spawn_gravity,
        ),
        PhysicsShape::Sphere { sphere_radius } => physics_world.create_sphere(
            *sphere_radius,
            spawn_position,
            body.object_layer,
            spawn_density,
            spawn_gravity,
        ),
        PhysicsShape::Capsule {
            capsule_half_height,
            capsule_radius,
        } => physics_world.create_capsule(
            *capsule_half_height,
            *capsule_radius,
            spawn_position,
            body.object_layer,
            spawn_density,
            spawn_gravity,
        ),
        PhysicsShape::Cylinder {
            cylinder_half_height,
            cylinder_radius,
        } => physics_world.create_cylinder(
            *cylinder_half_height,
            *cylinder_radius,
            spawn_position,
            body.object_layer,
            spawn_density,
            spawn_gravity,
        ),
        PhysicsShape::TaperedCylinder {
            tapered_half_height,
            top_radius,
            bottom_radius,
        } => physics_world.create_tapered_cylinder(
            *tapered_half_height,
            *top_radius,
            *bottom_radius,
            spawn_position,
            body.object_layer,
            spawn_density,
            spawn_gravity,
        ),
        PhysicsShape::TaperedCapsule {
            tapered_half_height,
            top_radius,
            bottom_radius,
        } => physics_world.create_tapered_capsule(
            *tapered_half_height,
            *top_radius,
            *bottom_radius,
            spawn_position,
            body.object_layer,
            spawn_density,
            spawn_gravity,
        ),
        PhysicsShape::Plane {
            surface_normal,
            plane_constant,
        } => physics_world.create_plane(
            *surface_normal,
            *plane_constant,
            50.0,
            body.object_layer,
        ),
        PhysicsShape::Compound { compound_parts } => physics_world.create_compound(
            compound_parts,
            spawn_position,
            body.object_layer,
            body.motion,
            spawn_density,
            spawn_gravity,
        ),
        PhysicsShape::Hull { hull_points } => physics_world.create_hull(
            hull_points,
            spawn_position,
            body.object_layer,
            body.motion,
            spawn_density,
            spawn_gravity,
        ),
        PhysicsShape::Mesh {
            mesh_vertices,
            mesh_triangles,
        } => {
            assert!(
                body.motion == JoltMotion::Static,
                "mesh shapes are static-only, got {:?}",
                body.motion
            );
            physics_world.create_mesh(
                mesh_vertices,
                mesh_triangles,
                spawn_position,
                body.object_layer,
            )
        }
        PhysicsShape::Heightfield {
            field_heights,
            field_width,
            field_cell,
        } => {
            assert!(
                body.motion == JoltMotion::Static,
                "heightfields are static-only, got {:?}",
                body.motion
            );
            physics_world.create_heightfield(
                field_heights,
                *field_width,
                *field_cell,
                spawn_position,
                body.object_layer,
            )
        }
    };
    // One allocation for component + debug table: the entity keeps its
    // descriptor, the table shares it.
    physics_world.file_shape(body_id_raw, std::sync::Arc::clone(&shape.0));
    // Creation bakes identity rotation, so rotate the live body into the
    // spawn pose before the first step. Non-identity only: saves an FFI
    // round-trip for the common unrotated case.
    if spawn_transform.rotation != Quat::IDENTITY {
        physics_world.set_body_rotation(body_id_raw, spawn_transform.rotation);
    }
    // Creation takes no friction-style args, so apply present components
    // here. Absent components keep Jolt's own defaults, skipping the FFI
    // round-trip. Later writes flow through the pre-step `Changed` pushes.
    if spawn_friction != DEFAULT_FRICTION {
        physics_world.set_body_friction(body_id_raw, spawn_friction);
    }
    if spawn_restitution != DEFAULT_RESTITUTION {
        physics_world.set_body_restitution(body_id_raw, spawn_restitution);
    }
    if spawn_ccd.map_or(false, |ccd| ccd.use_ccd) {
        physics_world.set_body_ccd(body_id_raw, true);
    }
    // Damping rides the same path: creation takes no damping args, so apply
    // the spawn component here when present. Later writes flow through the
    // `Changed` push in `sync_jolt_damping`.
    if let Some(spawn_damping) = spawn_damping {
        physics_world.set_body_damping(
            body_id_raw,
            spawn_damping.linear_damping,
            spawn_damping.angular_damping,
        );
    }
    let mut baked_entity = commands.entity(trigger_entity);
    baked_entity.insert((
        JoltBodyId { body_id_raw },
        crate::body_forces::JoltSleeping { sleeping: false },
        PreviousBodyTransform {
            previous_position: spawn_transform.translation,
            previous_rotation: spawn_transform.rotation,
        },
    ));
    // Non-static bodies carry unified velocity from bake: game code drives
    // by writing, the sync writes the measured result back (change
    // detection bypassed). Statics never move, so they skip both.
    if body.motion != JoltMotion::Static {
        baked_entity.insert((
            crate::body_forces::JoltLinearVelocity {
                linear_velocity: Vec3::ZERO,
            },
            crate::body_forces::JoltAngularVelocity {
                angular_velocity: Vec3::ZERO,
            },
        ));
    }
}
/// Copies each body's physics transform into its entity's `Transform` and
/// remembers the previous tick's pose. Same pass writes the measured
/// velocities back into the unified components (pose + motion come from one
/// FFI call, so readback costs no extra round-trip). The writeback bypasses
/// change detection: it updates what you read without looking like a new
/// drive request. Sleeping bodies are filtered out: frozen means frozen, no
/// FFI for values that cannot change. Runs after the physics step in the
/// same Fixed tick; the render interpolation blends between the two poses.
pub fn sync_body_transforms(
    mut body_query: Query<(
        &JoltBodyId,
        &crate::body_forces::JoltSleeping,
        &mut Transform,
        &mut PreviousBodyTransform,
        Option<&mut crate::body_forces::JoltLinearVelocity>,
        Option<&mut crate::body_forces::JoltAngularVelocity>,
    )>,
    physics_world: Res<JoltPhysicsWorld>,
) {
    for (
        body_id,
        body_sleeping,
        mut entity_transform,
        mut previous_transform,
        unified_linear,
        unified_angular,
    ) in body_query.iter_mut()
    {
        if **body_sleeping {
            continue;
        }
        previous_transform.previous_position = entity_transform.translation;
        previous_transform.previous_rotation = entity_transform.rotation;
        let body_motion = physics_world.body_full_motion(body_id.body_id_raw);
        entity_transform.translation = body_motion.body_position;
        entity_transform.rotation = body_motion.body_rotation;
        if let Some(mut unified_linear) = unified_linear {
            unified_linear
                .bypass_change_detection()
                .linear_velocity = body_motion.body_velocity;
        }
        if let Some(mut unified_angular) = unified_angular {
            unified_angular
                .bypass_change_detection()
                .angular_velocity = body_motion.body_spin;
        }
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

/// Destroys the Jolt body when its entity is despawned. Skips ragdoll parts:
/// their ids die with the whole ragdoll in `despawn_jolt_ragdoll`
/// (double destroy crashes).
pub fn despawn_jolt_body(
 trigger: On<Remove, JoltBodyId>,
 body_query: Query<&JoltBodyId>,
 part_bodies: Query<(), With<crate::ragdoll::RagdollPartBody>>,
 mut physics_world: ResMut<JoltPhysicsWorld>,
) {
 let trigger_entity = trigger.event().entity;
 if part_bodies.contains(trigger_entity) {
 return;
 }
 let Ok(body_id) = body_query.get(trigger_entity) else {
 panic!(
 "JoltBodyId gone on {:?} before despawn ran",
 trigger_entity
 );
 };
 physics_world.remove_and_destroy_body(body_id.body_id_raw);
}
