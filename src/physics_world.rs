//! Owned Jolt physics world: lifetime, body API, stepping.
//!
//! The solver itself runs in C++ on Jolt's ThreadPool job system (SIMD stays
//! on via the `jolt_sys` build). This type only owns the world pointer.

use crate::body_sync::JoltMotion;
use crate::ragdoll::RagdollDriveAxis;
use crate::spatial_queries::RayHit;
use bevy::prelude::{Dir3, Quat, Vec3};
use jolt_sys::{
    BJoltWorld, VehicleDifferentialFfi, VehicleEngineFfi, VehicleLeanFfi, VehicleRollBarFfi,
    VehicleTransmissionFfi, VehicleWheelFfi, bjolt_apply_buoyancy, bjolt_apply_force,
    bjolt_apply_impulse, bjolt_bodies_no_collide, bjolt_body_is_active, bjolt_body_remove_destroy,
    bjolt_body_set_damping, bjolt_body_set_density, bjolt_body_set_sensor, bjolt_body_state,
    bjolt_body_transform, bjolt_character_can_walk_stairs, bjolt_character_create,
    bjolt_character_destroy, bjolt_character_ground, bjolt_character_move,
    bjolt_character_refresh_contacts, bjolt_character_rotation, bjolt_character_set_mass,
    bjolt_character_set_padding, bjolt_character_set_rotation, bjolt_character_set_shape_offset,
    bjolt_character_set_up, bjolt_character_set_user_data, bjolt_character_stance,
    bjolt_character_stick_to_floor, bjolt_character_teleport, bjolt_character_update,
    bjolt_character_walk_stairs, bjolt_constraint_drive_at,
    bjolt_constraint_drive_swing_twist, bjolt_constraint_path_fraction,
    bjolt_constraint_path_looping, bjolt_create_box, bjolt_create_capsule,
    bjolt_create_cloth_settings, bjolt_create_compound, bjolt_create_cone_constraint,
    bjolt_create_cube_settings, bjolt_create_cylinder, bjolt_create_distance_constraint,
    bjolt_create_fixed_constraint, bjolt_create_floor, bjolt_create_gear_constraint,
    bjolt_create_heightfield, bjolt_create_hinge_constraint, bjolt_create_hull, bjolt_create_mesh,
    bjolt_create_motorcycle, bjolt_create_path_cart, bjolt_create_plane,
    bjolt_create_point_constraint, bjolt_create_pulley_constraint,
    bjolt_create_rack_pinion_constraint, bjolt_create_shared_settings, bjolt_create_six_dof,
    bjolt_create_slider_constraint, bjolt_create_soft_body, bjolt_create_sphere,
    bjolt_create_sphere_settings, bjolt_create_swing_twist_constraint,
    bjolt_create_tapered_capsule, bjolt_create_tapered_cylinder, bjolt_create_tracked_vehicle,
    bjolt_create_wheeled_vehicle, bjolt_destroy_shared_settings, bjolt_drain_contact_added,
    bjolt_drain_contact_removed, bjolt_drain_slept, bjolt_drain_woke, bjolt_gravity_factor,
    bjolt_init, bjolt_move_kinematic, bjolt_ragdoll_body_count, bjolt_ragdoll_body_ids,
    bjolt_ragdoll_build_add_part, bjolt_ragdoll_build_create, bjolt_ragdoll_build_destroy,
    bjolt_ragdoll_build_finalize, bjolt_ragdoll_build_set_hinge,
    bjolt_ragdoll_build_set_swing_twist, bjolt_ragdoll_build_stabilize, bjolt_ragdoll_create,
    bjolt_ragdoll_destroy, bjolt_ragdoll_drive, bjolt_ragdoll_motor_off, bjolt_ragdoll_set_layer,
    bjolt_ragdoll_set_motion,
    bjolt_remove_constraint, bjolt_rigid_character_add_impulse, bjolt_rigid_character_add_velocity,
    bjolt_rigid_character_body, bjolt_rigid_character_create, bjolt_rigid_character_destroy,
    bjolt_rigid_character_ground, bjolt_rigid_character_pose, bjolt_rigid_character_post,
    bjolt_rigid_character_set_layer, bjolt_rigid_character_set_pose,
    bjolt_rigid_character_set_velocity, bjolt_rigid_character_stance, bjolt_set_angular_velocity,
    bjolt_set_ccd, bjolt_set_friction, bjolt_set_gravity, bjolt_set_gravity_factor,
    bjolt_set_linear_velocity, bjolt_set_motion_type, bjolt_set_position,
    bjolt_set_position_rotation, bjolt_set_restitution, bjolt_set_rotation, bjolt_set_velocity,
    bjolt_shared_face_count, bjolt_shared_faces, bjolt_shared_vertex_count, bjolt_sleep_body,
    bjolt_soft_contacts, bjolt_soft_destroy, bjolt_soft_inv_masses, bjolt_soft_iterations,
    bjolt_soft_pressure, bjolt_soft_push, bjolt_soft_set_inv_masses, bjolt_soft_set_iterations,
    bjolt_soft_set_pressure, bjolt_soft_set_vertex_radius, bjolt_soft_velocities,
    bjolt_soft_vertex_count, bjolt_soft_vertex_radius, bjolt_soft_vertices, bjolt_soft_volume,
    bjolt_tracked_drive, bjolt_vehicle_drive, bjolt_vehicle_shift, bjolt_wake_body,
    bjolt_world_body_count, bjolt_world_create_with_layers, bjolt_world_destroy, bjolt_world_gravity,
    bjolt_world_update,
};

/// Which frame joint anchors/axes live in. `World` takes global positions
/// and directions; `LocalToBodyCom` takes them relative to each body's
/// center of mass (NOT the Bevy `Transform` origin). Re-exported from
/// `jolt_sys` constants so the FFI discriminant stays in one place.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum JointSpace {
    #[default]
    World,
    LocalToBodyCom,
}

impl JointSpace {
    fn ffi_space(self) -> u8 {
        match self {
            JointSpace::World => jolt_sys::JOINT_SPACE_WORLD,
            JointSpace::LocalToBodyCom => jolt_sys::JOINT_SPACE_LOCAL_TO_BODY_COM,
        }
    }
}

/// Which bend constraint the shared-settings builders create.
/// Mirrors Jolt's `EBendType`: none, cheap distance, or expensive
/// dihedral (handles triangles not starting in the same plane).
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum SoftBendType {
    #[default]
    None,
    Distance,
    Dihedral,
}

/// Full Jolt soft-body creation settings, one field per C++ knob.
/// Passed to [`JoltWorld::create_soft_body`]; nothing hides in FFI defaults.
#[derive(Clone, Debug)]
pub struct SoftBodyConfig {
    /// Spawn position of the soft body.
    pub body_position: Vec3,
    /// Spawn rotation of the soft body.
    pub body_rotation: Quat,
    /// Collision layer, same teams as rigid bodies.
    pub object_layer: u16,
    /// Solver iterations for this body.
    pub num_iterations: u32,
    /// Linear damping: dv/dt = -damping * v, near zero.
    pub linear_damping: f32,
    /// Fastest any vertex may travel (m/s).
    pub max_linear_velocity: f32,
    /// Bounciness on contact.
    pub restitution: f32,
    /// Grip on contact.
    pub friction: f32,
    /// Balloon pressure (n * R * T); inflates closed shapes.
    pub pressure: f32,
    /// Gravity scale for this body.
    pub gravity_factor: f32,
    /// Particle skin thickness: pushes verts off surfaces.
    pub vertex_radius: f32,
    /// False pins the body to the static world (only free verts move).
    pub update_position: bool,
    /// Bakes rotation into verts, keeps body rotation identity (more exact).
    pub make_rotation_identity: bool,
    /// Whether the body may sleep.
    pub allow_sleeping: bool,
    /// Faces collide from both sides (ray casts, shape queries).
    pub faces_double_sided: bool,
    /// App user data.
    pub user_data: u64,
}

impl Default for SoftBodyConfig {
    fn default() -> Self {
        Self {
            body_position: Vec3::ZERO,
            body_rotation: Quat::IDENTITY,
            object_layer: 0,
            num_iterations: 5,
            linear_damping: 0.1,
            max_linear_velocity: 500.0,
            restitution: 0.0,
            friction: 0.2,
            pressure: 0.0,
            gravity_factor: 1.0,
            vertex_radius: 0.0,
            update_position: true,
            make_rotation_identity: true,
            allow_sleeping: true,
            faces_double_sided: false,
            user_data: 0,
        }
    }
}

/// Degrees of freedom for a rigid character body. Bitmask mirroring Jolt's
/// `EAllowedDOFs`: lock axes the game never drives (e.g. freeze rotation
/// for an upright capsule).
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct CharacterDofs(pub u8);

impl CharacterDofs {
    /// All translation + rotation.
    pub const ALL: Self = Self(0b111111);
    /// Upright capsule: moves freely, never tips.
    pub const TRANSLATION_ONLY: Self = Self(0b000111);

    fn ffi_dofs(self) -> u8 {
        self.0
    }
}

/// Position plus linear and angular velocity of one body at the current step.
pub struct BodySnapshot {
    pub body_position: Vec3,
    pub body_velocity: Vec3,
    pub body_spin: Vec3,
}

/// Everything the per-tick sync reads from one body in a single FFI call:
/// pose for the entity transform, velocities for the measured components.
pub struct BodyMotion {
    pub body_position: Vec3,
    pub body_rotation: Quat,
    pub body_velocity: Vec3,
    pub body_spin: Vec3,
}

/// What a body looks like, remembered at creation so the debug visualizer
/// can draw it. Jolt owns the real shape; this is just the outline recipe.
/// `Clone` (not `Copy`): a compound owns a heap part list, and copying that
/// implicitly per frame would hide real cost.
#[derive(Clone, Debug)]
pub enum PhysicsShape {
    Box {
        half_extents: Vec3,
    },
    Sphere {
        sphere_radius: f32,
    },
    Capsule {
        capsule_half_height: f32,
        capsule_radius: f32,
    },
    Cylinder {
        cylinder_half_height: f32,
        cylinder_radius: f32,
    },
    TaperedCylinder {
        tapered_half_height: f32,
        top_radius: f32,
        bottom_radius: f32,
    },
    TaperedCapsule {
        tapered_half_height: f32,
        top_radius: f32,
        bottom_radius: f32,
    },
    Plane {
        surface_normal: Vec3,
        plane_constant: f32,
    },
    Compound {
        compound_parts: Vec<CompoundPart>,
    },
    /// Shrink-wrapped lump from a point soup: dents filled, convex only, so
    /// dynamics can tumble it. Rocks, crates, wreckage.
    Hull {
        hull_points: Vec<Vec3>,
    },
    /// Exact-triangle static scenery: keeps every dent and hole. Static
    /// only (bake rejects motion): archways, stairs meshes, rubble.
    Mesh {
        mesh_vertices: Vec<Vec3>,
        mesh_triangles: Vec<[u32; 3]>,
    },
    /// Terrain grid: one height per cell, grid lookup instead of a tree
    /// walk. Static only. Cheaper than the equivalent mesh at scale;
    /// no overhangs by construction.
    Heightfield {
        field_heights: Vec<f32>,
        field_width: u32,
        field_cell: f32,
    },
}

/// One chunk of a compound body: a box, sphere, or capsule posed relative to
/// the body origin. Offsets in meters, rotation as a quaternion.
#[derive(Clone, Copy, Debug)]
pub struct CompoundPart {
    pub part_geometry: CompoundGeometry,
    pub part_offset: Vec3,
    pub part_rotation: Quat,
}

/// Box spans half extents; sphere uses one radius; capsule pairs the
/// cylinder half height with the cap radius.
#[derive(Clone, Copy, Debug)]
pub enum CompoundGeometry {
    Box {
        part_half_extents: Vec3,
    },
    Sphere {
        part_radius: f32,
    },
    Capsule {
        part_half_height: f32,
        part_radius: f32,
    },
}

/// Which object layers exist and which pairs can collide, decided in Rust
/// values with no built-in names: declare every team your game needs up
/// front with [`CollisionLayers::new`] and wire who-hits-who with
/// [`CollisionLayers::set_collide`].
#[derive(Clone, Debug)]
pub struct CollisionLayers {
    layer_count: usize,
    collide_matrix: [[u8; jolt_sys::MAX_OBJECT_LAYERS]; jolt_sys::MAX_OBJECT_LAYERS],
}

impl CollisionLayers {
    /// Single self-colliding layer 0: every body on layer 0 hits every
    /// other body on layer 0. Enough for demos with one team; real games
    /// declare their own table with [`CollisionLayers::new`].
    pub fn single_layer() -> Self {
        let mut collide_matrix = [[0u8; jolt_sys::MAX_OBJECT_LAYERS]; jolt_sys::MAX_OBJECT_LAYERS];
        collide_matrix[0][0] = 1;
        Self {
            layer_count: 1,
            collide_matrix,
        }
    }

    pub fn new(layer_count: usize) -> Self {
        assert!(
            (1..=jolt_sys::MAX_OBJECT_LAYERS).contains(&layer_count),
            "layer count {} out of range 1..={}",
            layer_count,
            jolt_sys::MAX_OBJECT_LAYERS
        );
        // Every layer collides with everything until told otherwise; carve
        // out the pairs that should not meet with `set_collide`.
        let mut collide_matrix = [[0u8; jolt_sys::MAX_OBJECT_LAYERS]; jolt_sys::MAX_OBJECT_LAYERS];
        for first_layer in 0..layer_count {
            for other_layer in 0..layer_count {
                collide_matrix[first_layer][other_layer] = 1;
            }
        }
        Self {
            layer_count,
            collide_matrix,
        }
    }

    /// Set whether two layers can collide. Always stored symmetric: setting
    /// (a, b) also sets (b, a), since Jolt asks the question both ways.
    pub fn set_collide(&mut self, first_layer: u16, second_layer: u16, can_collide: bool) {
        assert!(
            (first_layer as usize) < self.layer_count,
            "layer {} beyond count {}",
            first_layer,
            self.layer_count
        );
        assert!(
            (second_layer as usize) < self.layer_count,
            "layer {} beyond count {}",
            second_layer,
            self.layer_count
        );
        let collide_flag = u8::from(can_collide);
        self.collide_matrix[first_layer as usize][second_layer as usize] = collide_flag;
        self.collide_matrix[second_layer as usize][first_layer as usize] = collide_flag;
    }
}

impl Default for CollisionLayers {
    fn default() -> Self {
        Self::single_layer()
    }
}
/// Fixed-size Jolt budgets, decided once at world creation: Jolt
/// preallocates and never grows. Size for the biggest scene, not the
/// smallest: an exhausted body pool fails creates (loudly, via the wrapper
/// guard), while overflowing pairs/contacts silently drops collisions for
/// a step (ghosting). Rule of thumb: pairs ≈ 10–20× bodies.
#[derive(Clone, Copy, Debug)]
pub struct JoltWorldBudgets {
    /// Max live bodies. One slot per body regardless of shape.
    pub max_bodies: u32,
    /// Max broadphase pair candidates per step.
    pub max_body_pairs: u32,
    /// Max solved contacts per step.
    pub max_contact_constraints: u32,
    /// Solver scratch bytes per step.
    pub temp_allocator_bytes: u64,
}

impl Default for JoltWorldBudgets {
    fn default() -> Self {
        Self {
            max_bodies: 4096,
            max_body_pairs: 4096,
            max_contact_constraints: 4096,
            temp_allocator_bytes: 32 * 1024 * 1024,
        }
    }
}

/// A Jolt physics world: floor + dynamic bodies, stepped on the ThreadPool job system.
pub struct JoltWorld {
    world_ptr: *mut BJoltWorld,
    body_shapes: std::collections::HashMap<u32, std::sync::Arc<PhysicsShape>>,
    character_positions:
        std::collections::HashMap<u32, (Vec3, crate::character::JoltCharacterGround)>,
    character_shapes: std::collections::HashMap<u32, (f32, f32)>,
    rigid_character_shapes: std::collections::HashMap<u32, (f32, f32)>,
    next_ragdoll_group: u32,
    sensor_bodies: std::collections::HashSet<u32>,
    /// Sim speed multiplier: 1 = real time, 0.2 = slow motion, 2 = double
    /// speed. Every tick delta the sim consumes is multiplied by this, so
    /// the fixed step stays fixed and the world just advances less (or
    /// more) per tick. The render interpolation already smooths the rest.
    time_scale: f32,
}

impl JoltWorld {
    pub fn new() -> Self {
        Self::with_layers(CollisionLayers::default())
    }

    pub fn with_layers(collision_layers: CollisionLayers) -> Self {
        Self::with_layers_and_budgets(collision_layers, JoltWorldBudgets::default())
    }

    pub fn with_layers_and_budgets(
        collision_layers: CollisionLayers,
        world_budgets: JoltWorldBudgets,
    ) -> Self {
        assert!(
            world_budgets.max_bodies >= 1,
            "need at least 1 body, got {}",
            world_budgets.max_bodies
        );
        assert!(
            world_budgets.max_body_pairs >= world_budgets.max_bodies,
            "pairs ({}) below bodies ({}): every body can pair, size pairs >= bodies",
            world_budgets.max_body_pairs,
            world_budgets.max_bodies
        );
        assert!(
            world_budgets.max_contact_constraints >= 1,
            "need at least 1 contact, got {}",
            world_budgets.max_contact_constraints
        );
        let world_ptr = unsafe {
            assert!(bjolt_init(), "Jolt initialization failed");
            bjolt_world_create_with_layers(
                collision_layers.layer_count as u32,
                collision_layers.collide_matrix.as_ptr() as *const u8,
                world_budgets.max_bodies,
                world_budgets.max_body_pairs,
                world_budgets.max_contact_constraints,
                world_budgets.temp_allocator_bytes,
            )
        };
        assert!(!world_ptr.is_null(), "Jolt world creation failed");
        Self {
            world_ptr,
            body_shapes: std::collections::HashMap::new(),
            character_positions: std::collections::HashMap::new(),
            character_shapes: std::collections::HashMap::new(),
            rigid_character_shapes: std::collections::HashMap::new(),
            next_ragdoll_group: 1,
            sensor_bodies: std::collections::HashSet::new(),
            time_scale: 1.0,
        }
    }
    /// Live bodies + budget max: log headroom while tuning scene size.
    /// Pairs/contacts have no Jolt-side counter — size those by rule
    /// (see [`JoltWorldBudgets`]); bodies are the ones you can watch.
    pub fn body_stats(&mut self) -> (u32, u32) {
        let mut max_bodies = 0u32;
        let live_bodies =
            unsafe { bjolt_world_body_count(self.world_ptr, &mut max_bodies) };
        (live_bodies, max_bodies)
    }

    /// Current sim speed multiplier (1 = real time).
    pub fn time_scale(&self) -> f32 {
        self.time_scale
    }

    /// Sets the sim speed multiplier: 0.2 = slow motion, 2 = double speed.
    /// Must be finite and non-negative (0 pauses the sim). Takes effect on
    /// the next tick; no rebuild, no state to flush.
    pub fn set_time_scale(&mut self, time_scale: f32) {
        assert!(
            time_scale.is_finite() && time_scale >= 0.0,
            "time scale must be finite and non-negative, got {}",
            time_scale
        );
        self.time_scale = time_scale;
    }

    /// Fixed tick delta scaled to sim time: every system that feeds a delta
    /// into Jolt reads this, so slow motion stays consistent everywhere.
    pub fn sim_tick_delta(&self, fixed_tick_delta: f32) -> f32 {
        fixed_tick_delta * self.time_scale
    }

    /// Every known body and its outline recipe, for the debug visualizer.
    /// Shared ownership: bake files one copy, debug clones the pointer.
    pub fn body_shapes(&self) -> &std::collections::HashMap<u32, std::sync::Arc<PhysicsShape>> {
        &self.body_shapes
    }

    /// Rigid character capsules for the debug visualizer: Jolt owns the
    /// real body; this is just the outline recipe filed at creation.
    pub fn rigid_character_shapes(&self) -> &std::collections::HashMap<u32, (f32, f32)> {
        &self.rigid_character_shapes
    }

    /// Files one shared outline recipe for a body: bake and vehicles call
    /// this with the component's own allocation, so geometry lives once.
    /// Debug draw only; physics owns the real shape.
    pub fn file_shape(&mut self, body_id_raw: u32, outline: std::sync::Arc<PhysicsShape>) {
        self.body_shapes.insert(body_id_raw, outline);
    }

    /// Drops a filed outline (ragdoll teardown). Physics bodies are already
    /// gone; this only clears the debug recipe.
    pub fn unfile_shape(&mut self, body_id_raw: u32) {
        self.body_shapes.remove(&body_id_raw);
    }

    pub fn create_floor(&mut self, half_extents: Vec3, floor_position_height: f32) -> u32 {
        let body_id_raw = unsafe {
            bjolt_create_floor(
                self.world_ptr,
                half_extents.x,
                half_extents.y,
                half_extents.z,
                floor_position_height,
            )
        };
        body_id_raw
    }

    pub fn create_sphere(
        &mut self,
        sphere_radius: f32,
        spawn_position: Vec3,
        object_layer: u16,
        density_kg_per_m3: f32,
        gravity_factor: f32,
    ) -> u32 {
        let body_id_raw = unsafe {
            bjolt_create_sphere(
                self.world_ptr,
                sphere_radius,
                spawn_position.x,
                spawn_position.y,
                spawn_position.z,
                object_layer,
                density_kg_per_m3,
                gravity_factor,
            )
        };
        body_id_raw
    }

    /// True infinite ground: normal + constant define the surface, half
    /// extent only bounds the broadphase box. Static only in Jolt.
    pub fn create_plane(
        &mut self,
        surface_normal: Vec3,
        plane_constant: f32,
        half_extent: f32,
        object_layer: u16,
    ) -> u32 {
        let body_id_raw = unsafe {
            bjolt_create_plane(
                self.world_ptr,
                surface_normal.x,
                surface_normal.y,
                surface_normal.z,
                plane_constant,
                half_extent,
                object_layer,
            )
        };
        body_id_raw
    }

    /// One rigid body from box/sphere/capsule parts posed relative to the
    /// body origin. Empty and over-16 lists assert: Jolt would reject them
    /// with a silent 0.
    pub fn create_compound(
        &mut self,
        compound_parts: &[CompoundPart],
        spawn_position: Vec3,
        object_layer: u16,
        motion: JoltMotion,
        density_kg_per_m3: f32,
        gravity_factor: f32,
    ) -> u32 {
        assert!(
            !compound_parts.is_empty() && compound_parts.len() <= jolt_sys::MAX_COMPOUND_PARTS,
            "compound needs 1..={} parts, got {}",
            jolt_sys::MAX_COMPOUND_PARTS,
            compound_parts.len()
        );
        let ffi_parts: Vec<jolt_sys::CompoundPartFfi> = compound_parts
            .iter()
            .map(|compound_part| {
                let (part_kind, part_half_x, part_half_y, part_half_z) =
                    match compound_part.part_geometry {
                        CompoundGeometry::Box { part_half_extents } => (
                            0,
                            part_half_extents.x,
                            part_half_extents.y,
                            part_half_extents.z,
                        ),
                        CompoundGeometry::Sphere { part_radius } => (1, part_radius, 0.0, 0.0),
                        CompoundGeometry::Capsule {
                            part_half_height,
                            part_radius,
                        } => (2, part_half_height, part_radius, 0.0),
                    };
                jolt_sys::CompoundPartFfi {
                    part_kind,
                    part_half_x,
                    part_half_y,
                    part_half_z,
                    offset_x: compound_part.part_offset.x,
                    offset_y: compound_part.part_offset.y,
                    offset_z: compound_part.part_offset.z,
                    rot_x: compound_part.part_rotation.x,
                    rot_y: compound_part.part_rotation.y,
                    rot_z: compound_part.part_rotation.z,
                    rot_w: compound_part.part_rotation.w,
                }
            })
            .collect();
        let body_id_raw = unsafe {
            bjolt_create_compound(
                self.world_ptr,
                ffi_parts.as_ptr(),
                ffi_parts.len() as u32,
                spawn_position.x,
                spawn_position.y,
                spawn_position.z,
                object_layer,
                motion as u8,
                density_kg_per_m3,
                gravity_factor,
            )
        };
        assert_ne!(body_id_raw, 0, "Jolt rejected the compound shape");
        body_id_raw
    }

    pub fn create_box(
        &mut self,
        half_extents: Vec3,
        spawn_position: Vec3,
        object_layer: u16,
        motion: JoltMotion,
        density_kg_per_m3: f32,
        gravity_factor: f32,
    ) -> u32 {
        let body_id_raw = unsafe {
            bjolt_create_box(
                self.world_ptr,
                half_extents.x,
                half_extents.y,
                half_extents.z,
                spawn_position.x,
                spawn_position.y,
                spawn_position.z,
                object_layer,
                motion as u8,
                density_kg_per_m3,
                gravity_factor,
            )
        };
        body_id_raw
    }

    /// Shrink-wrapped convex lump from a point soup. Convex, so any motion
    /// works: dynamics tumble it. Rejects empty input and cook failures.
    pub fn create_hull(
        &mut self,
        hull_points: &[Vec3],
        spawn_position: Vec3,
        object_layer: u16,
        motion: JoltMotion,
        density_kg_per_m3: f32,
        gravity_factor: f32,
    ) -> u32 {
        assert!(!hull_points.is_empty(), "hull needs at least one point");
        let flat_points: Vec<f32> = hull_points
            .iter()
            .flat_map(|hull_point| [hull_point.x, hull_point.y, hull_point.z])
            .collect();
        let body_id_raw = unsafe {
            bjolt_create_hull(
                self.world_ptr,
                flat_points.as_ptr(),
                flat_points.len() as u32 / 3,
                spawn_position.x,
                spawn_position.y,
                spawn_position.z,
                object_layer,
                motion as u8,
                density_kg_per_m3,
                gravity_factor,
            )
        };
        assert_ne!(body_id_raw, 0, "Jolt rejected the hull shape");
        body_id_raw
    }

    /// Exact-triangle static scenery. Static only: Jolt cannot simulate mesh
    /// shapes, so any other motion panics at bake. Rejects empty input and
    /// out-of-range indices alongside Jolt's own cook check.
    pub fn create_mesh(
        &mut self,
        mesh_vertices: &[Vec3],
        mesh_triangles: &[[u32; 3]],
        spawn_position: Vec3,
        object_layer: u16,
    ) -> u32 {
        assert!(!mesh_vertices.is_empty(), "mesh needs vertices");
        assert!(!mesh_triangles.is_empty(), "mesh needs triangles");
        assert!(
            mesh_triangles
                .iter()
                .flatten()
                .all(|vertex_index| (*vertex_index as usize) < mesh_vertices.len()),
            "mesh triangle index out of range"
        );
        let flat_vertices: Vec<f32> = mesh_vertices
            .iter()
            .flat_map(|mesh_vertex| [mesh_vertex.x, mesh_vertex.y, mesh_vertex.z])
            .collect();
        let flat_triangles: Vec<u32> = mesh_triangles.iter().flatten().copied().collect();
        let body_id_raw = unsafe {
            bjolt_create_mesh(
                self.world_ptr,
                flat_vertices.as_ptr(),
                flat_vertices.len() as u32 / 3,
                flat_triangles.as_ptr(),
                flat_triangles.len() as u32 / 3,
                spawn_position.x,
                spawn_position.y,
                spawn_position.z,
                object_layer,
            )
        };
        assert_ne!(body_id_raw, 0, "Jolt rejected the mesh shape");
        body_id_raw
    }

    /// Terrain grid: `field_width * field_width` heights, one ground level
    /// per cell, `field_cell` meters apart, centered on the spawn. Static
    /// only: Jolt cannot simulate heightfields. Rejects non-square counts
    /// and cook failures.
    pub fn create_heightfield(
        &mut self,
        field_heights: &[f32],
        field_width: u32,
        field_cell: f32,
        spawn_position: Vec3,
        object_layer: u16,
    ) -> u32 {
        assert!(
            field_heights.len() as u32 == field_width * field_width,
            "heightfield needs width^2 heights, got {} for width {}",
            field_heights.len(),
            field_width
        );
        let body_id_raw = unsafe {
            bjolt_create_heightfield(
                self.world_ptr,
                field_heights.as_ptr(),
                field_heights.len() as u32,
                field_width,
                field_cell,
                spawn_position.x,
                spawn_position.y,
                spawn_position.z,
                object_layer,
            )
        };
        assert_ne!(body_id_raw, 0, "Jolt rejected the heightfield shape");
        body_id_raw
    }

    /// Drives a kinematic body toward a target pose with velocity Jolt derives
    /// from the delta, so it shoves dynamics aside. `delta_time` is the tick
    /// delta; call once per tick from `FixedUpdate`, ordered
    /// `.before(step_physics_world)`.
    pub fn move_kinematic(
        &mut self,
        body_id_raw: u32,
        target_position: Vec3,
        target_rotation: Quat,
        delta_time: f32,
    ) {
        unsafe {
            bjolt_move_kinematic(
                self.world_ptr,
                body_id_raw,
                target_position.x,
                target_position.y,
                target_position.z,
                target_rotation.x,
                target_rotation.y,
                target_rotation.z,
                target_rotation.w,
                delta_time,
            )
        }
    }

    pub fn create_capsule(
        &mut self,
        capsule_half_height: f32,
        capsule_radius: f32,
        spawn_position: Vec3,
        object_layer: u16,
        density_kg_per_m3: f32,
        gravity_factor: f32,
    ) -> u32 {
        let body_id_raw = unsafe {
            bjolt_create_capsule(
                self.world_ptr,
                capsule_half_height,
                capsule_radius,
                spawn_position.x,
                spawn_position.y,
                spawn_position.z,
                object_layer,
                density_kg_per_m3,
                gravity_factor,
            )
        };
        body_id_raw
    }

    pub fn create_cylinder(
        &mut self,
        cylinder_half_height: f32,
        cylinder_radius: f32,
        spawn_position: Vec3,
        object_layer: u16,
        density_kg_per_m3: f32,
        gravity_factor: f32,
    ) -> u32 {
        let body_id_raw = unsafe {
            bjolt_create_cylinder(
                self.world_ptr,
                cylinder_half_height,
                cylinder_radius,
                spawn_position.x,
                spawn_position.y,
                spawn_position.z,
                object_layer,
                density_kg_per_m3,
                gravity_factor,
            )
        };
        body_id_raw
    }

    pub fn create_tapered_cylinder(
        &mut self,
        tapered_half_height: f32,
        top_radius: f32,
        bottom_radius: f32,
        spawn_position: Vec3,
        object_layer: u16,
        density_kg_per_m3: f32,
        gravity_factor: f32,
    ) -> u32 {
        let body_id_raw = unsafe {
            bjolt_create_tapered_cylinder(
                self.world_ptr,
                tapered_half_height,
                top_radius,
                bottom_radius,
                spawn_position.x,
                spawn_position.y,
                spawn_position.z,
                object_layer,
                density_kg_per_m3,
                gravity_factor,
            )
        };
        body_id_raw
    }

    pub fn create_tapered_capsule(
        &mut self,
        tapered_half_height: f32,
        top_radius: f32,
        bottom_radius: f32,
        spawn_position: Vec3,
        object_layer: u16,
        density_kg_per_m3: f32,
        gravity_factor: f32,
    ) -> u32 {
        let body_id_raw = unsafe {
            bjolt_create_tapered_capsule(
                self.world_ptr,
                tapered_half_height,
                top_radius,
                bottom_radius,
                spawn_position.x,
                spawn_position.y,
                spawn_position.z,
                object_layer,
                density_kg_per_m3,
                gravity_factor,
            )
        };
        body_id_raw
    }

    /// Pose plus linear and angular velocity in one FFI round-trip: the
    /// per-tick sync reads everything here so bodies cost one call, not two.
    pub fn body_full_motion(&self, body_id_raw: u32) -> BodyMotion {
        let mut body_position = [0.0f32; 3];
        let mut body_rotation = [0.0f32; 4];
        let mut body_velocity = [0.0f32; 3];
        let mut body_spin = [0.0f32; 3];
        unsafe {
            bjolt_body_transform(
                self.world_ptr,
                body_id_raw,
                body_position.as_mut_ptr(),
                body_rotation.as_mut_ptr(),
                body_velocity.as_mut_ptr(),
                body_spin.as_mut_ptr(),
            );
        }
        BodyMotion {
            body_position: Vec3::from_array(body_position),
            body_rotation: Quat::from_array(body_rotation),
            body_velocity: Vec3::from_array(body_velocity),
            body_spin: Vec3::from_array(body_spin),
        }
    }

    pub fn body_full_transform(&self, body_id_raw: u32) -> (Vec3, Quat) {
        let body_motion = self.body_full_motion(body_id_raw);
        (body_motion.body_position, body_motion.body_rotation)
    }

    /// Closest body a ray hits, if any. `ray_direction` sets both direction
    /// and reach: hits past `origin + direction` are not reported.
    pub fn cast_ray(&self, ray_origin: Vec3, ray_direction: Vec3) -> Option<RayHit> {
        crate::spatial_queries::cast_ray_all(self.world_ptr, ray_origin, ray_direction)
            .into_iter()
            .next()
    }

    /// Every body a ray passes through, nearest first. Empty on a miss.
    pub fn cast_ray_all(&self, ray_origin: Vec3, ray_direction: Vec3) -> Vec<RayHit> {
        crate::spatial_queries::cast_ray_all(self.world_ptr, ray_origin, ray_direction)
    }

    /// Every body containing a point (solid shapes count as filled).
    pub fn collide_point_all(&self, probe_point: Vec3) -> Vec<u32> {
        crate::spatial_queries::collide_point_all(self.world_ptr, probe_point)
    }

    /// Every body overlapping a probe volume centered at a point.
    pub fn overlap_shape_all(
        &self,
        probe: crate::spatial_queries::QueryProbe,
        probe_center: Vec3,
    ) -> Vec<crate::spatial_queries::OverlapHit> {
        crate::spatial_queries::overlap_shape_all(self.world_ptr, probe, probe_center)
    }

    /// Sweeps a probe volume along a direction; every body touched, nearest
    /// first. `cast_direction` sets direction and reach.
    pub fn cast_shape_all(
        &self,
        probe: crate::spatial_queries::QueryProbe,
        probe_center: Vec3,
        cast_direction: Vec3,
    ) -> Vec<RayHit> {
        crate::spatial_queries::cast_shape_all(self.world_ptr, probe, probe_center, cast_direction)
    }

    pub fn update(&mut self, delta_time: f32, collision_steps: i32) {
        unsafe { bjolt_world_update(self.world_ptr, delta_time, collision_steps) }
    }

    /// Welds two bodies in their current relative pose. Returns 0 on failure.
    pub fn create_fixed_constraint(
        &mut self,
        body1_raw: u32,
        body2_raw: u32,
        joint_space: JointSpace,
    ) -> u32 {
        unsafe {
            bjolt_create_fixed_constraint(
                self.world_ptr,
                body1_raw,
                body2_raw,
                joint_space.ffi_space(),
            )
        }
    }

    /// Keeps two anchor points within a distance band. Negative bounds fall
    /// back to the anchors' current distance. Returns 0 on failure.
    pub fn create_distance_constraint(
        &mut self,
        body1_raw: u32,
        body2_raw: u32,
        point1: Vec3,
        point2: Vec3,
        min_distance: f32,
        max_distance: f32,
        joint_space: JointSpace,
    ) -> u32 {
        unsafe {
            bjolt_create_distance_constraint(
                self.world_ptr,
                body1_raw,
                body2_raw,
                point1.x,
                point1.y,
                point1.z,
                point2.x,
                point2.y,
                point2.z,
                min_distance,
                max_distance,
                joint_space.ffi_space(),
            )
        }
    }

    /// Single-axis hinge at an anchor point. The hinge axis is the free
    /// rotation; the normal axis defines angle zero. Limits in radians clamp
    /// the swing: min in [-pi, 0], max in [0, pi] (full swing by default).
    /// The motor spring engages on the first drive call. Returns 0 on failure.
    pub fn create_hinge_constraint(
        &mut self,
        body1_raw: u32,
        body2_raw: u32,
        hinge_point: Vec3,
        hinge_axis1: Dir3,
        normal_axis1: Dir3,
        hinge_axis2: Dir3,
        normal_axis2: Dir3,
        limits_min: f32,
        limits_max: f32,
        motor: crate::joint_sync::JointMotor,
        joint_space: JointSpace,
    ) -> u32 {
        unsafe {
            bjolt_create_hinge_constraint(
                self.world_ptr,
                body1_raw,
                body2_raw,
                hinge_point.x,
                hinge_point.y,
                hinge_point.z,
                hinge_axis1.as_vec3().x,
                hinge_axis1.as_vec3().y,
                hinge_axis1.as_vec3().z,
                normal_axis1.as_vec3().x,
                normal_axis1.as_vec3().y,
                normal_axis1.as_vec3().z,
                hinge_axis2.as_vec3().x,
                hinge_axis2.as_vec3().y,
                hinge_axis2.as_vec3().z,
                normal_axis2.as_vec3().x,
                normal_axis2.as_vec3().y,
                normal_axis2.as_vec3().z,
                limits_min,
                limits_max,
                motor.frequency_hz,
                motor.damping,
                motor.force_limit,
                joint_space.ffi_space(),
            )
        }
    }

    pub fn remove_constraint(&mut self, constraint_id: u32) {
        unsafe { bjolt_remove_constraint(self.world_ptr, constraint_id) }
    }

    /// Ball-and-socket at an anchor point: positions locked, rotation free.
    /// Returns 0 on failure.
    pub fn create_point_constraint(
        &mut self,
        body1_raw: u32,
        body2_raw: u32,
        constraint_point: Vec3,
        joint_space: JointSpace,
    ) -> u32 {
        unsafe {
            bjolt_create_point_constraint(
                self.world_ptr,
                body1_raw,
                body2_raw,
                constraint_point.x,
                constraint_point.y,
                constraint_point.z,
                joint_space.ffi_space(),
            )
        }
    }

    /// Prismatic slide along an axis with travel limits, locked in the
    /// current relative pose otherwise. The motor spring engages on the first
    /// drive call. Returns 0 on failure.
    pub fn create_slider_constraint(
        &mut self,
        body1_raw: u32,
        body2_raw: u32,
        slider_axis1: Dir3,
        normal_axis1: Dir3,
        slider_axis2: Dir3,
        normal_axis2: Dir3,
        limits_min: f32,
        limits_max: f32,
        motor: crate::joint_sync::JointMotor,
        joint_space: JointSpace,
    ) -> u32 {
        unsafe {
            bjolt_create_slider_constraint(
                self.world_ptr,
                body1_raw,
                body2_raw,
                slider_axis1.as_vec3().x,
                slider_axis1.as_vec3().y,
                slider_axis1.as_vec3().z,
                normal_axis1.as_vec3().x,
                normal_axis1.as_vec3().y,
                normal_axis1.as_vec3().z,
                slider_axis2.as_vec3().x,
                slider_axis2.as_vec3().y,
                slider_axis2.as_vec3().z,
                normal_axis2.as_vec3().x,
                normal_axis2.as_vec3().y,
                normal_axis2.as_vec3().z,
                limits_min,
                limits_max,
                motor.frequency_hz,
                motor.damping,
                motor.force_limit,
                joint_space.ffi_space(),
            )
        }
    }

    /// Ball-and-socket with swing capped to a cone around the twist axis.
    /// Returns 0 on failure.
    pub fn create_cone_constraint(
        &mut self,
        body1_raw: u32,
        body2_raw: u32,
        constraint_point: Vec3,
        twist_axis1: Dir3,
        twist_axis2: Dir3,
        half_cone_angle: f32,
        joint_space: JointSpace,
    ) -> u32 {
        unsafe {
            bjolt_create_cone_constraint(
                self.world_ptr,
                body1_raw,
                body2_raw,
                constraint_point.x,
                constraint_point.y,
                constraint_point.z,
                twist_axis1.as_vec3().x,
                twist_axis1.as_vec3().y,
                twist_axis1.as_vec3().z,
                twist_axis2.as_vec3().x,
                twist_axis2.as_vec3().y,
                twist_axis2.as_vec3().z,
                half_cone_angle,
                joint_space.ffi_space(),
            )
        }
    }

    /// Shoulder-style joint: separate swing cone plus twist range.
    /// Per-body axes (like the cone): differing body2 vectors offset the
    /// rest pose inside the cone without moving its center. Returns 0 on failure.
    pub fn create_swing_twist_constraint(
        &mut self,
        body1_raw: u32,
        body2_raw: u32,
        constraint_position: Vec3,
        twist_axis1: Dir3,
        plane_axis1: Dir3,
        twist_axis2: Dir3,
        plane_axis2: Dir3,
        normal_half_cone_angle: f32,
        plane_half_cone_angle: f32,
        twist_min_angle: f32,
        twist_max_angle: f32,
        joint_space: JointSpace,
    ) -> u32 {
        unsafe {
            bjolt_create_swing_twist_constraint(
                self.world_ptr,
                body1_raw,
                body2_raw,
                constraint_position.x,
                constraint_position.y,
                constraint_position.z,
                twist_axis1.as_vec3().x,
                twist_axis1.as_vec3().y,
                twist_axis1.as_vec3().z,
                plane_axis1.as_vec3().x,
                plane_axis1.as_vec3().y,
                plane_axis1.as_vec3().z,
                twist_axis2.as_vec3().x,
                twist_axis2.as_vec3().y,
                twist_axis2.as_vec3().z,
                plane_axis2.as_vec3().x,
                plane_axis2.as_vec3().y,
                plane_axis2.as_vec3().z,
                normal_half_cone_angle,
                plane_half_cone_angle,
                twist_min_angle,
                twist_max_angle,
                joint_space.ffi_space(),
            )
        }
    }

    /// Fully general six-DOF joint: per-axis translation/rotation limits plus
    /// a velocity motor on any axis subset. Limit conventions per axis: min >
    /// max fixes it, +-large frees it. `motor_axes` bit `i` (0 = TX .. 5 = RZ)
    /// arms that axis with `motor`. Returns 0 on failure.
    #[allow(clippy::too_many_arguments)]
    pub fn create_six_dof(
        &mut self,
        body1_raw: u32,
        body2_raw: u32,
        frame: crate::joint_sync::SixDofFrame,
        limits: crate::joint_sync::SixDofLimits,
        motor_axes: u8,
        motor: crate::joint_sync::JointMotor,
        joint_space: JointSpace,
    ) -> u32 {
        unsafe {
            bjolt_create_six_dof(
                self.world_ptr,
                body1_raw,
                body2_raw,
                frame.position1.x,
                frame.position1.y,
                frame.position1.z,
                frame.axis_x1.as_vec3().x,
                frame.axis_x1.as_vec3().y,
                frame.axis_x1.as_vec3().z,
                frame.axis_y1.as_vec3().x,
                frame.axis_y1.as_vec3().y,
                frame.axis_y1.as_vec3().z,
                frame.position2.x,
                frame.position2.y,
                frame.position2.z,
                frame.axis_x2.as_vec3().x,
                frame.axis_x2.as_vec3().y,
                frame.axis_x2.as_vec3().z,
                frame.axis_y2.as_vec3().x,
                frame.axis_y2.as_vec3().y,
                frame.axis_y2.as_vec3().z,
                limits.translation_min.x,
                limits.translation_max.x,
                limits.translation_min.y,
                limits.translation_max.y,
                limits.translation_min.z,
                limits.translation_max.z,
                limits.rotation_min.x,
                limits.rotation_max.x,
                limits.rotation_min.y,
                limits.rotation_max.y,
                limits.rotation_min.z,
                limits.rotation_max.z,
                motor_axes,
                motor.frequency_hz,
                motor.damping,
                motor.force_limit,
                joint_space.ffi_space(),
            )
        }
    }

    /// Elevator counterweight: rope length of body 1 plus ratio-scaled rope
    /// length of body 2 stays inside the band. Returns 0 on failure.
    pub fn create_pulley_constraint(
        &mut self,
        body1_raw: u32,
        body2_raw: u32,
        body_point1: Vec3,
        fixed_point1: Vec3,
        body_point2: Vec3,
        fixed_point2: Vec3,
        ratio: f32,
        min_length: f32,
        max_length: f32,
        joint_space: JointSpace,
    ) -> u32 {
        unsafe {
            bjolt_create_pulley_constraint(
                self.world_ptr,
                body1_raw,
                body2_raw,
                body_point1.x,
                body_point1.y,
                body_point1.z,
                fixed_point1.x,
                fixed_point1.y,
                fixed_point1.z,
                body_point2.x,
                body_point2.y,
                body_point2.z,
                fixed_point2.x,
                fixed_point2.y,
                fixed_point2.z,
                ratio,
                min_length,
                max_length,
                joint_space.ffi_space(),
            )
        }
    }

    /// Couples two hinged rotations through a gear ratio. Both bodies must
    /// already be pinned by hinge constraints passed as ids. Returns 0 on failure.
    pub fn create_gear_constraint(
        &mut self,
        body1_raw: u32,
        body2_raw: u32,
        hinge_axis: Dir3,
        ratio: f32,
        hinge_id1: u32,
        hinge_id2: u32,
        joint_space: JointSpace,
    ) -> u32 {
        unsafe {
            bjolt_create_gear_constraint(
                self.world_ptr,
                body1_raw,
                body2_raw,
                hinge_axis.as_vec3().x,
                hinge_axis.as_vec3().y,
                hinge_axis.as_vec3().z,
                ratio,
                hinge_id1,
                hinge_id2,
                joint_space.ffi_space(),
            )
        }
    }

    /// Links a spinning pinion to a sliding rack. Pinion hinge and rack slider
    /// constraints pass in as ids. Returns 0 on failure.
    pub fn create_rack_pinion_constraint(
        &mut self,
        body1_raw: u32,
        body2_raw: u32,
        hinge_axis: Dir3,
        slider_axis: Dir3,
        ratio: f32,
        pinion_hinge_id: u32,
        rack_slider_id: u32,
        joint_space: JointSpace,
    ) -> u32 {
        unsafe {
            bjolt_create_rack_pinion_constraint(
                self.world_ptr,
                body1_raw,
                body2_raw,
                hinge_axis.as_vec3().x,
                hinge_axis.as_vec3().y,
                hinge_axis.as_vec3().z,
                slider_axis.as_vec3().x,
                slider_axis.as_vec3().y,
                slider_axis.as_vec3().z,
                ratio,
                pinion_hinge_id,
                rack_slider_id,
                joint_space.ffi_space(),
            )
        }
    }

    /// Cart on a Hermite spline track: knot list plus a looping flag.
    /// Needs a static anchor body plus the cart body. The motor spring
    /// engages on the first drive call. Returns 0 on failure.
    pub fn create_path_cart(
        &mut self,
        static_body_raw: u32,
        cart_body_raw: u32,
        track_knots: &[crate::joint_sync::PathKnot],
        looping: bool,
        motor: crate::joint_sync::JointMotor,
        cart_rotation: crate::joint_sync::PathRotation,
    ) -> u32 {
        use jolt_sys::{MAX_PATH_POINTS, PathPointFfi};
        let points: Vec<PathPointFfi> = track_knots
            .iter()
            .map(|knot| PathPointFfi {
                pos_x: knot.knot_position.x,
                pos_y: knot.knot_position.y,
                pos_z: knot.knot_position.z,
                tan_x: knot.knot_tangent.x,
                tan_y: knot.knot_tangent.y,
                tan_z: knot.knot_tangent.z,
                nrm_x: knot.knot_normal.x,
                nrm_y: knot.knot_normal.y,
                nrm_z: knot.knot_normal.z,
            })
            .collect();
        unsafe {
            bjolt_create_path_cart(
                self.world_ptr,
                static_body_raw,
                cart_body_raw,
                points.as_ptr(),
                points.len().min(MAX_PATH_POINTS) as u8,
                u8::from(looping),
                motor.frequency_hz,
                motor.damping,
                motor.force_limit,
                cart_rotation.ffi_mode(),
            )
        }
    }

    /// Fully specified vehicle: chassis box plus wheels, engine, gearbox,
    /// and controller tuning from a [`VehicleSpec`](crate::vehicle::VehicleSpec).
    /// Returns the chassis body + constraint ids for
    /// [`JoltVehicleId`](crate::vehicle::JoltVehicleId), or `None` on failure.
    pub fn create_vehicle(
        &mut self,
        spec: &crate::vehicle::VehicleSpec,
        spawn_position: Vec3,
    ) -> Option<crate::vehicle::JoltVehicleId> {
        use jolt_sys::{MAX_VEHICLE_DIFFS, MAX_VEHICLE_ROLL_BARS, MAX_VEHICLE_WHEELS};
        let mut body_raw = 0u32;
        let mut constraint_id = 0u32;
        let wheels: Vec<VehicleWheelFfi> = spec.wheels.iter().map(|wheel| wheel.to_ffi()).collect();
        let engine: VehicleEngineFfi = spec.engine.to_ffi();
        let gearbox: VehicleTransmissionFfi = spec.transmission.to_ffi();
        let created = match &spec.kind {
            crate::vehicle::VehicleKind::Wheeled {
                differentials,
                roll_bars,
                limited_slip_ratio,
            } => {
                let diffs: Vec<VehicleDifferentialFfi> =
                    differentials.iter().map(|diff| diff.to_ffi()).collect();
                let bars: Vec<VehicleRollBarFfi> =
                    roll_bars.iter().map(|bar| bar.to_ffi()).collect();
                unsafe {
                    bjolt_create_wheeled_vehicle(
                        self.world_ptr,
                        spec.object_layer,
                        spawn_position.x,
                        spawn_position.y,
                        spawn_position.z,
                        spec.half_extents.x,
                        spec.half_extents.y,
                        spec.half_extents.z,
                        spec.center_of_mass_offset.x,
                        spec.center_of_mass_offset.y,
                        spec.center_of_mass_offset.z,
                        spec.mass_kg,
                        spec.max_pitch_roll_angle,
                        spec.tester_radius,
                        wheels.as_ptr(),
                        wheels.len().min(MAX_VEHICLE_WHEELS) as u8,
                        &engine,
                        &gearbox,
                        diffs.as_ptr(),
                        diffs.len().min(MAX_VEHICLE_DIFFS) as u8,
                        bars.as_ptr(),
                        bars.len().min(MAX_VEHICLE_ROLL_BARS) as u8,
                        *limited_slip_ratio,
                        &mut body_raw,
                        &mut constraint_id,
                    )
                }
            }
            crate::vehicle::VehicleKind::Tracked { tracks } => {
                let track_ffis = [tracks[0].to_ffi(), tracks[1].to_ffi()];
                unsafe {
                    bjolt_create_tracked_vehicle(
                        self.world_ptr,
                        spec.object_layer,
                        spawn_position.x,
                        spawn_position.y,
                        spawn_position.z,
                        spec.half_extents.x,
                        spec.half_extents.y,
                        spec.half_extents.z,
                        spec.center_of_mass_offset.x,
                        spec.center_of_mass_offset.y,
                        spec.center_of_mass_offset.z,
                        spec.mass_kg,
                        spec.max_pitch_roll_angle,
                        spec.tester_radius,
                        wheels.as_ptr(),
                        wheels.len().min(MAX_VEHICLE_WHEELS) as u8,
                        &engine,
                        &gearbox,
                        track_ffis.as_ptr(),
                        &mut body_raw,
                        &mut constraint_id,
                    )
                }
            }
            crate::vehicle::VehicleKind::Motorcycle {
                differentials,
                lean,
            } => {
                let diffs: Vec<VehicleDifferentialFfi> =
                    differentials.iter().map(|diff| diff.to_ffi()).collect();
                let lean_ffi: VehicleLeanFfi = lean.to_ffi();
                unsafe {
                    bjolt_create_motorcycle(
                        self.world_ptr,
                        spec.object_layer,
                        spawn_position.x,
                        spawn_position.y,
                        spawn_position.z,
                        spec.half_extents.x,
                        spec.half_extents.y,
                        spec.half_extents.z,
                        spec.center_of_mass_offset.x,
                        spec.center_of_mass_offset.y,
                        spec.center_of_mass_offset.z,
                        spec.mass_kg,
                        spec.max_pitch_roll_angle,
                        spec.tester_radius,
                        wheels.as_ptr(),
                        wheels.len().min(MAX_VEHICLE_WHEELS) as u8,
                        &engine,
                        &gearbox,
                        diffs.as_ptr(),
                        diffs.len().min(MAX_VEHICLE_DIFFS) as u8,
                        &lean_ffi,
                        &mut body_raw,
                        &mut constraint_id,
                    )
                }
            }
        };
        (created != 0).then_some(crate::vehicle::JoltVehicleId {
            body_id_raw: body_raw,
            constraint_id_raw: constraint_id,
        })
    }

    /// Wheeled/motorcycle input: gas, steer, foot brake, hand brake.
    /// No-op on bad ids.
    pub fn vehicle_drive(
        &mut self,
        constraint_id: u32,
        forward: f32,
        right: f32,
        brake: f32,
        hand_brake: f32,
    ) {
        unsafe {
            bjolt_vehicle_drive(
                self.world_ptr,
                constraint_id,
                forward,
                right,
                brake,
                hand_brake,
            )
        }
    }

    /// Tank input: gas plus per-track multipliers. No-op on bad ids.
    pub fn tracked_drive(
        &mut self,
        constraint_id: u32,
        forward: f32,
        left_ratio: f32,
        right_ratio: f32,
        brake: f32,
    ) {
        unsafe {
            bjolt_tracked_drive(
                self.world_ptr,
                constraint_id,
                forward,
                left_ratio,
                right_ratio,
                brake,
            )
        }
    }

    /// Manual gear: -1 reverse, 0 neutral, 1+ forward, plus clutch 0..1.
    /// Auto boxes ignore it. No-op on bad ids.
    pub fn vehicle_shift(&mut self, constraint_id: u32, gear: i32, clutch_friction: f32) {
        unsafe { bjolt_vehicle_shift(self.world_ptr, constraint_id, gear, clutch_friction) }
    }

    /// One-shot linear + angular impulse at center of mass. Zero halves are
    /// skipped; pass one pair for a combined kick.
    pub fn apply_impulse(&mut self, body_id_raw: u32, linear_impulse: Vec3, angular_impulse: Vec3) {
        unsafe {
            bjolt_apply_impulse(
                self.world_ptr,
                body_id_raw,
                linear_impulse.x,
                linear_impulse.y,
                linear_impulse.z,
                angular_impulse.x,
                angular_impulse.y,
                angular_impulse.z,
            )
        }
    }

    /// Persistent force + torque. Jolt clears accumulated forces each step,
    /// so call once per tick while the push lasts. Zero halves skipped.
    pub fn apply_force(&mut self, body_id_raw: u32, push_force: Vec3, push_torque: Vec3) {
        unsafe {
            bjolt_apply_force(
                self.world_ptr,
                body_id_raw,
                push_force.x,
                push_force.y,
                push_force.z,
                push_torque.x,
                push_torque.y,
                push_torque.z,
            )
        }
    }

    /// Direct velocity overwrite (not a kick): zero halves stop that axis.
    pub fn set_body_velocity(
        &mut self,
        body_id_raw: u32,
        linear_velocity: Vec3,
        angular_velocity: Vec3,
    ) {
        unsafe {
            bjolt_set_velocity(
                self.world_ptr,
                body_id_raw,
                linear_velocity.x,
                linear_velocity.y,
                linear_velocity.z,
                angular_velocity.x,
                angular_velocity.y,
                angular_velocity.z,
            )
        }
    }
    /// Linear-only velocity overwrite. Leaves angular velocity untouched, so
    /// a driven move never wipes out spin.
    pub fn set_linear_velocity(&mut self, body_id_raw: u32, linear_velocity: Vec3) {
        unsafe {
            bjolt_set_linear_velocity(
                self.world_ptr,
                body_id_raw,
                linear_velocity.x,
                linear_velocity.y,
                linear_velocity.z,
            )
        }
    }

    /// Angular-only velocity overwrite. Leaves linear velocity untouched.
    pub fn set_angular_velocity(&mut self, body_id_raw: u32, angular_velocity: Vec3) {
        unsafe {
            bjolt_set_angular_velocity(
                self.world_ptr,
                body_id_raw,
                angular_velocity.x,
                angular_velocity.y,
                angular_velocity.z,
            )
        }
    }

    /// Rotates a live body to a pose (spawn rotation fix-up, teleports).
    /// Activates the body so the pose takes effect on the next step.
    pub fn set_body_rotation(&mut self, body_id_raw: u32, body_rotation: Quat) {
        unsafe {
            bjolt_set_rotation(
                self.world_ptr,
                body_id_raw,
                body_rotation.x,
                body_rotation.y,
                body_rotation.z,
                body_rotation.w,
            )
        }
    }

    /// Surface grip on a live body (0 ice, 1+ rubber). Applied at bake and
    /// changeable at runtime.
    pub fn set_body_friction(&mut self, body_id_raw: u32, friction: f32) {
        unsafe { bjolt_set_friction(self.world_ptr, body_id_raw, friction) }
    }

    /// Bounciness on a live body (0 dead, 1 superball). Same timing as friction.
    pub fn set_body_restitution(&mut self, body_id_raw: u32, restitution: f32) {
        unsafe { bjolt_set_restitution(self.world_ptr, body_id_raw, restitution) }
    }

    /// Continuous collision on a live body: sweeps the shape so fast bodies
    /// stop at the first hit. Applied at bake for flagged bodies.
    pub fn set_body_ccd(&mut self, body_id_raw: u32, use_ccd: bool) {
        unsafe { bjolt_set_ccd(self.world_ptr, body_id_raw, use_ccd) }
    }

    /// Buoyancy push for one tick: Jolt lifts the submerged part toward
    /// the surface, with water drag. Called every Fixed tick per floater.
    #[allow(clippy::too_many_arguments)]
    pub fn apply_buoyancy(
        &mut self,
        body_id_raw: u32,
        surface_height: f32,
        buoyancy: f32,
        linear_drag: f32,
        angular_drag: f32,
        current_velocity: Vec3,
        world_gravity: Vec3,
        tick_delta: f32,
    ) {
        unsafe {
            bjolt_apply_buoyancy(
                self.world_ptr,
                body_id_raw,
                surface_height,
                buoyancy,
                linear_drag,
                angular_drag,
                current_velocity.x,
                current_velocity.y,
                current_velocity.z,
                world_gravity.x,
                world_gravity.y,
                world_gravity.z,
                tick_delta,
            )
        }
    }

    /// Shared soft-body settings from plain shape data. See
    /// [`JoltWorld::create_soft_shared`] on `JoltSoftSharedSettings` usage;
    /// this is the world-side constructor the bake path calls.
    /// Creates shared soft-body settings from plain shape data: vertex
    /// positions/velocities/inverse masses, triangle triples, edge pairs +
    /// compliances, volume quads + compliances. Returns an opaque handle
    /// (0 on failure) that outlives bodies: destroy it explicitly once no
    /// body uses it. Empty slices skip that group; with no hand-placed
    /// edges, Jolt auto-builds edge/shear/bend constraints from the faces.
    #[allow(clippy::too_many_arguments)]
    pub fn create_soft_shared(
        &mut self,
        vertex_positions: &[Vec3],
        vertex_velocities: &[Vec3],
        vertex_inv_masses: &[f32],
        face_triangles: &[[u32; 3]],
        edge_pairs: &[[u32; 2]],
        edge_compliances: &[f32],
        volume_quads: &[[u32; 4]],
        volume_compliances: &[f32],
        edge_compliance: f32,
        shear_compliance: f32,
        bend_compliance: f32,
        bend_type: SoftBendType,
    ) -> u64 {
        let flat_positions: Vec<f32> = vertex_positions
            .iter()
            .flat_map(|vertex_position| vertex_position.to_array())
            .collect();
        let flat_velocities: Vec<f32> = vertex_velocities
            .iter()
            .flat_map(|vertex_velocity| vertex_velocity.to_array())
            .collect();
        let flat_faces: Vec<u32> = face_triangles.iter().flatten().copied().collect();
        let flat_edges: Vec<u32> = edge_pairs.iter().flatten().copied().collect();
        let flat_volumes: Vec<u32> = volume_quads.iter().flatten().copied().collect();
        unsafe {
            bjolt_create_shared_settings(
                flat_positions.as_ptr(),
                if vertex_velocities.is_empty() {
                    std::ptr::null()
                } else {
                    flat_velocities.as_ptr()
                },
                vertex_inv_masses.as_ptr(),
                vertex_positions.len() as u32,
                if face_triangles.is_empty() {
                    std::ptr::null()
                } else {
                    flat_faces.as_ptr()
                },
                face_triangles.len() as u32,
                if edge_pairs.is_empty() {
                    std::ptr::null()
                } else {
                    flat_edges.as_ptr()
                },
                if edge_compliances.is_empty() {
                    std::ptr::null()
                } else {
                    edge_compliances.as_ptr()
                },
                edge_pairs.len() as u32,
                if volume_quads.is_empty() {
                    std::ptr::null()
                } else {
                    flat_volumes.as_ptr()
                },
                if volume_compliances.is_empty() {
                    std::ptr::null()
                } else {
                    volume_compliances.as_ptr()
                },
                volume_quads.len() as u32,
                edge_compliance,
                shear_compliance,
                bend_compliance,
                bend_type as u8,
            )
        }
    }

    /// Frees shared settings. Call once no live body uses the handle.
    pub fn destroy_soft_shared(&mut self, shared_handle: u64) {
        unsafe { bjolt_destroy_shared_settings(shared_handle) }
    }

    /// Solid cube shared settings with volume constraints (Jolt's own
    /// `sCreateCube`): the real 3D soft body. 0 on bad sizes.
    pub fn create_cube_shared(&mut self, grid_size: u32, grid_spacing: f32) -> u64 {
        unsafe { bjolt_create_cube_settings(grid_size, grid_spacing) }
    }

    /// Cloth-grid shared settings with the top rows pinned. 0 on bad sizes.
    pub fn create_cloth_shared(
        &mut self,
        grid_nx: u32,
        grid_nz: u32,
        grid_spacing: f32,
        pinned_rows: u32,
        bend_type: SoftBendType,
    ) -> u64 {
        unsafe {
            bjolt_create_cloth_settings(
                grid_nx,
                grid_nz,
                grid_spacing,
                pinned_rows,
                bend_type as u8,
            )
        }
    }

    /// Hollow sphere shared settings (pressure inflates it). 0 on bad sizes.
    pub fn create_sphere_shared(
        &mut self,
        sphere_radius: f32,
        theta_segments: u32,
        phi_segments: u32,
        bend_type: SoftBendType,
    ) -> u64 {
        unsafe {
            bjolt_create_sphere_settings(
                sphere_radius,
                theta_segments,
                phi_segments,
                bend_type as u8,
            )
        }
    }

    /// Vertex count baked into shared settings. 0 on bad handles.
    pub fn soft_shared_vertex_count(&self, shared_handle: u64) -> u32 {
        unsafe { bjolt_shared_vertex_count(shared_handle) }
    }

    /// Face count baked into shared settings. 0 on bad handles.
    pub fn soft_shared_face_count(&self, shared_handle: u64) -> u32 {
        unsafe { bjolt_shared_face_count(shared_handle) }
    }

    /// Face triangles baked into shared settings. Truncates at the buffer
    /// length; returns triangles written.
    pub fn soft_shared_faces(&self, shared_handle: u64, out_triangles: &mut [[u32; 3]]) -> u32 {
        unsafe {
            bjolt_shared_faces(
                shared_handle,
                out_triangles.as_mut_ptr() as *mut u32,
                out_triangles.len() as u32,
            )
        }
    }

    /// Creates a soft body from shared settings with the full Jolt
    /// per-body knob set. Returns the body id (0 on failure).
    pub fn create_soft_body(&mut self, shared_handle: u64, soft_config: &SoftBodyConfig) -> u32 {
        unsafe {
            bjolt_create_soft_body(
                self.world_ptr,
                shared_handle,
                soft_config.body_position.x,
                soft_config.body_position.y,
                soft_config.body_position.z,
                soft_config.body_rotation.x,
                soft_config.body_rotation.y,
                soft_config.body_rotation.z,
                soft_config.body_rotation.w,
                soft_config.object_layer,
                soft_config.num_iterations,
                soft_config.linear_damping,
                soft_config.max_linear_velocity,
                soft_config.restitution,
                soft_config.friction,
                soft_config.pressure,
                soft_config.gravity_factor,
                soft_config.vertex_radius,
                soft_config.update_position,
                soft_config.make_rotation_identity,
                soft_config.allow_sleeping,
                soft_config.faces_double_sided,
                soft_config.user_data,
            )
        }
    }

    /// Vertex count for mesh sizing. 0 on bad ids or non-soft bodies.
    pub fn soft_vertex_count(&self, body_id_raw: u32) -> u32 {
        unsafe { bjolt_soft_vertex_count(self.world_ptr, body_id_raw) }
    }

    /// World-space vertex positions for mesh rebuild. Truncates at the
    /// buffer length; returns vertices written.
    pub fn soft_vertices(&self, body_id_raw: u32, out_positions: &mut [Vec3]) -> u32 {
        unsafe {
            bjolt_soft_vertices(
                self.world_ptr,
                body_id_raw,
                out_positions.as_mut_ptr() as *mut f32,
                out_positions.len() as u32,
            )
        }
    }

    /// World-space vertex velocities. Truncates at the buffer length.
    pub fn soft_velocities(&self, body_id_raw: u32, out_velocities: &mut [Vec3]) -> u32 {
        unsafe {
            bjolt_soft_velocities(
                self.world_ptr,
                body_id_raw,
                out_velocities.as_mut_ptr() as *mut f32,
                out_velocities.len() as u32,
            )
        }
    }

    /// Per-vertex inverse masses. Zero means pinned.
    pub fn soft_inv_masses(&self, body_id_raw: u32, out_inv_masses: &mut [f32]) -> u32 {
        unsafe {
            bjolt_soft_inv_masses(
                self.world_ptr,
                body_id_raw,
                out_inv_masses.as_mut_ptr(),
                out_inv_masses.len() as u32,
            )
        }
    }

    /// Overwrites per-vertex inverse masses live: zero nails a vertex,
    /// positive frees it. Only mass and velocity are safe to touch at
    /// runtime per Jolt; positions stay solver-owned.
    pub fn soft_set_inv_masses(&mut self, body_id_raw: u32, inv_masses: &[f32]) {
        unsafe {
            bjolt_soft_set_inv_masses(
                self.world_ptr,
                body_id_raw,
                inv_masses.as_ptr(),
                inv_masses.len() as u32,
            )
        }
    }

    /// Per-vertex contact flags (true = touched something last update).
    pub fn soft_contacts(&self, body_id_raw: u32, out_contacted: &mut [bool]) -> u32 {
        let mut contact_bytes = vec![0u8; out_contacted.len()];
        let written = unsafe {
            bjolt_soft_contacts(
                self.world_ptr,
                body_id_raw,
                contact_bytes.as_mut_ptr(),
                contact_bytes.len() as u32,
            )
        };
        for (contact_flag, contact_byte) in out_contacted.iter_mut().zip(contact_bytes.iter()) {
            *contact_flag = *contact_byte != 0;
        }
        written
    }

    /// Current pressure (balloon bodies). 0 on bad ids.
    pub fn soft_pressure(&self, body_id_raw: u32) -> f32 {
        unsafe { bjolt_soft_pressure(self.world_ptr, body_id_raw) }
    }

    /// Sets pressure live (inflate/deflate a balloon body).
    pub fn soft_set_pressure(&mut self, body_id_raw: u32, pressure: f32) {
        unsafe { bjolt_soft_set_pressure(self.world_ptr, body_id_raw, pressure) }
    }

    /// Current solver iterations. 0 on bad ids.
    pub fn soft_iterations(&self, body_id_raw: u32) -> u32 {
        unsafe { bjolt_soft_iterations(self.world_ptr, body_id_raw) }
    }

    /// Sets solver iterations live.
    pub fn soft_set_iterations(&mut self, body_id_raw: u32, num_iterations: u32) {
        unsafe { bjolt_soft_set_iterations(self.world_ptr, body_id_raw, num_iterations) }
    }

    /// Current vertex radius (skin thickness). 0 on bad ids.
    pub fn soft_vertex_radius(&self, body_id_raw: u32) -> f32 {
        unsafe { bjolt_soft_vertex_radius(self.world_ptr, body_id_raw) }
    }

    /// Sets vertex radius live.
    pub fn soft_set_vertex_radius(&mut self, body_id_raw: u32, vertex_radius: f32) {
        unsafe { bjolt_soft_set_vertex_radius(self.world_ptr, body_id_raw, vertex_radius) }
    }

    /// Current enclosed volume (closed shapes). 0 on bad ids.
    pub fn soft_volume(&self, body_id_raw: u32) -> f32 {
        unsafe { bjolt_soft_volume(self.world_ptr, body_id_raw) }
    }

    /// Destroys a soft body. Never touches rigid bodies.
    pub fn soft_destroy(&mut self, body_id_raw: u32) {
        unsafe { bjolt_soft_destroy(self.world_ptr, body_id_raw) }
    }

    /// Wind for one tick: uniform breeze over the sheet, pins hold the top
    /// so folds ripple. Call every Fixed tick before the step per cloth.
    pub fn soft_push(&mut self, body_id_raw: u32, push_force: Vec3) {
        unsafe {
            bjolt_soft_push(
                self.world_ptr,
                body_id_raw,
                push_force.x,
                push_force.y,
                push_force.z,
            )
        }
    }

    /// Moves a live body (spawn fix-up, respawns, resets). Wakes the body;
    /// the sync carries the pose out to Bevy on the next tick.
    pub fn move_body(&mut self, body_id_raw: u32, body_position: Vec3) {
        unsafe {
            bjolt_set_position(
                self.world_ptr,
                body_id_raw,
                body_position.x,
                body_position.y,
                body_position.z,
            )
        }
    }

    /// Full pose teleport: position + rotation atomically, so the body
    /// never observes a half-moved frame. Keeps prior momentum: follow
    /// with `set_body_velocity` to zero for a dead stop.
    pub fn teleport_body(&mut self, body_id_raw: u32, body_position: Vec3, body_rotation: Quat) {
        unsafe {
            bjolt_set_position_rotation(
                self.world_ptr,
                body_id_raw,
                body_position.x,
                body_position.y,
                body_position.z,
                body_rotation.x,
                body_rotation.y,
                body_rotation.z,
                body_rotation.w,
            )
        }
    }

    /// Velocity motor on a slider, hinge, or path joint. False on bad ids.
    pub fn constraint_drive_at(&mut self, constraint_id: u32, target_velocity: f32) -> bool {
        unsafe { bjolt_constraint_drive_at(self.world_ptr, constraint_id, target_velocity) }
    }

    /// Velocity motor on a swing-twist joint: `axis` 0 = twist (spin about
    /// the constraint X axis), 1 = swing (sweep about constraint Y/Z).
    /// False on bad ids or non-swing-twist joints.
    pub fn constraint_drive_swing_twist(
        &mut self,
        constraint_id: u32,
        axis: u8,
        target_velocity: f32,
    ) -> bool {
        unsafe {
            bjolt_constraint_drive_swing_twist(self.world_ptr, constraint_id, axis, target_velocity)
        }
    }

    /// Current path fraction of a path constraint. NaN on bad ids or
    /// non-path joints. Debug probe for loop-seam diagnosis.
    pub fn constraint_path_fraction(&mut self, constraint_id: u32) -> f32 {
        unsafe { bjolt_constraint_path_fraction(self.world_ptr, constraint_id) }
    }

    /// Whether a path constraint's spline loops. -1 on bad ids.
    pub fn constraint_path_looping(&mut self, constraint_id: u32) -> i32 {
        unsafe { bjolt_constraint_path_looping(self.world_ptr, constraint_id) }
    }
    pub fn body_is_active(&self, body_id_raw: u32) -> bool {
        unsafe { bjolt_body_is_active(self.world_ptr, body_id_raw) }
    }

    /// Freezes a body where it stands: still solid, still in the broadphase,
    /// wakes on contact. For dormant crowds, not forever-settled props (set
    /// `JoltBody.motion` to static for those).
    pub fn sleep_body(&mut self, body_id_raw: u32) {
        unsafe { bjolt_sleep_body(self.world_ptr, body_id_raw) }
    }

    /// Rejoins a sleeping body next step with velocities intact. Firing at
    /// an already-awake body is a harmless no-op.
    pub fn wake_body(&mut self, body_id_raw: u32) {
        unsafe { bjolt_wake_body(self.world_ptr, body_id_raw) }
    }

    /// Live motion-type flip: static is solid and unwakeable, kinematic and
    /// dynamic rejoin awake with velocities intact. Reads `JoltBody.motion`,
    /// which is the source of truth — the sync pushes `Changed` values here
    /// before the step.
    pub fn set_body_motion(&mut self, body_id_raw: u32, motion: JoltMotion) {
        let motion_code = match motion {
            JoltMotion::Static => 0,
            JoltMotion::Kinematic => 1,
            JoltMotion::Dynamic => 2,
        };
        unsafe { bjolt_set_motion_type(self.world_ptr, body_id_raw, motion_code) }
    }

    /// Live gravity multiplier for one body (1 = normal). Reads the motion
    /// properties, so it reflects both the spawn value and later sets.
    pub fn body_gravity_factor(&self, body_id_raw: u32) -> f32 {
        unsafe { bjolt_gravity_factor(self.world_ptr, body_id_raw) }
    }

    /// Linear + angular damping on a live body (0 = Jolt default glide).
    /// Takes effect on the next step; ragdolls use it to settle instead of
    /// crawling on joint micro-motion.
    pub fn set_body_damping(
        &mut self,
        body_id_raw: u32,
        linear_damping: f32,
        angular_damping: f32,
    ) {
        unsafe {
            bjolt_body_set_damping(self.world_ptr, body_id_raw, linear_damping, angular_damping)
        }
    }

    /// Live density in kg/m³: rescales mass + inertia to density × shape
    /// volume. Takes effect on the next step, no re-bake needed.
    pub fn set_body_density(&mut self, body_id_raw: u32, density_kg_per_m3: f32) {
        unsafe { bjolt_body_set_density(self.world_ptr, body_id_raw, density_kg_per_m3) }
    }

    /// Changes the gravity multiplier on a live body (0 floats, 2 pulls
    /// double). Takes effect on the next step, no re-bake needed.
    pub fn set_body_gravity_factor(&mut self, body_id_raw: u32, gravity_factor: f32) {
        unsafe { bjolt_set_gravity_factor(self.world_ptr, body_id_raw, gravity_factor) }
    }

    /// Global gravity every body feels, scaled per body by its gravity
    /// factor. Jolt defaults to (0, -9.81, 0).
    pub fn world_gravity(&self) -> Vec3 {
        let mut gravity = [0.0f32; 3];
        unsafe { bjolt_world_gravity(self.world_ptr, gravity.as_mut_ptr()) }
        Vec3::from_array(gravity)
    }

    /// Replaces global gravity at runtime (moon level, zero-g room, flipped
    /// world). Takes effect on the next step.
    pub fn set_world_gravity(&mut self, world_gravity: Vec3) {
        unsafe {
            bjolt_set_gravity(
                self.world_ptr,
                world_gravity.x,
                world_gravity.y,
                world_gravity.z,
            )
        }
    }

    pub fn body_snapshot(&self, body_id_raw: u32) -> BodySnapshot {
        let mut body_position = [0.0f32; 3];
        let mut body_velocity = [0.0f32; 3];
        let mut body_spin = [0.0f32; 3];
        unsafe {
            bjolt_body_state(
                self.world_ptr,
                body_id_raw,
                body_position.as_mut_ptr(),
                body_velocity.as_mut_ptr(),
                body_spin.as_mut_ptr(),
            );
        }
        BodySnapshot {
            body_position: Vec3::from_array(body_position),
            body_velocity: Vec3::from_array(body_velocity),
            body_spin: Vec3::from_array(body_spin),
        }
    }

    pub fn remove_and_destroy_body(&mut self, body_id_raw: u32) {
        unsafe { bjolt_body_remove_destroy(self.world_ptr, body_id_raw) }
        self.body_shapes.remove(&body_id_raw);
        self.sensor_bodies.remove(&body_id_raw);
    }

    /// Flags a body as a sensor (overlaps report, nothing pushes). Tracks
    /// the flag in `sensor_bodies` so `body_collides` can read it back: no
    /// FFI getter exists, so the world remembers what it set.
    pub fn set_body_sensor(&mut self, body_id_raw: u32, is_sensor: bool) {
        unsafe { bjolt_body_set_sensor(self.world_ptr, body_id_raw, is_sensor) }
        if is_sensor {
            self.sensor_bodies.insert(body_id_raw);
        } else {
            self.sensor_bodies.remove(&body_id_raw);
        }
    }

    /// Whether a body currently collides (false while sensor-flagged).
    /// Reads the tracked `set_body_sensor` state. Game code reacts to
    /// `JoltDisabled` presence instead of polling this per frame; tests
    /// use it as the disable-state readback.
    pub fn body_collides(&self, body_id_raw: u32) -> bool {
        !self.sensor_bodies.contains(&body_id_raw)
    }

    /// Drains contact begin pairs into the buffers. Returns pairs kept.
    pub fn drain_contact_added(&mut self, pair_a: &mut [u32], pair_b: &mut [u32]) -> u32 {
        assert_eq!(pair_a.len(), pair_b.len(), "contact buffers must match");
        unsafe {
            bjolt_drain_contact_added(
                self.world_ptr,
                pair_a.as_mut_ptr(),
                pair_b.as_mut_ptr(),
                pair_a.len() as u32,
            )
        }
    }

    /// Drains contact end pairs into the buffers. Returns pairs kept.
    pub fn drain_contact_removed(&mut self, pair_a: &mut [u32], pair_b: &mut [u32]) -> u32 {
        assert_eq!(pair_a.len(), pair_b.len(), "contact buffers must match");
        unsafe {
            bjolt_drain_contact_removed(
                self.world_ptr,
                pair_a.as_mut_ptr(),
                pair_b.as_mut_ptr(),
                pair_a.len() as u32,
            )
        }
    }

    /// Max sleep/wake transitions drained per step. Activations are rare;
    /// 64 never fills in practice, overflow drops the newest.
    pub const MAX_ACTIVATION_EVENTS: usize = 64;

    /// Drains slept body ids into the buffer. Returns ids kept.
    pub fn drain_slept(&mut self, slept_ids: &mut [u32]) -> u32 {
        unsafe {
            bjolt_drain_slept(
                self.world_ptr,
                slept_ids.as_mut_ptr(),
                slept_ids.len() as u32,
            )
        }
    }

    /// Drains woken body ids into the buffer. Returns ids kept.
    pub fn drain_woke(&mut self, woke_ids: &mut [u32]) -> u32 {
        unsafe { bjolt_drain_woke(self.world_ptr, woke_ids.as_mut_ptr(), woke_ids.len() as u32) }
    }

    /// Stops two bodies colliding (ragdoll parent-child pairs). Shared group
    /// table per pair; game code calls this once per link at bake.
    pub fn set_bodies_no_collide(&mut self, body_a_raw: u32, body_b_raw: u32) {
        unsafe { bjolt_bodies_no_collide(self.world_ptr, body_a_raw, body_b_raw) }
    }

    /// Ragdoll builder holding Jolt settings + skeleton. Free with
    /// [`JoltWorld::ragdoll_build_destroy`] after create.
    pub fn ragdoll_build_create(&mut self) -> *mut jolt_sys::BJoltRagdollBuild {
        unsafe { bjolt_ragdoll_build_create() }
    }

    /// Adds one part to the builder. `parent_index` -1 = root; parents must
    /// come first. `shape_kind` 0 = capsule (`dim_x` cylinder half height,
    /// `dim_y` radius), 1 = box (half extents), 2 = sphere (`dim_x`
    /// radius). Returns the part index, or -1 on a bad shape.
    #[allow(clippy::too_many_arguments)]
    pub fn ragdoll_build_add_part(
        &mut self,
        build: *mut jolt_sys::BJoltRagdollBuild,
        parent_index: i32,
        shape_kind: u8,
        dim_x: f32,
        dim_y: f32,
        dim_z: f32,
        part_position: Vec3,
        part_rotation: Quat,
        object_layer: u16,
        density_kg_per_m3: f32,
        motion: JoltMotion,
    ) -> i32 {
        let motion_code = match motion {
            JoltMotion::Static => 0,
            JoltMotion::Kinematic => 1,
            JoltMotion::Dynamic => 2,
        };
        unsafe {
            bjolt_ragdoll_build_add_part(
                build,
                parent_index,
                shape_kind,
                dim_x,
                dim_y,
                dim_z,
                part_position.x,
                part_position.y,
                part_position.z,
                part_rotation.x,
                part_rotation.y,
                part_rotation.z,
                part_rotation.w,
                object_layer,
                density_kg_per_m3,
                motion_code,
            )
        }
    }

    /// Hinge limit between a part and its parent, about `anchor` (world
    /// space). Seated pose reads zero.
    pub fn ragdoll_build_set_hinge(
        &mut self,
        build: *mut jolt_sys::BJoltRagdollBuild,
        part_index: i32,
        anchor: Vec3,
        hinge_axis1: Vec3,
        normal_axis1: Vec3,
        hinge_axis2: Vec3,
        normal_axis2: Vec3,
        limits_min: f32,
        limits_max: f32,
    ) -> bool {
        unsafe {
            bjolt_ragdoll_build_set_hinge(
                build,
                part_index,
                anchor.x,
                anchor.y,
                anchor.z,
                hinge_axis1.x,
                hinge_axis1.y,
                hinge_axis1.z,
                normal_axis1.x,
                normal_axis1.y,
                normal_axis1.z,
                hinge_axis2.x,
                hinge_axis2.y,
                hinge_axis2.z,
                normal_axis2.x,
                normal_axis2.y,
                normal_axis2.z,
                limits_min,
                limits_max,
            )
        }
    }

    /// Swing-twist limit between a part and its parent, about `anchor`
    /// (world space). Per-side frames.
    #[allow(clippy::too_many_arguments)]
    pub fn ragdoll_build_set_swing_twist(
        &mut self,
        build: *mut jolt_sys::BJoltRagdollBuild,
        part_index: i32,
        anchor: Vec3,
        twist_axis1: Vec3,
        plane_axis1: Vec3,
        twist_axis2: Vec3,
        plane_axis2: Vec3,
        normal_half_cone: f32,
        plane_half_cone: f32,
        twist_min: f32,
        twist_max: f32,
    ) -> bool {
        unsafe {
            bjolt_ragdoll_build_set_swing_twist(
                build,
                part_index,
                anchor.x,
                anchor.y,
                anchor.z,
                twist_axis1.x,
                twist_axis1.y,
                twist_axis1.z,
                plane_axis1.x,
                plane_axis1.y,
                plane_axis1.z,
                twist_axis2.x,
                twist_axis2.y,
                twist_axis2.z,
                plane_axis2.x,
                plane_axis2.y,
                plane_axis2.z,
                normal_half_cone,
                plane_half_cone,
                twist_min,
                twist_max,
            )
        }
    }

    /// Mass stabilization (ratio clamp + parent-inertia boost), in place.
    /// Run after all parts, before create. False on failure.
    pub fn ragdoll_build_stabilize(&mut self, build: *mut jolt_sys::BJoltRagdollBuild) -> bool {
        unsafe { bjolt_ragdoll_build_stabilize(build) }
    }

    /// Constraint priorities + shared parent-child no-collide filter.
    pub fn ragdoll_build_finalize(&mut self, build: *mut jolt_sys::BJoltRagdollBuild) {
        unsafe { bjolt_ragdoll_build_finalize(build) }
    }

    /// Creates bodies + constraints and adds them in one shot. Mints a fresh
    /// collision group per ragdoll. Returns the 1-based registry id, or 0 on
    /// failure.
    pub fn ragdoll_create(&mut self, build: *mut jolt_sys::BJoltRagdollBuild) -> u32 {
        let group_id = self.next_ragdoll_group;
        self.next_ragdoll_group += 1;
        unsafe { bjolt_ragdoll_create(self.world_ptr, build, group_id, 0) }
    }

    /// Part count (= body count) of a live ragdoll.
    pub fn ragdoll_body_count(&self, ragdoll_id: u32) -> u32 {
        unsafe { bjolt_ragdoll_body_count(self.world_ptr, ragdoll_id) }
    }

    /// Body ids in part order. Returns ids written.
    pub fn ragdoll_body_ids(&self, ragdoll_id: u32, out_ids: &mut [u32]) -> u32 {
        unsafe {
            bjolt_ragdoll_body_ids(
                self.world_ptr,
                ragdoll_id,
                out_ids.as_mut_ptr(),
                out_ids.len() as u32,
            )
        }
    }

    /// Removes bodies + constraints and releases the registry slot. Never mix
    /// with per-body remove/destroy on these ids. `destroyed_body_ids` (from
    /// `JoltRagdollParts.body_ids`) clears tracked sensor flags so a
    /// recycled raw id never reads stale-disabled.
    pub fn ragdoll_destroy(&mut self, ragdoll_id: u32, destroyed_body_ids: &[u32]) {
        unsafe { bjolt_ragdoll_destroy(self.world_ptr, ragdoll_id) }
        for destroyed_body in destroyed_body_ids {
            self.sensor_bodies.remove(destroyed_body);
        }
    }

    /// Flags every body in the ragdoll as sensor-quiet (or restores
    /// collision). Disable path for settled ragdolls: the bodies stay in
    /// the broadphase but never wake on contact. Reads ids from
    /// `JoltRagdollParts.body_ids`, so callers pass ids, not entities.
    pub fn ragdoll_set_disabled(&mut self, ragdoll_disabled_body_ids: &[u32], ragdoll_disabled: bool) {
        for ragdoll_body in ragdoll_disabled_body_ids {
            unsafe { bjolt_body_set_sensor(self.world_ptr, *ragdoll_body, ragdoll_disabled) }
            if ragdoll_disabled {
                self.sensor_bodies.insert(*ragdoll_body);
            } else {
                self.sensor_bodies.remove(ragdoll_body);
            }
        }
    }

    /// Flips every body in the ragdoll to one motion. Kinematic = follow
    /// bones (hitbox mode, cheap); dynamic = simulate (ragdoll mode).
    pub fn ragdoll_set_motion(&mut self, ragdoll_id: u32, motion: JoltMotion) {
        let motion_code = match motion {
            JoltMotion::Static => 0,
            JoltMotion::Kinematic => 1,
            JoltMotion::Dynamic => 2,
        };
        unsafe { bjolt_ragdoll_set_motion(self.world_ptr, ragdoll_id, motion_code) }
    }

    /// Moves every part body to another object layer (broadphase re-insert,
    /// no velocity change). Pair with the motion flip: quiet team for
    /// hitbox follow, world-meeting team for simulation.
    pub fn ragdoll_set_layer(&mut self, ragdoll_id: u32, object_layer: u16) {
        unsafe { bjolt_ragdoll_set_layer(self.world_ptr, ragdoll_id, object_layer) }
    }

    /// Drives the velocity motor on the joint feeding `part_index` (0 =
    /// root, which has no joint). Speed 0 brakes (motor holds);
    /// [`ragdoll_motor_off`](Self::ragdoll_motor_off) releases. False on a
    /// bad id, the root part, or a mismatched joint type.
    pub fn ragdoll_drive(
        &mut self,
        ragdoll_id: u32,
        part_index: u32,
        drive_axis: RagdollDriveAxis,
        target_velocity: f32,
    ) -> bool {
        let axis = match drive_axis {
            RagdollDriveAxis::Hinge | RagdollDriveAxis::Twist => 0,
            RagdollDriveAxis::Swing => 1,
        };
        unsafe {
            bjolt_ragdoll_drive(
                self.world_ptr,
                ragdoll_id,
                part_index,
                axis,
                target_velocity,
            )
        }
    }

    /// Releases both motors on the joint feeding `part_index`, so the limb
    /// hangs on limits alone. False on a bad id, the root part, or a joint
    /// type with no motors.
    pub fn ragdoll_motor_off(&mut self, ragdoll_id: u32, part_index: u32) -> bool {
        unsafe { bjolt_ragdoll_motor_off(self.world_ptr, ragdoll_id, part_index) }
    }

    /// Frees the builder (settings only, after create).
    pub fn ragdoll_build_destroy(&mut self, build: *mut jolt_sys::BJoltRagdollBuild) {
        unsafe { bjolt_ragdoll_build_destroy(build) }
    }

    /// Creates a virtual character capsule; bottom sits at the position.
    #[allow(clippy::too_many_arguments)]
    pub fn character_create(
        &mut self,
        character_position: Vec3,
        capsule_half_height: f32,
        capsule_radius: f32,
        object_layer: u16,
        mass_kg: f32,
        max_strength: f32,
        max_slope_degrees: f32,
        character_padding: f32,
        penetration_recovery: f32,
    ) -> u32 {
        let character_id_raw = unsafe {
            bjolt_character_create(
                self.world_ptr,
                character_position.x,
                character_position.y,
                character_position.z,
                capsule_half_height,
                capsule_radius,
                object_layer,
                mass_kg,
                max_strength,
                max_slope_degrees,
                character_padding,
                penetration_recovery,
            )
        };
        self.character_shapes
            .insert(character_id_raw, (capsule_half_height, capsule_radius));
        character_id_raw
    }

    /// Destroys a character. Never touches bodies: order-independent.
    pub fn character_destroy(&mut self, character_id_raw: u32) {
        unsafe { bjolt_character_destroy(self.world_ptr, character_id_raw) }
        self.character_shapes.remove(&character_id_raw);
    }

    /// One movement step: sets the velocity and runs ExtendedUpdate (move +
    /// stick-to-floor + walk-stairs). Pose lands in the character; read it
    /// back with [`JoltWorld::character_pose`].
    pub fn character_move(
        &mut self,
        character_id_raw: u32,
        delta_time: f32,
        wanted_velocity: Vec3,
        world_gravity: Vec3,
        step_up_height: f32,
        stick_to_floor_distance: f32,
    ) {
        let mut character_position = [0.0f32; 3];
        let mut character_velocity = [0.0f32; 3];
        let mut ground_normal = [0.0f32; 3];
        let mut ground_state = 0u32;
        let mut is_supported = 0u32;
        unsafe {
            bjolt_character_move(
                self.world_ptr,
                character_id_raw,
                delta_time,
                wanted_velocity.x,
                wanted_velocity.y,
                wanted_velocity.z,
                world_gravity.x,
                world_gravity.y,
                world_gravity.z,
                step_up_height,
                stick_to_floor_distance,
                character_position.as_mut_ptr(),
                character_velocity.as_mut_ptr(),
                ground_normal.as_mut_ptr(),
                &mut ground_state,
                &mut is_supported,
            );
        }
        self.character_positions.insert(
            character_id_raw,
            (
                Vec3::from_array(character_position),
                crate::character::JoltCharacterGround {
                    ground_state: crate::character::CharacterGround::from_raw(ground_state),
                    ground_normal: Vec3::from_array(ground_normal),
                    is_supported: is_supported != 0,
                },
            ),
        );
    }

    /// Last move's position + ground reading for a character.
    pub fn character_pose(
        &mut self,
        character_id_raw: u32,
    ) -> (Vec3, crate::character::JoltCharacterGround) {
        self.character_positions
            .remove(&character_id_raw)
            .unwrap_or((Vec3::ZERO, crate::character::JoltCharacterGround::default()))
    }

    /// Teleports a character to a position with a velocity.
    pub fn character_teleport(
        &mut self,
        character_id_raw: u32,
        target_position: Vec3,
        target_velocity: Vec3,
    ) {
        unsafe {
            bjolt_character_teleport(
                self.world_ptr,
                character_id_raw,
                target_position.x,
                target_position.y,
                target_position.z,
                target_velocity.x,
                target_velocity.y,
                target_velocity.z,
            );
        }
    }

    /// Swaps the character capsule (stand/crouch). Returns false when the new
    /// shape stays penetrating: the old capsule keeps running.
    pub fn character_stance(
        &mut self,
        character_id_raw: u32,
        capsule_half_height: f32,
        capsule_radius: f32,
    ) -> bool {
        unsafe {
            bjolt_character_stance(
                self.world_ptr,
                character_id_raw,
                capsule_half_height,
                capsule_radius,
            )
        }
    }

    /// Sets virtual character rotation (usually yaw only).
    pub fn character_set_rotation(&mut self, character_id_raw: u32, character_rotation: Quat) {
        unsafe {
            bjolt_character_set_rotation(
                self.world_ptr,
                character_id_raw,
                character_rotation.x,
                character_rotation.y,
                character_rotation.z,
                character_rotation.w,
            )
        }
    }

    /// Current virtual character rotation.
    pub fn character_rotation(&self, character_id_raw: u32) -> Quat {
        let mut raw_rotation = [0.0f32, 0.0, 0.0, 1.0];
        unsafe {
            bjolt_character_rotation(self.world_ptr, character_id_raw, raw_rotation.as_mut_ptr())
        }
        Quat::from_array(raw_rotation)
    }

    /// Sets virtual character mass + push strength live.
    pub fn character_set_mass(&mut self, character_id_raw: u32, mass_kg: f32, max_strength: f32) {
        unsafe { bjolt_character_set_mass(self.world_ptr, character_id_raw, mass_kg, max_strength) }
    }

    /// Sets penetration recovery live (padding is construction-only in Jolt).
    pub fn character_set_recovery(&mut self, character_id_raw: u32, penetration_recovery: f32) {
        unsafe {
            bjolt_character_set_padding(self.world_ptr, character_id_raw, 0.0, penetration_recovery)
        }
    }

    /// Sets virtual character up vector + max slope live.
    pub fn character_set_up(
        &mut self,
        character_id_raw: u32,
        up_direction: Vec3,
        max_slope_degrees: f32,
    ) {
        unsafe {
            bjolt_character_set_up(
                self.world_ptr,
                character_id_raw,
                up_direction.x,
                up_direction.y,
                up_direction.z,
                max_slope_degrees,
            )
        }
    }

    /// Sets virtual character shape offset live.
    pub fn character_set_shape_offset(&mut self, character_id_raw: u32, shape_offset: Vec3) {
        unsafe {
            bjolt_character_set_shape_offset(
                self.world_ptr,
                character_id_raw,
                shape_offset.x,
                shape_offset.y,
                shape_offset.z,
            )
        }
    }

    /// Sets virtual character user data.
    pub fn character_set_user_data(&mut self, character_id_raw: u32, user_data: u64) {
        unsafe { bjolt_character_set_user_data(self.world_ptr, character_id_raw, user_data) }
    }

    /// Plain update: collide + settle without stairs or floor stick.
    /// Returns position + velocity + ground reading like the full move.
    pub fn character_update(
        &mut self,
        character_id_raw: u32,
        delta_time: f32,
        wanted_velocity: Vec3,
        world_gravity: Vec3,
    ) -> (Vec3, Vec3, crate::character::JoltCharacterGround) {
        let mut raw_position = [0.0f32; 3];
        let mut raw_velocity = [0.0f32; 3];
        let mut raw_normal = [0.0f32; 3];
        let mut ground_state = 0u32;
        let mut is_supported = 0u32;
        unsafe {
            bjolt_character_update(
                self.world_ptr,
                character_id_raw,
                delta_time,
                wanted_velocity.x,
                wanted_velocity.y,
                wanted_velocity.z,
                world_gravity.x,
                world_gravity.y,
                world_gravity.z,
                raw_position.as_mut_ptr(),
                raw_velocity.as_mut_ptr(),
                raw_normal.as_mut_ptr(),
                &mut ground_state,
                &mut is_supported,
            );
        }
        (
            Vec3::from_array(raw_position),
            Vec3::from_array(raw_velocity),
            crate::character::JoltCharacterGround {
                ground_state: crate::character::CharacterGround::from_raw(ground_state),
                ground_normal: Vec3::from_array(raw_normal),
                is_supported: is_supported != 0,
            },
        )
    }

    /// Whether stairs are climbable in this direction right now.
    pub fn character_can_walk_stairs(
        &mut self,
        character_id_raw: u32,
        wanted_velocity: Vec3,
    ) -> bool {
        unsafe {
            bjolt_character_can_walk_stairs(
                self.world_ptr,
                character_id_raw,
                wanted_velocity.x,
                wanted_velocity.y,
                wanted_velocity.z,
            )
        }
    }

    /// Single stair climb (no settle). Returns true when a step landed.
    pub fn character_walk_stairs(
        &mut self,
        character_id_raw: u32,
        delta_time: f32,
        step_up_height: f32,
        step_forward: f32,
        step_forward_test: f32,
        step_down_extra: f32,
    ) -> bool {
        unsafe {
            bjolt_character_walk_stairs(
                self.world_ptr,
                character_id_raw,
                delta_time,
                step_up_height,
                step_forward,
                step_forward_test,
                step_down_extra,
            )
        }
    }

    /// Single floor stick (no settle). Returns true when ground caught.
    pub fn character_stick_to_floor(
        &mut self,
        character_id_raw: u32,
        stick_down_distance: f32,
    ) -> bool {
        unsafe {
            bjolt_character_stick_to_floor(self.world_ptr, character_id_raw, stick_down_distance)
        }
    }

    /// Re-resolves contacts after an external change (teleport, moving
    /// platform jump). Call after moving things by hand.
    pub fn character_refresh_contacts(&mut self, character_id_raw: u32) {
        unsafe { bjolt_character_refresh_contacts(self.world_ptr, character_id_raw) }
    }

    /// Full ground reading: contact position, surface normal, surface
    /// velocity, supporting body id (0 airborne), body user data.
    pub fn character_ground_detail(
        &mut self,
        character_id_raw: u32,
    ) -> (Vec3, Vec3, Vec3, u32, u64) {
        let mut raw_position = [0.0f32; 3];
        let mut raw_normal = [0.0f32; 3];
        let mut raw_velocity = [0.0f32; 3];
        let mut ground_body = 0u32;
        let mut ground_user_data = 0u64;
        unsafe {
            bjolt_character_ground(
                self.world_ptr,
                character_id_raw,
                raw_position.as_mut_ptr(),
                raw_normal.as_mut_ptr(),
                raw_velocity.as_mut_ptr(),
                &mut ground_body,
                &mut ground_user_data,
            );
        }
        (
            Vec3::from_array(raw_position),
            Vec3::from_array(raw_normal),
            Vec3::from_array(raw_velocity),
            ground_body,
            ground_user_data,
        )
    }

    /// Creates a rigid character: a real capsule body Jolt simulates.
    /// Returns the character id (0 on failure).
    #[allow(clippy::too_many_arguments)]
    pub fn rigid_character_create(
        &mut self,
        character_position: Vec3,
        character_rotation: Quat,
        capsule_half_height: f32,
        capsule_radius: f32,
        object_layer: u16,
        mass_kg: f32,
        friction: f32,
        gravity_factor: f32,
        allowed_dofs: CharacterDofs,
        user_data: u64,
    ) -> u32 {
        let character_id_raw = unsafe {
            bjolt_rigid_character_create(
                self.world_ptr,
                character_position.x,
                character_position.y,
                character_position.z,
                character_rotation.x,
                character_rotation.y,
                character_rotation.z,
                character_rotation.w,
                capsule_half_height,
                capsule_radius,
                object_layer,
                mass_kg,
                friction,
                gravity_factor,
                allowed_dofs.ffi_dofs(),
                user_data,
            )
        };
        self.rigid_character_shapes
            .insert(character_id_raw, (capsule_half_height, capsule_radius));
        character_id_raw
    }

    /// Destroys a rigid character (removes its body first).
    pub fn rigid_character_destroy(&mut self, character_id_raw: u32) {
        unsafe { bjolt_rigid_character_destroy(self.world_ptr, character_id_raw) }
        self.rigid_character_shapes.remove(&character_id_raw);
    }

    /// Refreshes ground state after the physics step. Call every tick
    /// after stepping: without it the ground reading goes stale.
    pub fn rigid_character_post(&mut self, character_id_raw: u32, max_separation: f32) {
        unsafe { bjolt_rigid_character_post(self.world_ptr, character_id_raw, max_separation) }
    }

    /// Drives a rigid character by velocity (linear + angular).
    pub fn rigid_character_set_velocity(
        &mut self,
        character_id_raw: u32,
        linear_velocity: Vec3,
        angular_velocity: Vec3,
    ) {
        unsafe {
            bjolt_rigid_character_set_velocity(
                self.world_ptr,
                character_id_raw,
                linear_velocity.x,
                linear_velocity.y,
                linear_velocity.z,
                angular_velocity.x,
                angular_velocity.y,
                angular_velocity.z,
            )
        }
    }

    /// Adds world-space velocity to a rigid character.
    pub fn rigid_character_add_velocity(&mut self, character_id_raw: u32, extra_velocity: Vec3) {
        unsafe {
            bjolt_rigid_character_add_velocity(
                self.world_ptr,
                character_id_raw,
                extra_velocity.x,
                extra_velocity.y,
                extra_velocity.z,
            )
        }
    }

    /// Kicks a rigid character at its center of mass.
    pub fn rigid_character_add_impulse(&mut self, character_id_raw: u32, kick_impulse: Vec3) {
        unsafe {
            bjolt_rigid_character_add_impulse(
                self.world_ptr,
                character_id_raw,
                kick_impulse.x,
                kick_impulse.y,
                kick_impulse.z,
            )
        }
    }

    /// Rigid character pose + ground reading: position, rotation, velocity,
    /// ground normal/state/support.
    pub fn rigid_character_pose(
        &mut self,
        character_id_raw: u32,
    ) -> (Vec3, Quat, Vec3, crate::character::JoltCharacterGround) {
        let mut raw_position = [0.0f32; 3];
        let mut raw_rotation = [0.0f32, 0.0, 0.0, 1.0];
        let mut raw_velocity = [0.0f32; 3];
        let mut raw_normal = [0.0f32; 3];
        let mut ground_state = 0u32;
        let mut is_supported = 0u32;
        unsafe {
            bjolt_rigid_character_pose(
                self.world_ptr,
                character_id_raw,
                raw_position.as_mut_ptr(),
                raw_rotation.as_mut_ptr(),
                raw_velocity.as_mut_ptr(),
                raw_normal.as_mut_ptr(),
                &mut ground_state,
                &mut is_supported,
            );
        }
        (
            Vec3::from_array(raw_position),
            Quat::from_array(raw_rotation),
            Vec3::from_array(raw_velocity),
            crate::character::JoltCharacterGround {
                ground_state: crate::character::CharacterGround::from_raw(ground_state),
                ground_normal: Vec3::from_array(raw_normal),
                is_supported: is_supported != 0,
            },
        )
    }

    /// Teleports a rigid character (position + rotation, keeps momentum:
    /// zero velocity separately for a dead stop).
    pub fn rigid_character_set_pose(
        &mut self,
        character_id_raw: u32,
        target_position: Vec3,
        target_rotation: Quat,
    ) {
        unsafe {
            bjolt_rigid_character_set_pose(
                self.world_ptr,
                character_id_raw,
                target_position.x,
                target_position.y,
                target_position.z,
                target_rotation.x,
                target_rotation.y,
                target_rotation.z,
                target_rotation.w,
            )
        }
    }

    /// Underlying rigid body id (for queries, sensors, debug draw).
    pub fn rigid_character_body(&mut self, character_id_raw: u32) -> u32 {
        unsafe { bjolt_rigid_character_body(self.world_ptr, character_id_raw) }
    }

    /// Moves a rigid character to another collision layer.
    pub fn rigid_character_set_layer(&mut self, character_id_raw: u32, object_layer: u16) {
        unsafe { bjolt_rigid_character_set_layer(self.world_ptr, character_id_raw, object_layer) }
    }

    /// Swaps the rigid capsule (stand/crouch). False = still penetrating,
    /// old capsule keeps running.
    pub fn rigid_character_stance(
        &mut self,
        character_id_raw: u32,
        capsule_half_height: f32,
        capsule_radius: f32,
        max_penetration: f32,
    ) -> bool {
        unsafe {
            bjolt_rigid_character_stance(
                self.world_ptr,
                character_id_raw,
                capsule_half_height,
                capsule_radius,
                max_penetration,
            )
        }
    }

    /// Rigid ground detail: contact position, normal, surface velocity,
    /// supporting body id, body user data.
    pub fn rigid_character_ground_detail(
        &mut self,
        character_id_raw: u32,
    ) -> (Vec3, Vec3, Vec3, u32, u64) {
        let mut raw_position = [0.0f32; 3];
        let mut raw_normal = [0.0f32; 3];
        let mut raw_velocity = [0.0f32; 3];
        let mut ground_body = 0u32;
        let mut ground_user_data = 0u64;
        unsafe {
            bjolt_rigid_character_ground(
                self.world_ptr,
                character_id_raw,
                raw_position.as_mut_ptr(),
                raw_normal.as_mut_ptr(),
                raw_velocity.as_mut_ptr(),
                &mut ground_body,
                &mut ground_user_data,
            );
        }
        (
            Vec3::from_array(raw_position),
            Vec3::from_array(raw_normal),
            Vec3::from_array(raw_velocity),
            ground_body,
            ground_user_data,
        )
    }
}

impl Default for JoltWorld {
    fn default() -> Self {
        Self::new()
    }
}

impl Drop for JoltWorld {
    fn drop(&mut self) {
        unsafe { bjolt_world_destroy(self.world_ptr) }
    }
}

// The world pointer is only touched through `&mut self` for mutation.
// Concurrent `&self` reads (position / active checks) go through Jolt's
// internal body locks, so sharing across threads is sound.
unsafe impl Send for JoltWorld {}

unsafe impl Sync for JoltWorld {}
