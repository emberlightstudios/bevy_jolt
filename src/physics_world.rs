//! Owned Jolt physics world: lifetime, body API, stepping.
//!
//! The solver itself runs in C++ on Jolt's ThreadPool job system (SIMD stays
//! on via the `jolt_sys` build). This type only owns the world pointer.

use crate::body_sync::JoltMotion;
use bevy::prelude::{Dir3, Quat, Vec3};
use jolt_sys::{
    BJoltWorld, bjolt_apply_force, bjolt_apply_impulse, bjolt_body_is_active, bjolt_body_remove_destroy,
    bjolt_body_state, bjolt_body_transform, bjolt_car_bounds, bjolt_cast_ray, bjolt_constraint_drive_at,
    bjolt_create_box, bjolt_create_capsule, bjolt_create_cone_constraint, bjolt_create_cylinder,
    bjolt_create_demo_car, bjolt_create_distance_constraint, bjolt_create_fixed_constraint,
    bjolt_create_floor, bjolt_create_gear_constraint, bjolt_create_hinge_constraint,
    bjolt_create_path_cart, bjolt_create_plane, bjolt_create_point_constraint,
    bjolt_create_pulley_constraint, bjolt_create_rack_pinion_constraint, bjolt_create_six_dof_slider,
    bjolt_create_sphere, bjolt_create_slider_constraint, bjolt_create_swing_twist_constraint,
    bjolt_create_tapered_capsule, bjolt_create_tapered_cylinder, bjolt_init, bjolt_kick_body,
    bjolt_move_kinematic, bjolt_remove_constraint, bjolt_reset_body, bjolt_set_angular_velocity,
    bjolt_set_linear_velocity, bjolt_set_velocity, bjolt_vehicle_drive,
    bjolt_world_create_with_layers, bjolt_world_destroy, bjolt_world_update,
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

/// Position and linear velocity of one body at the current step.
pub struct BodySnapshot {
    pub body_position: Vec3,
    pub body_velocity: Vec3,
}

/// Closest body hit by a ray cast: body id plus fraction along the ray.
pub struct RayHit {
    pub hit_body_id: u32,
    pub hit_fraction: f32,
}

/// What a body looks like, remembered at creation so the debug visualizer
/// can draw it. Jolt owns the real shape; this is just the outline recipe.
#[derive(Clone, Copy, Debug)]
pub enum PhysicsShape {
    Box { half_extents: Vec3 },
    Sphere { sphere_radius: f32 },
    Capsule { capsule_half_height: f32, capsule_radius: f32 },
    Cylinder { cylinder_half_height: f32, cylinder_radius: f32 },
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
    Plane { surface_normal: Vec3, plane_constant: f32 },
}

/// Which object layers exist and which pairs can collide, decided in Rust
/// and handed to Jolt once at world creation. The default table uses
/// [`CollisionLayers::NON_MOVING`] (static ground) and
/// [`CollisionLayers::MOVING`] (dynamic bodies); custom teams start at 2.
#[derive(Clone, Debug)]
pub struct CollisionLayers {
    layer_count: usize,
    collide_matrix: [[u8; jolt_sys::MAX_OBJECT_LAYERS]; jolt_sys::MAX_OBJECT_LAYERS],
}

impl CollisionLayers {
    /// Static ground layer in the default table.
    pub const NON_MOVING: u16 = 0;

    /// Dynamic bodies layer in the default table.
    pub const MOVING: u16 = 1;

    pub fn new(layer_count: usize) -> Self {
        assert!(
            (2..=jolt_sys::MAX_OBJECT_LAYERS).contains(&layer_count),
            "layer count {} out of range 2..={}",
            layer_count,
            jolt_sys::MAX_OBJECT_LAYERS
        );
        let mut collide_matrix = jolt_sys::default_collision_matrix();
        // New custom layers collide with everything until told otherwise.
        for custom_layer in 2..layer_count {
            for other_layer in 0..layer_count {
                collide_matrix[custom_layer][other_layer] = 1;
                collide_matrix[other_layer][custom_layer] = 1;
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
        Self::new(2)
    }
}

/// A Jolt physics world: floor + dynamic bodies, stepped on the ThreadPool job system.
pub struct JoltWorld {
    world_ptr: *mut BJoltWorld,
    body_shapes: std::collections::HashMap<u32, PhysicsShape>,
}

impl JoltWorld {
    pub fn new() -> Self {
        Self::with_layers(CollisionLayers::default())
    }

    pub fn with_layers(collision_layers: CollisionLayers) -> Self {
        let world_ptr = unsafe {
            assert!(bjolt_init(), "Jolt initialization failed");
            bjolt_world_create_with_layers(
                collision_layers.layer_count as u32,
                collision_layers.collide_matrix.as_ptr() as *const u8,
            )
        };
        assert!(!world_ptr.is_null(), "Jolt world creation failed");
        Self {
            world_ptr,
            body_shapes: std::collections::HashMap::new(),
        }
    }

    /// Every known body and its outline recipe, for the debug visualizer.
    pub fn body_shapes(&self) -> &std::collections::HashMap<u32, PhysicsShape> {
        &self.body_shapes
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
        self.body_shapes.insert(
            body_id_raw,
            PhysicsShape::Box { half_extents },
        );
        body_id_raw
    }

    pub fn create_sphere(
        &mut self,
        sphere_radius: f32,
        spawn_position: Vec3,
        object_layer: u16,
        density_kg_per_m3: f32,
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
            )
        };
        self.body_shapes
            .insert(body_id_raw, PhysicsShape::Sphere { sphere_radius });
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
        self.body_shapes.insert(
            body_id_raw,
            PhysicsShape::Plane {
                surface_normal,
                plane_constant,
            },
        );
        body_id_raw
    }

    pub fn create_box(
        &mut self,
        half_extents: Vec3,
        spawn_position: Vec3,
        object_layer: u16,
        motion: JoltMotion,
        density_kg_per_m3: f32,
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
            )
        };
        self.body_shapes.insert(
            body_id_raw,
            PhysicsShape::Box { half_extents },
        );
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
            )
        };
        self.body_shapes.insert(
            body_id_raw,
            PhysicsShape::Capsule {
                capsule_half_height,
                capsule_radius,
            },
        );
        body_id_raw
    }

    pub fn create_cylinder(
        &mut self,
        cylinder_half_height: f32,
        cylinder_radius: f32,
        spawn_position: Vec3,
        object_layer: u16,
        density_kg_per_m3: f32,
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
            )
        };
        self.body_shapes.insert(
            body_id_raw,
            PhysicsShape::Cylinder {
                cylinder_half_height,
                cylinder_radius,
            },
        );
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
            )
        };
        self.body_shapes.insert(
            body_id_raw,
            PhysicsShape::TaperedCylinder {
                tapered_half_height,
                top_radius,
                bottom_radius,
            },
        );
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
            )
        };
        self.body_shapes.insert(
            body_id_raw,
            PhysicsShape::TaperedCapsule {
                tapered_half_height,
                top_radius,
                bottom_radius,
            },
        );
        body_id_raw
    }

    pub fn body_full_transform(&self, body_id_raw: u32) -> (Vec3, Quat) {
        let mut body_position = [0.0f32; 3];
        let mut body_rotation = [0.0f32; 4];
        unsafe {
            bjolt_body_transform(
                self.world_ptr,
                body_id_raw,
                body_position.as_mut_ptr(),
                body_rotation.as_mut_ptr(),
            );
        }
        (
            Vec3::from_array(body_position),
            Quat::from_array(body_rotation),
        )
    }

    pub fn cast_ray(&self, ray_origin: Vec3, ray_direction: Vec3) -> Option<RayHit> {
        let mut hit_body_id = 0u32;
        let mut hit_fraction = 0.0f32;
        let found_hit = unsafe {
            bjolt_cast_ray(
                self.world_ptr,
                ray_origin.x,
                ray_origin.y,
                ray_origin.z,
                ray_direction.x,
                ray_direction.y,
                ray_direction.z,
                &mut hit_body_id,
                &mut hit_fraction,
            )
        };
        if found_hit {
            Some(RayHit {
                hit_body_id,
                hit_fraction,
            })
        } else {
            None
        }
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
    /// rotation; the normal axis defines angle zero. Returns 0 on failure.
    pub fn create_hinge_constraint(
        &mut self,
        body1_raw: u32,
        body2_raw: u32,
        hinge_point: Vec3,
        hinge_axis: Dir3,
        normal_axis: Dir3,
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
                hinge_axis.as_vec3().x,
                hinge_axis.as_vec3().y,
                hinge_axis.as_vec3().z,
                normal_axis.as_vec3().x,
                normal_axis.as_vec3().y,
                normal_axis.as_vec3().z,
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
    /// current relative pose otherwise. Returns 0 on failure.
    pub fn create_slider_constraint(
        &mut self,
        body1_raw: u32,
        body2_raw: u32,
        slider_axis: Dir3,
        normal_axis: Dir3,
        limits_min: f32,
        limits_max: f32,
        joint_space: JointSpace,
    ) -> u32 {
        unsafe {
            bjolt_create_slider_constraint(
                self.world_ptr,
                body1_raw,
                body2_raw,
                slider_axis.as_vec3().x,
                slider_axis.as_vec3().y,
                slider_axis.as_vec3().z,
                normal_axis.as_vec3().x,
                normal_axis.as_vec3().y,
                normal_axis.as_vec3().z,
                limits_min,
                limits_max,
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
    /// Returns 0 on failure.
    pub fn create_swing_twist_constraint(
        &mut self,
        body1_raw: u32,
        body2_raw: u32,
        constraint_position: Vec3,
        twist_axis: Dir3,
        plane_axis: Dir3,
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
                twist_axis.as_vec3().x,
                twist_axis.as_vec3().y,
                twist_axis.as_vec3().z,
                plane_axis.as_vec3().x,
                plane_axis.as_vec3().y,
                plane_axis.as_vec3().z,
                normal_half_cone_angle,
                plane_half_cone_angle,
                twist_min_angle,
                twist_max_angle,
                joint_space.ffi_space(),
            )
        }
    }

    /// Six-DOF used as a piston: everything locked except free Y travel.
    /// Positions lock to the current poses plus the Y band. Returns 0 on failure.
    pub fn create_six_dof_slider(
        &mut self,
        body1_raw: u32,
        body2_raw: u32,
        limit_y_min: f32,
        limit_y_max: f32,
    ) -> u32 {
        unsafe { bjolt_create_six_dof_slider(self.world_ptr, body1_raw, body2_raw, limit_y_min, limit_y_max) }
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

    /// Glues a body to a straight track so it shuttles between two stops.
    /// Needs a static anchor body plus the cart body. Returns 0 on failure.
    pub fn create_path_cart(
        &mut self,
        static_body_raw: u32,
        cart_body_raw: u32,
        track_from: Vec3,
        track_to: Vec3,
    ) -> u32 {
        unsafe {
            bjolt_create_path_cart(
                self.world_ptr,
                static_body_raw,
                cart_body_raw,
                track_from.x,
                track_from.y,
                track_from.z,
                track_to.x,
                track_to.y,
                track_to.z,
            )
        }
    }

    /// Four-wheel demo car: spawns the chassis body plus ray-cast wheels and
    /// returns both the body and the vehicle constraint id. Drive it with
    /// [`JoltWorld::vehicle_drive`]. Returns `None` on failure.
    pub fn create_demo_car(
        &mut self,
        object_layer: u16,
        spawn_position: Vec3,
    ) -> Option<(u32, u32)> {
        let mut body_raw = 0u32;
        let mut constraint_id = 0u32;
        let created = unsafe {
            bjolt_create_demo_car(
                self.world_ptr,
                object_layer,
                spawn_position.x,
                spawn_position.y,
                spawn_position.z,
                &mut body_raw,
                &mut constraint_id,
            )
        };
        (created != 0).then_some((body_raw, constraint_id))
    }

    /// Gas, steering, brake in [-1, 1]-ish ranges. No-op on bad ids.
    pub fn vehicle_drive(&mut self, constraint_id: u32, forward: f32, right: f32, brake: f32) {
        unsafe { bjolt_vehicle_drive(self.world_ptr, constraint_id, forward, right, brake) }
    }

    /// One-shot velocity kick so a resting rig shows motion immediately.
    pub fn kick_body(&mut self, body_id_raw: u32, velocity: Vec3) {
        unsafe { bjolt_kick_body(self.world_ptr, body_id_raw, velocity.x, velocity.y, velocity.z) }
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
    pub fn set_body_velocity(&mut self, body_id_raw: u32, linear_velocity: Vec3, angular_velocity: Vec3) {
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


    /// Velocity motor on a slider, hinge, or path joint. False on bad ids.
    pub fn constraint_drive_at(&mut self, constraint_id: u32, target_velocity: f32) -> bool {
        unsafe { bjolt_constraint_drive_at(self.world_ptr, constraint_id, target_velocity) }
    }

    /// Teleports the car body back inside a rectangle when it leaves.
    /// Cheap demo guard so the car can drive without a chase camera.
    pub fn clamp_car_to_bounds(&mut self, body_id_raw: u32, min: Vec3, max: Vec3) {
        unsafe {
            bjolt_car_bounds(
                self.world_ptr, body_id_raw, min.x, max.x, min.z, max.z,
            )
        }
    }

    /// Drops a body back at a spawn pose with zero velocity. Demo looping.
    pub fn reset_body_to(&mut self, body_id_raw: u32, spawn_position: Vec3) {
        unsafe {
            bjolt_reset_body(
                self.world_ptr, body_id_raw, spawn_position.x, spawn_position.y, spawn_position.z,
            )
        }
    }

    pub fn body_is_active(&self, body_id_raw: u32) -> bool {
        unsafe { bjolt_body_is_active(self.world_ptr, body_id_raw) }
    }

    pub fn body_snapshot(&self, body_id_raw: u32) -> BodySnapshot {
        let mut body_position = [0.0f32; 3];
        let mut body_velocity = [0.0f32; 3];
        unsafe {
            bjolt_body_state(
                self.world_ptr,
                body_id_raw,
                body_position.as_mut_ptr(),
                body_velocity.as_mut_ptr(),
            );
        }
        BodySnapshot {
            body_position: Vec3::from_array(body_position),
            body_velocity: Vec3::from_array(body_velocity),
        }
    }

    pub fn remove_and_destroy_body(&mut self, body_id_raw: u32) {
        unsafe { bjolt_body_remove_destroy(self.world_ptr, body_id_raw) }
        self.body_shapes.remove(&body_id_raw);
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
