//! Owned Jolt physics world: lifetime, body API, stepping.
//!
//! The solver itself runs in C++ on Jolt's ThreadPool job system (SIMD stays
//! on via the `jolt_sys` build). This type only owns the world pointer.

use bevy::prelude::{Quat, Vec3};
use jolt_sys::{
    BJoltWorld, bjolt_body_is_active, bjolt_body_remove_destroy, bjolt_body_state,
    bjolt_body_transform, bjolt_cast_ray, bjolt_create_box, bjolt_create_capsule,
    bjolt_create_floor, bjolt_create_plane, bjolt_create_sphere, bjolt_init, bjolt_world_create,
    bjolt_world_destroy, bjolt_world_update,
};

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
    Plane { surface_normal: Vec3, plane_constant: f32 },
}

/// A Jolt physics world: floor + dynamic bodies, stepped on the ThreadPool job system.
pub struct JoltWorld {
    world_ptr: *mut BJoltWorld,
    body_shapes: std::collections::HashMap<u32, PhysicsShape>,
}

impl JoltWorld {
    pub fn new() -> Self {
        let world_ptr = unsafe {
            assert!(bjolt_init(), "Jolt initialization failed");
            bjolt_world_create()
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

    pub fn create_sphere(&mut self, sphere_radius: f32, spawn_height: f32) -> u32 {
        let body_id_raw =
            unsafe { bjolt_create_sphere(self.world_ptr, sphere_radius, spawn_height) };
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
        is_static: bool,
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
                is_static,
            )
        };
        self.body_shapes.insert(
            body_id_raw,
            PhysicsShape::Box { half_extents },
        );
        body_id_raw
    }

    pub fn create_capsule(
        &mut self,
        capsule_half_height: f32,
        capsule_radius: f32,
        spawn_position: Vec3,
        object_layer: u16,
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
