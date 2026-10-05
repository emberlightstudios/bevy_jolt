//! Spatial queries against the Jolt world: rays, points, overlaps, sweeps.
//!
//! Every query is a plain method on [`JoltWorld`] returning owned hit lists.
//! Batch queries cap the report at [`MAX_QUERY_HITS`] entries: Jolt always
//! searches everything, the cap only bounds the returned list (hits come
//! back nearest-first, so the cap drops the farthest ones).

use bevy::prelude::Vec3;
use jolt_sys::{
    BJoltWorld, bjolt_cast_ray_all, bjolt_cast_shape_all, bjolt_collide_point_all,
    bjolt_overlap_shape_all,
};

/// Max hits any batch query reports. Nearest-first, so overflow drops the
/// farthest hits; the world search itself is never truncated.
pub const MAX_QUERY_HITS: usize = 32;

/// One body a ray (or sweep) passed through: which body, how far along, and
/// where the contact sits in world space.
#[derive(Clone, Copy, Debug)]
pub struct RayHit {
    pub hit_body_id: u32,
    pub hit_fraction: f32,
    pub hit_contact: Vec3,
}

/// One body overlapping a probe volume: which body, how deep the
/// penetration runs, and the contact point in world space.
#[derive(Clone, Copy, Debug)]
pub struct OverlapHit {
    pub hit_body_id: u32,
    pub penetration_depth: f32,
    pub hit_contact: Vec3,
}

/// Probe volume for overlap and sweep queries. Box spans its half extents;
/// sphere uses one radius.
#[derive(Clone, Copy, Debug)]
pub enum QueryProbe {
    Box { probe_half_extents: Vec3 },
    Sphere { probe_radius: f32 },
}

impl QueryProbe {
    fn ffi_kind(self) -> u32 {
        match self {
            QueryProbe::Box { .. } => 0,
            QueryProbe::Sphere { .. } => 1,
        }
    }

    fn ffi_halves(self) -> (f32, f32, f32) {
        match self {
            QueryProbe::Box {
                probe_half_extents,
            } => (probe_half_extents.x, probe_half_extents.y, probe_half_extents.z),
            QueryProbe::Sphere { probe_radius } => (probe_radius, 0.0, 0.0),
        }
    }
}

/// Every body a ray passes through, nearest first. Empty when the ray hits
/// nothing. `ray_direction` sets both direction and reach: hits past
/// `origin + direction` are not reported.
pub fn cast_ray_all(world_ptr: *mut BJoltWorld, ray_origin: Vec3, ray_direction: Vec3) -> Vec<RayHit> {
    let mut hit_bodies = [0u32; MAX_QUERY_HITS];
    let mut hit_fractions = [0.0f32; MAX_QUERY_HITS];
    let hit_total = unsafe {
        bjolt_cast_ray_all(
            world_ptr,
            ray_origin.x,
            ray_origin.y,
            ray_origin.z,
            ray_direction.x,
            ray_direction.y,
            ray_direction.z,
            hit_bodies.as_mut_ptr(),
            hit_fractions.as_mut_ptr(),
            MAX_QUERY_HITS as u32,
        )
    };
    let hit_kept = (hit_total as usize).min(MAX_QUERY_HITS);
    (0..hit_kept)
        .map(|hit_index| {
            let hit_fraction = hit_fractions[hit_index];
            RayHit {
                hit_body_id: hit_bodies[hit_index],
                hit_fraction,
                hit_contact: ray_origin + ray_direction * hit_fraction,
            }
        })
        .collect()
}

/// Every body containing a point (solid shapes count as filled). Empty when
/// no body covers the point.
pub fn collide_point_all(world_ptr: *mut BJoltWorld, probe_point: Vec3) -> Vec<u32> {
    let mut hit_bodies = [0u32; MAX_QUERY_HITS];
    let mut unused_fractions = [0.0f32; MAX_QUERY_HITS];
    let hit_total = unsafe {
        bjolt_collide_point_all(
            world_ptr,
            probe_point.x,
            probe_point.y,
            probe_point.z,
            hit_bodies.as_mut_ptr(),
            unused_fractions.as_mut_ptr(),
            MAX_QUERY_HITS as u32,
        )
    };
    let hit_kept = (hit_total as usize).min(MAX_QUERY_HITS);
    hit_bodies[..hit_kept].to_vec()
}

/// Every body overlapping a probe volume centered at a point. Identity
/// rotation on the probe; contact points come back in world space.
pub fn overlap_shape_all(
    world_ptr: *mut BJoltWorld,
    probe: QueryProbe,
    probe_center: Vec3,
) -> Vec<OverlapHit> {
    let (half_x, half_y, half_z) = probe.ffi_halves();
    let mut hit_bodies = [0u32; MAX_QUERY_HITS];
    let mut hit_depths = [0.0f32; MAX_QUERY_HITS];
    let mut contact_flat = [0.0f32; 3 * MAX_QUERY_HITS];
    let hit_total = unsafe {
        bjolt_overlap_shape_all(
            world_ptr,
            probe.ffi_kind(),
            probe_center.x,
            probe_center.y,
            probe_center.z,
            half_x,
            half_y,
            half_z,
            hit_bodies.as_mut_ptr(),
            hit_depths.as_mut_ptr(),
            contact_flat.as_mut_ptr(),
            MAX_QUERY_HITS as u32,
        )
    };
    let hit_kept = (hit_total as usize).min(MAX_QUERY_HITS);
    (0..hit_kept)
        .map(|hit_index| OverlapHit {
            hit_body_id: hit_bodies[hit_index],
            penetration_depth: hit_depths[hit_index],
            hit_contact: Vec3::new(
                contact_flat[3 * hit_index],
                contact_flat[3 * hit_index + 1],
                contact_flat[3 * hit_index + 2],
            ),
        })
        .collect()
}

/// Sweeps a probe volume along a direction and reports every body touched,
/// nearest first. `cast_direction` sets both direction and reach, like the
/// ray in [`cast_ray_all`].
pub fn cast_shape_all(
    world_ptr: *mut BJoltWorld,
    probe: QueryProbe,
    probe_center: Vec3,
    cast_direction: Vec3,
) -> Vec<RayHit> {
    let (half_x, half_y, half_z) = probe.ffi_halves();
    let mut hit_bodies = [0u32; MAX_QUERY_HITS];
    let mut hit_fractions = [0.0f32; MAX_QUERY_HITS];
    let mut contact_flat = [0.0f32; 3 * MAX_QUERY_HITS];
    let hit_total = unsafe {
        bjolt_cast_shape_all(
            world_ptr,
            probe.ffi_kind(),
            probe_center.x,
            probe_center.y,
            probe_center.z,
            half_x,
            half_y,
            half_z,
            cast_direction.x,
            cast_direction.y,
            cast_direction.z,
            hit_bodies.as_mut_ptr(),
            hit_fractions.as_mut_ptr(),
            contact_flat.as_mut_ptr(),
            MAX_QUERY_HITS as u32,
        )
    };
    let hit_kept = (hit_total as usize).min(MAX_QUERY_HITS);
    (0..hit_kept)
        .map(|hit_index| RayHit {
            hit_body_id: hit_bodies[hit_index],
            hit_fraction: hit_fractions[hit_index],
            hit_contact: Vec3::new(
                contact_flat[3 * hit_index],
                contact_flat[3 * hit_index + 1],
                contact_flat[3 * hit_index + 2],
            ),
        })
        .collect()
}
