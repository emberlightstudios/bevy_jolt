//! Buoyancy: flat water that floats opted-in bodies. The game owns the
//! water level; Jolt computes the submerged volume and pushes up each tick.
//!
//! Add [`JoltWater`] once (surface height + floatiness + drag), then tag
//! floating bodies with [`JoltBuoyant`]. The system runs before the physics
//! step, same as forces.

use bevy::prelude::*;

use crate::body_sync::JoltBodyId;
use crate::plugin::JoltPhysicsWorld;

/// One flat water volume: surface height plus how hard it pushes and grips.
/// Insert once; every [`JoltBuoyant`] body floats on the same water.
#[derive(Resource, Clone, Copy, Debug)]
pub struct JoltWater {
    /// World y of the water surface. Parts below this get pushed up.
    pub surface_height: f32,
    /// Float strength 0..1+: 1 floats neutrally, higher pops out faster.
    pub buoyancy: f32,
    /// Water drag on straight motion while submerged.
    pub linear_drag: f32,
    /// Water drag on spin while submerged.
    pub angular_drag: f32,
    /// Current flow (rivers push, lakes sit still).
    pub current_velocity: Vec3,
}

/// Tags a body as floating: the pre-step system applies buoyancy every
/// Fixed tick while water exists. Remove to sink.
#[derive(Component, Clone, Copy, Debug)]
pub struct JoltBuoyant {
    /// Float strength multiplier on top of the water's own buoyancy.
    /// 1 takes the water as-is; heavier floaters use less, rafts use more.
    pub buoyancy_scale: f32,
}

/// Pushes every [`JoltBuoyant`] body up by its submerged volume, before the
/// physics step. Bodies missing their id (not baked yet) are skipped for
/// the tick. No water resource = no floating, bodies sink as usual.
pub fn apply_buoyancy(
    buoyant_query: Query<(&JoltBodyId, &JoltBuoyant)>,
    water: Option<Res<JoltWater>>,
    fixed_time: Res<Time<Fixed>>,
    mut physics_world: ResMut<JoltPhysicsWorld>,
) {
    let Some(water) = water else {
        return;
    };
    let tick_delta = physics_world.sim_tick_delta(fixed_time.delta().as_secs_f32());
    let world_gravity = physics_world.world_gravity();
    for (body_id, buoyant) in &buoyant_query {
        physics_world.apply_buoyancy(
            body_id.body_id_raw,
            water.surface_height,
            water.buoyancy * buoyant.buoyancy_scale,
            water.linear_drag,
            water.angular_drag,
            water.current_velocity,
            world_gravity,
            tick_delta,
        );
    }
}
