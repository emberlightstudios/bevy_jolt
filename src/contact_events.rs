//! Contact + sensor events: what touched what, as Bevy events.
//!
//! Jolt records begin/end pairs during the step (no Bevy access under
//! solver locks). [`drain_contact_events`] runs after the physics step,
//! resolves raw body ids to entities, and fires [`JoltContactAdded`] /
//! [`JoltContactRemoved`]. Bodies whose entity is gone (despawned parts)
//! are skipped: events are observations, never commands.
//!
//! Sensors: mark a body with [`JoltSensor`] and it overlaps without pushing
//! (trigger volumes, pickups, checkpoints). Sensor pairs arrive through the
//! same events — check for the marker on either entity.

use bevy::prelude::*;
// FFI via JoltWorld.

use crate::body_sync::JoltBodyId;
use crate::plugin::JoltPhysicsWorld;

/// Max contact pairs drained per step. Overflow drops the newest; the queue
/// resets every drain so one busy tick never poisons the next.
pub const MAX_CONTACT_EVENTS: usize = 128;

/// Marks a body as a sensor at bake: overlaps report through contact events
/// but apply no collision response. Add alongside [`JoltBody`](crate::JoltBody).
#[derive(Component, Clone, Copy, Debug, Default)]
pub struct JoltSensor;

/// Two bodies started touching this step. Either entity may carry
/// [`JoltSensor`] (trigger overlap, no push); otherwise a real collision.
#[derive(EntityEvent, Clone, Copy, Debug)]
pub struct JoltContactAdded {
    #[event_target]
    pub first_entity: Entity,
    pub second_entity: Entity,
}

/// Two bodies stopped touching this step.
#[derive(EntityEvent, Clone, Copy, Debug)]
pub struct JoltContactRemoved {
    #[event_target]
    pub first_entity: Entity,
    pub second_entity: Entity,
}

/// Applies [`JoltSensor`] at bake: flags the Jolt body so overlaps report
/// without pushing. Bodies missing their id (not baked yet) keep the marker
/// and retry next tick via the poll below... (see `apply_pending_sensors`).
pub fn bake_jolt_sensor(
    trigger: On<Add, JoltSensor>,
    sensor_query: Query<&JoltBodyId>,
    mut physics_world: ResMut<JoltPhysicsWorld>,
    mut commands: Commands,
) {
    let sensor_entity = trigger.event().entity;
    if let Ok(body_id) = sensor_query.get(sensor_entity) {
        physics_world.set_body_sensor(body_id.body_id_raw, true);
    } else {
        commands.entity(sensor_entity).insert(PendingSensor);
    }
}

/// Retry marker for sensors added before their body baked. Removed once the
/// flag lands.
#[derive(Component, Clone, Copy, Debug, Default)]
pub struct PendingSensor;

/// Flags bodies whose [`JoltSensor`] arrived before bake. Runs before the
/// physics step so triggers never miss their first overlap.
pub fn apply_pending_sensors(
    pending_query: Query<(Entity, &JoltBodyId), With<PendingSensor>>,
    mut commands: Commands,
    mut physics_world: ResMut<JoltPhysicsWorld>,
) {
    for (sensor_entity, body_id) in &pending_query {
        physics_world.set_body_sensor(body_id.body_id_raw, true);
        commands.entity(sensor_entity).remove::<PendingSensor>();
    }
}

/// Drains Jolt's contact queues after the physics step and fires Bevy
/// events. Entity lookup is a hash scan per pair — fine at this cap.
pub fn drain_contact_events(
    mut commands: Commands,
    mut physics_world: ResMut<JoltPhysicsWorld>,
    body_entities: Query<(Entity, &JoltBodyId)>,
) {
    let mut pair_a = [0u32; MAX_CONTACT_EVENTS];
    let mut pair_b = [0u32; MAX_CONTACT_EVENTS];
    let added_kept = physics_world.drain_contact_added(&mut pair_a, &mut pair_b);
    for pair_index in 0..added_kept as usize {
        let Some((first_entity, _)) = body_entities
            .iter()
            .find(|(_, body_id)| body_id.body_id_raw == pair_a[pair_index])
        else {
            continue;
        };
        let Some((second_entity, _)) = body_entities
            .iter()
            .find(|(_, body_id)| body_id.body_id_raw == pair_b[pair_index])
        else {
            continue;
        };
        commands.trigger(JoltContactAdded {
            first_entity,
            second_entity,
        });
    }
    let removed_kept = physics_world.drain_contact_removed(&mut pair_a, &mut pair_b);
    for pair_index in 0..removed_kept as usize {
        let Some((first_entity, _)) = body_entities
            .iter()
            .find(|(_, body_id)| body_id.body_id_raw == pair_a[pair_index])
        else {
            continue;
        };
        let Some((second_entity, _)) = body_entities
            .iter()
            .find(|(_, body_id)| body_id.body_id_raw == pair_b[pair_index])
        else {
            continue;
        };
        commands.trigger(JoltContactRemoved {
            first_entity,
            second_entity,
        });
    }
}
