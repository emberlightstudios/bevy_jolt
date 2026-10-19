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

/// Un-flags the body when [`JoltSensor`] leaves it: on despawn, or when game
/// code removes just the marker. The flag lives on the Jolt body, not on the
/// marker, so nothing else clears it and the body would stay non-colliding for
/// good. Mirror of `remove_jolt_disabled`; a body that never baked is skipped
/// (removal means gone), and the parked `PendingSensor` is dropped by the retry
/// below without ever flagging.
pub fn remove_jolt_sensor(
    trigger: On<Remove, JoltSensor>,
    sensor_query: Query<&JoltBodyId>,
    mut physics_world: ResMut<JoltPhysicsWorld>,
) {
    let unsensored_entity = trigger.event().entity;
    if let Ok(body_id) = sensor_query.get(unsensored_entity) {
        physics_world.set_body_sensor(body_id.body_id_raw, false);
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
    sensor_query: Query<(), With<JoltSensor>>,
    mut commands: Commands,
    mut physics_world: ResMut<JoltPhysicsWorld>,
) {
    for (sensor_entity, body_id) in &pending_query {
        // Marker removed while waiting: stale park, clear it without touching
        // Jolt. Same guard as `apply_pending_disables`.
        if sensor_query.contains(sensor_entity) {
            physics_world.set_body_sensor(body_id.body_id_raw, true);
        }
        commands.entity(sensor_entity).remove::<PendingSensor>();
    }
}

/// Drains Jolt's contact queues after the physics step and fires Bevy
/// events.
pub fn drain_contact_events(
    mut commands: Commands,
    mut physics_world: ResMut<JoltPhysicsWorld>,
    body_entities: Query<(Entity, &JoltBodyId)>,
) {
    let mut added_ids_first = [0u32; MAX_CONTACT_EVENTS];
    let mut added_ids_second = [0u32; MAX_CONTACT_EVENTS];
    let mut removed_ids_first = [0u32; MAX_CONTACT_EVENTS];
    let mut removed_ids_second = [0u32; MAX_CONTACT_EVENTS];
    let added_kept = physics_world.drain_contact_added(&mut added_ids_first, &mut added_ids_second);
    let removed_kept =
        physics_world.drain_contact_removed(&mut removed_ids_first, &mut removed_ids_second);
    // Most ticks touch nothing: skip the body pass and its map entirely.
    if added_kept == 0 && removed_kept == 0 {
        return;
    }
    // One pass over the bodies, then O(1) per pair. Each pair needs two ids,
    // so the old per-id scan of the whole query cost up to 2 * 2 *
    // MAX_CONTACT_EVENTS full scans of every body each fixed tick.
    let entity_of_body = crate::body_sync::body_entity_index(&body_entities);
    for pair_index in 0..added_kept as usize {
        // A body despawned between the step and this drain is legitimately
        // absent; skip the pair rather than reporting half of it.
        let (Some(first_entity), Some(second_entity)) = (
            entity_of_body.get(&added_ids_first[pair_index]),
            entity_of_body.get(&added_ids_second[pair_index]),
        ) else {
            continue;
        };
        commands.trigger(JoltContactAdded {
            first_entity: *first_entity,
            second_entity: *second_entity,
        });
    }
    for pair_index in 0..removed_kept as usize {
        let (Some(first_entity), Some(second_entity)) = (
            entity_of_body.get(&removed_ids_first[pair_index]),
            entity_of_body.get(&removed_ids_second[pair_index]),
        ) else {
            continue;
        };
        commands.trigger(JoltContactRemoved {
            first_entity: *first_entity,
            second_entity: *second_entity,
        });
    }
}
