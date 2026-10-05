//! Contact events: a ball drops through a sensor gate onto the floor.
//! Prints begin/end pairs and exits. The gate reports but never pushes.

use bevy::prelude::*;
use bevy_jolt::{
    JoltBody, JoltContactAdded, JoltContactRemoved, JoltPlugin, JoltSensor, JoltShape,
};

const SETTLE_TICKS: u32 = 300;

fn main() {
    App::new()
        .add_plugins(MinimalPlugins)
        .add_plugins(JoltPlugin::new().with_physics_hz(60.0))
        .insert_resource(ContactLog::default())
        .add_systems(Startup, spawn_contact_scene)
        .add_systems(FixedUpdate, watch_contact_scene)
        .add_observer(log_contact_added)
        .add_observer(log_contact_removed)
        .run();
}

#[derive(Resource)]
struct ContactDemo {
    ball: Entity,
    gate: Entity,
    floor: Entity,
}

fn spawn_contact_scene(mut commands: Commands) {
    let floor = commands
        .spawn((
            Transform::from_xyz(0.0, -1.0, 0.0),
            JoltBody::fixed(0),
            JoltShape::box_shape(Vec3::new(100.0, 1.0, 100.0)),
        ))
        .id();
    // Sensor gate across the ball's fall line: reports, never pushes.
    let gate = commands
        .spawn((
            Transform::from_xyz(0.0, 2.0, 0.0),
            JoltBody::fixed(0),
            JoltShape::box_shape(Vec3::new(2.0, 0.5, 2.0)),
            JoltSensor,
        ))
        .id();
    let ball = commands
        .spawn((
            Transform::from_xyz(0.0, 5.0, 0.0),
            JoltBody::dynamic(0),
            JoltShape::sphere(0.5),
        ))
        .id();
    commands.insert_resource(ContactDemo { ball, gate, floor });
}

#[derive(Resource, Default)]
struct ContactLog {
    ball_gate_begins: u32,
    ball_floor_begins: u32,
    total_begins: u32,
    total_ends: u32,
}

fn log_contact_added(
    trigger: On<JoltContactAdded>,
    demo: Res<ContactDemo>,
    mut contact_log: ResMut<ContactLog>,
) {
    let contact = trigger.event();
    contact_log.total_begins += 1;
    let ball_gate = (contact.first_entity == demo.ball && contact.second_entity == demo.gate)
        || (contact.first_entity == demo.gate && contact.second_entity == demo.ball);
    let ball_floor = (contact.first_entity == demo.ball && contact.second_entity == demo.floor)
        || (contact.first_entity == demo.floor && contact.second_entity == demo.ball);
    if ball_gate {
        contact_log.ball_gate_begins += 1;
        println!("begin: ball entered the sensor gate");
    }
    if ball_floor {
        contact_log.ball_floor_begins += 1;
        println!("begin: ball hit the floor");
    }
}

fn log_contact_removed(
    trigger: On<JoltContactRemoved>,
    demo: Res<ContactDemo>,
    mut contact_log: ResMut<ContactLog>,
) {
    let contact = trigger.event();
    contact_log.total_ends += 1;
    let ball_gate = (contact.first_entity == demo.ball && contact.second_entity == demo.gate)
        || (contact.first_entity == demo.gate && contact.second_entity == demo.ball);
    if ball_gate {
        println!("end: ball left the sensor gate");
    }
}

fn watch_contact_scene(
    mut tick_count: Local<u32>,
    demo: Res<ContactDemo>,
    transform_query: Query<&Transform>,
    contact_log: Res<ContactLog>,
    mut app_exit: MessageWriter<AppExit>,
) {
    *tick_count += 1;
    if *tick_count < SETTLE_TICKS {
        return;
    }
    if *tick_count > SETTLE_TICKS {
        return;
    }
    let Ok(ball_pose) = transform_query.get(demo.ball) else {
        return;
    };
    println!(
        "ball rests at y={:.3}, begins={} ends={}",
        ball_pose.translation.y, contact_log.total_begins, contact_log.total_ends
    );
    assert!(
        contact_log.ball_gate_begins >= 1,
        "sensor gate should report the fall-through"
    );
    assert!(
        contact_log.ball_floor_begins >= 1,
        "floor should report the landing"
    );
    // The gate never pushes: the ball falls straight through to the floor.
    assert!(
        (ball_pose.translation.y - 0.5).abs() < 0.1,
        "ball should rest on the floor (gate pushes nothing), got y={:.3}",
        ball_pose.translation.y
    );
    println!("Contacts reported; the sensor pushed nothing.");
    app_exit.write(AppExit::Success);
}
