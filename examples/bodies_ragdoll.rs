//! Ragdoll: an 11-part humanoid drops onto the floor, crumples, and
//! settles. Bodies render as physics wireframes; the console still prints
//! the rest span, then it exits.

use bevy::prelude::*;
use bevy_jolt::{JoltBody, JoltDebugPlugin, JoltPlugin, JoltRagdoll, JoltRagdollParts, JoltShape};

const SETTLE_TICKS: u32 = 400;

fn main() {
    App::new()
        .add_plugins(DefaultPlugins)
        .add_plugins(JoltPlugin::new().with_physics_hz(60.0))
        .add_plugins(JoltDebugPlugin)
        .add_systems(Startup, spawn_ragdoll_scene)
        .add_systems(FixedUpdate, watch_ragdoll_scene)
        .run();
}

#[derive(Resource)]
struct RagdollDemo {
    doll: Entity,
}
fn spawn_ragdoll_scene(mut commands: Commands) {
    commands.spawn((
        Camera3d::default(),
        Transform::from_xyz(0.0, 2.5, 6.0).looking_at(Vec3::new(0.0, 0.5, 0.0), Vec3::Y),
    ));
    commands.spawn((
        DirectionalLight::default(),
        Transform::from_xyz(3.0, 8.0, 5.0).looking_at(Vec3::ZERO, Vec3::Y),
    ));
    commands.spawn((
        Transform::from_xyz(0.0, -1.0, 0.0),
        JoltBody::fixed(0),
        JoltShape::box_shape(Vec3::new(100.0, 1.0, 100.0)),
    ));
    let doll = commands
        .spawn((
            Transform::from_xyz(0.0, 0.5, 0.0),
            JoltRagdoll::humanoid(0),
        ))
        .id();
    commands.insert_resource(RagdollDemo { doll });
}

fn watch_ragdoll_scene(
    mut tick_count: Local<u32>,
    demo: Res<RagdollDemo>,
    ragdoll_query: Query<&JoltRagdollParts>,
    transform_query: Query<&Transform>,
    mut app_exit: MessageWriter<AppExit>,
) {
    *tick_count += 1;
    if *tick_count < SETTLE_TICKS {
        return;
    }
    if *tick_count > SETTLE_TICKS {
        return;
    }
    let Ok(ragdoll_parts) = ragdoll_query.get(demo.doll) else {
        return;
    };
    assert_eq!(
        ragdoll_parts.part_entities.len(),
        11,
        "humanoid should bake 11 parts"
    );
    let mut lowest_y = f32::MAX;
    let mut highest_y = f32::MIN;
    for part_entity in &ragdoll_parts.part_entities {
        let Ok(part_pose) = transform_query.get(*part_entity) else {
            return;
        };
        lowest_y = lowest_y.min(part_pose.translation.y);
        highest_y = highest_y.max(part_pose.translation.y);
    }
    println!("ragdoll spans y={lowest_y:.3}..{highest_y:.3}");
    assert!(
        lowest_y < 0.3,
        "ragdoll should lie on the floor, lowest part at y={lowest_y:.3}"
    );
    assert!(
        highest_y - lowest_y < 1.2,
        "ragdoll should crumple flat, span is {:.3}",
        highest_y - lowest_y
    );
    println!("Ragdoll fell, folded, and settled.");
    app_exit.write(AppExit::Success);
}
