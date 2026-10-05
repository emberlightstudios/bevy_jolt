//! Compound colliders: a table (top + four legs) and a hammer (head +
//! handle) drop onto the floor and settle. Bodies render as physics
//! wireframes; the console still prints resting heights, then it exits.

use bevy::prelude::*;
use bevy_jolt::{CompoundGeometry, CompoundPart, JoltBody, JoltDebugPlugin, JoltPlugin, JoltShape};

const SETTLE_TICKS: u32 = 300;

fn main() {
    App::new()
        .add_plugins(DefaultPlugins)
        .add_plugins(JoltPlugin::new().with_physics_hz(60.0))
        .add_plugins(JoltDebugPlugin)
        .add_systems(Startup, spawn_compound_scene)
        .add_systems(FixedUpdate, watch_compound_scene)
        .run();
}

#[derive(Resource)]
struct CompoundDemo {
    table_body: Entity,
    hammer_body: Entity,
}

fn box_part(part_offset: Vec3, part_half_extents: Vec3) -> CompoundPart {
    CompoundPart {
        part_geometry: CompoundGeometry::Box { part_half_extents },
        part_offset,
        part_rotation: Quat::IDENTITY,
    }
}
fn spawn_compound_scene(mut commands: Commands) {
    commands.spawn((
        Camera3d::default(),
        Transform::from_xyz(0.0, 4.0, 10.0).looking_at(Vec3::new(0.0, 1.0, 0.0), Vec3::Y),
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
    // Table: 1.6 x 0.1 x 1.0 top at 0.75 plus four 0.1 x 0.7 x 0.1 legs.
    let mut table_parts = vec![box_part(
        Vec3::new(0.0, 0.75, 0.0),
        Vec3::new(0.8, 0.05, 0.5),
    )];
    for leg_x in [-0.7, 0.7] {
        for leg_z in [-0.4, 0.4] {
            table_parts.push(box_part(
                Vec3::new(leg_x, 0.35, leg_z),
                Vec3::new(0.05, 0.35, 0.05),
            ));
        }
    }
    let table_body = commands
        .spawn((
            Transform::from_xyz(-2.0, 2.0, 0.0),
            JoltBody::dynamic(0),
            JoltShape::compound(table_parts),
        ))
        .id();
    // Hammer: 0.5 x 0.2 x 0.2 head on a 0.12 x 0.8 x 0.12 handle.
    let hammer_body = commands
        .spawn((
            Transform::from_xyz(2.0, 2.0, 0.0),
            JoltBody::dynamic(0),
            JoltShape::compound(vec![
                box_part(Vec3::new(0.0, 0.5, 0.0), Vec3::new(0.25, 0.1, 0.1)),
                box_part(Vec3::new(0.0, 0.0, 0.0), Vec3::new(0.06, 0.4, 0.06)),
            ]),
        ))
        .id();
    commands.insert_resource(CompoundDemo {
        table_body,
        hammer_body,
    });
}

fn watch_compound_scene(
    mut tick_count: Local<u32>,
    demo: Res<CompoundDemo>,
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
    let Ok(table_pose) = transform_query.get(demo.table_body) else {
        return;
    };
    let Ok(hammer_pose) = transform_query.get(demo.hammer_body) else {
        return;
    };
    println!(
        "table origin at y={:.3}, hammer origin at y={:.3}",
        table_pose.translation.y, hammer_pose.translation.y
    );
    // Table legs span origin+0 to origin+0.7: the origin rests at 0. Hammer
    // lies on its side (handle 0.06 half width) or stands on the handle end
    // (0.4): either way well below the 2.0 spawn.
    assert!(
        table_pose.translation.y.abs() < 0.1,
        "table should stand on its legs, got y={:.3}",
        table_pose.translation.y
    );
    assert!(
        hammer_pose.translation.y < 1.0,
        "hammer should lie on the floor, got y={:.3}",
        hammer_pose.translation.y
    );
    println!("Compounds settled on their parts, not their origins.");
    app_exit.write(AppExit::Success);
}
