//! CCD: a fast bullet with sweeping stops at a thin wall, one without
//! tunnels through. Prints both fates and exits.

use bevy::prelude::*;
use bevy_jolt::{JoltBody, JoltLinearVelocity, JoltPlugin, JoltShape};

const SETTLE_TICKS: u32 = 200;
const BULLET_SPEED: f32 = 400.0;

fn main() {
    App::new()
        .add_plugins(MinimalPlugins)
        .add_plugins(JoltPlugin::new().with_physics_hz(60.0))
        .insert_resource(CcdDemo::default())
        .add_systems(Startup, spawn_ccd_scene)
        .add_systems(FixedUpdate, watch_ccd_scene)
        .run();
}

#[derive(Resource, Default)]
struct CcdDemo {
    plain_bullet: Option<Entity>,
    ccd_bullet: Option<Entity>,
    fired: bool,
}

fn spawn_ccd_scene(mut commands: Commands, mut demo: ResMut<CcdDemo>) {
    // Thin wall between the guns and the backstop line.
    commands.spawn((
        Transform::from_xyz(0.0, 1.0, 0.0),
        JoltBody::fixed(0),
        JoltShape::box_shape(Vec3::new(0.05, 2.0, 5.0)),
    ));
    let plain_bullet = commands
        .spawn((
            Transform::from_xyz(-10.0, 1.0, -1.0),
            JoltBody::dynamic(0).with_gravity(0.0),
            JoltShape::sphere(0.15),
        ))
        .id();
    let ccd_bullet = commands
        .spawn((
            Transform::from_xyz(-10.0, 1.0, 1.0),
            JoltBody::dynamic(0).with_gravity(0.0).with_ccd(true),
            JoltShape::sphere(0.15),
        ))
        .id();
    demo.plain_bullet = Some(plain_bullet);
    demo.ccd_bullet = Some(ccd_bullet);
}

fn watch_ccd_scene(
    mut tick_count: Local<u32>,
    mut demo: ResMut<CcdDemo>,
    transform_query: Query<&Transform>,
    body_ids: Query<&bevy_jolt::JoltBodyId>,
    mut commands: Commands,
    mut app_exit: MessageWriter<AppExit>,
) {
    let (Some(plain_bullet), Some(ccd_bullet)) = (demo.plain_bullet, demo.ccd_bullet) else {
        return;
    };
    // Bodies bake a tick after spawn: wait for both ids so the drive write
    // lands after bake (bake zeroes the component, an earlier write would
    // be overwritten).
    if body_ids.get(plain_bullet).is_err() || body_ids.get(ccd_bullet).is_err() {
        return;
    }
    if !demo.fired {
        // One-shot drive through the unified component: the write lands
        // before the next step, then the body coasts (change detection only
        // pushes fresh writes, so nothing re-drives it).
        for bullet in [plain_bullet, ccd_bullet] {
            commands.entity(bullet).insert(JoltLinearVelocity {
                linear_velocity: Vec3::X * BULLET_SPEED,
            });
        }
        demo.fired = true;
        return;
    }
    *tick_count += 1;
    if *tick_count < SETTLE_TICKS {
        return;
    }
    if *tick_count > SETTLE_TICKS {
        return;
    }
    let Ok(plain_pose) = transform_query.get(plain_bullet) else {
        return;
    };
    let Ok(ccd_pose) = transform_query.get(ccd_bullet) else {
        return;
    };
    println!(
        "plain bullet x={:.3}, ccd bullet x={:.3} (wall at x=0)",
        plain_pose.translation.x, ccd_pose.translation.x
    );
    assert!(
        plain_pose.translation.x > 1.0,
        "plain bullet should tunnel through the thin wall, got x={:.3}",
        plain_pose.translation.x
    );
    assert!(
        ccd_pose.translation.x < 0.5,
        "ccd bullet should stop at the wall, got x={:.3}",
        ccd_pose.translation.x
    );
    println!("Swept bullet stops; discrete bullet tunnels.");
    app_exit.write(AppExit::Success);
}
