//! Repro: looping path cart — logs position + rotation over time.
use bevy::prelude::*;
use bevy_jolt::{
    CollisionLayers, JoltBody, JoltJoint, JoltMotorDrive, JoltPhysicsWorld, JoltPlugin, JoltShape,
    JointMotor, JointSpace, PathRotation,
};
const WAYPOINTS: [Vec3; 6] = [
    Vec3::new(-3.0, 3.5, 0.0),
    Vec3::new(-1.5, 3.5, 1.5),
    Vec3::new(1.5, 3.5, 1.5),
    Vec3::new(3.0, 3.5, 0.0),
    Vec3::new(1.5, 3.5, -1.5),
    Vec3::new(-1.5, 3.5, -1.5),
];

fn tick(app: &mut App) {
    // Bare `app.update()` never advances the wall clock, so `Time<Fixed>`
    // sits at dt=0 and physics never steps. Advance the fixed clock by one
    // tick manually, then run FixedUpdate explicitly.
    let tick_delta = app.world().resource::<Time<Fixed>>().timestep();
    app.world_mut()
        .resource_mut::<Time<Fixed>>()
        .advance_by(tick_delta);
    app.update();
    app.world_mut().run_schedule(FixedUpdate);
}

#[test]
fn looping_path_cart_probe() {
    let mut app = App::new();
    app.add_plugins((MinimalPlugins, JoltPlugin::new()));
    let anchor = app
        .world_mut()
        .spawn((
            Transform::from_xyz(0.0, 3.5, 0.0),
            JoltBody::fixed(CollisionLayers::MOVING),
            JoltShape::box_shape(Vec3::splat(0.2)),
        ))
        .id();
    let start_waypoint: usize = std::env::var("PROBE_START")
        .ok()
        .and_then(|start_text| start_text.parse().ok())
        .unwrap_or(0);
    let cart = app
        .world_mut()
        .spawn((
            Transform::from_translation(WAYPOINTS[start_waypoint % WAYPOINTS.len()]),
            JoltBody::dynamic(CollisionLayers::MOVING),
            JoltShape::box_shape(Vec3::splat(0.35)),
        ))
        .id();
    let joint_motor = JointMotor {
        frequency_hz: 8.0,
        damping: 1.0,
        force_limit: 1.0e6,
    };
    app.world_mut().spawn((
        JoltJoint::path_waypoints(
            anchor,
            cart,
            &WAYPOINTS,
            true,
            joint_motor,
            PathRotation::ToPath,
            JointSpace::World,
        ),
        JoltMotorDrive(2.0),
    ));
    for step in 0..600 {
        tick(&mut app);
        if step % 60 == 0 || (step >= 380 && step <= 430) {
            let cart_translation = app
                .world()
                .get::<Transform>(cart)
                .expect("cart pose")
                .translation;
            let path_fraction = app
                .world_mut()
                .resource_mut::<JoltPhysicsWorld>()
                .constraint_path_fraction(1);
            let path_looping = app
                .world_mut()
                .resource_mut::<JoltPhysicsWorld>()
                .constraint_path_looping(1);
            println!(
                "tick {step}: cart=({:.3},{:.3},{:.3}) frac={path_fraction:.3} looping={path_looping}",
                cart_translation.x, cart_translation.y, cart_translation.z,
            );
        }
    }
}
