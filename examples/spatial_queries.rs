//! Spatial queries: one ray, one point, one overlap box, one sphere sweep
//! against a known stack of boxes. Prints every hit and exits.

use bevy::prelude::*;
use bevy_jolt::{JoltBody, JoltPhysicsWorld, JoltPlugin, JoltShape, QueryProbe};

const QUERY_TICKS: u32 = 120;

fn main() {
    App::new()
        .add_plugins(MinimalPlugins)
        .add_plugins(JoltPlugin::new().with_physics_hz(60.0))
        .add_systems(Startup, spawn_query_scene)
        .add_systems(FixedUpdate, run_queries)
        .run();
}

fn spawn_query_scene(mut commands: Commands) {
    commands.spawn((
        Transform::from_xyz(0.0, -1.0, 0.0),
        JoltBody::fixed(0),
        JoltShape::box_shape(Vec3::new(100.0, 1.0, 100.0)),
    ));
    // Three boxes in a row along x, resting on the floor.
    for stack_x in [-2.0, 0.0, 2.0] {
        commands.spawn((
            Transform::from_xyz(stack_x, 0.5, 0.0),
            JoltBody::dynamic(0),
            JoltShape::box_shape(Vec3::splat(0.5)),
        ));
    }
}

fn run_queries(
    mut tick_count: Local<u32>,
    physics_world: Res<JoltPhysicsWorld>,
    mut app_exit: MessageWriter<AppExit>,
) {
    *tick_count += 1;
    if *tick_count < QUERY_TICKS {
        return;
    }
    if *tick_count > QUERY_TICKS {
        return;
    }

    // Down the row: passes through all three boxes, then the floor.
    let row_hits = physics_world.cast_ray_all(Vec3::new(-6.0, 0.5, 0.0), Vec3::new(12.0, 0.0, 0.0));
    println!("ray down the row: {} hits", row_hits.len());
    for row_hit in &row_hits {
        println!(
            "  body={} fraction={:.3} contact=({:.2},{:.2},{:.2})",
            row_hit.hit_body_id,
            row_hit.hit_fraction,
            row_hit.hit_contact.x,
            row_hit.hit_contact.y,
            row_hit.hit_contact.z,
        );
    }
    assert!(row_hits.len() >= 3, "ray should cross all three boxes");

    // Straight down over the middle box: closest hit is its lid.
    let down_hit = physics_world
        .cast_ray(Vec3::new(0.0, 5.0, 0.0), Vec3::new(0.0, -10.0, 0.0))
        .expect("down ray should hit the middle box");
    println!(
        "down ray: body={} fraction={:.3} contact=({:.2},{:.2},{:.2})",
        down_hit.hit_body_id,
        down_hit.hit_fraction,
        down_hit.hit_contact.x,
        down_hit.hit_contact.y,
        down_hit.hit_contact.z,
    );
    assert!(
        (down_hit.hit_contact.y - 1.0).abs() < 0.05,
        "down ray should stop on the box lid"
    );

    // Point inside the middle box finds a body; point in thin air finds none.
    let inside_hits = physics_world.collide_point_all(Vec3::new(0.0, 0.5, 0.0));
    let air_hits = physics_world.collide_point_all(Vec3::new(0.0, 4.0, 0.0));
    println!(
        "point inside box: {} bodies, point in air: {} bodies",
        inside_hits.len(),
        air_hits.len()
    );
    assert!(!inside_hits.is_empty(), "point query should find the box");
    assert!(air_hits.is_empty(), "point query should miss thin air");

    // Fat box probe around the middle box touches it; sweep a small sphere
    // down the row and it taps all three.
    let overlap_probe = QueryProbe::Box {
        probe_half_extents: Vec3::splat(0.75),
    };
    let overlap_hits = physics_world.overlap_shape_all(overlap_probe, Vec3::new(0.0, 0.5, 0.0));
    println!("overlap box: {} bodies", overlap_hits.len());
    for overlap_hit in &overlap_hits {
        println!(
            "  body={} depth={:.3} contact=({:.2},{:.2},{:.2})",
            overlap_hit.hit_body_id,
            overlap_hit.penetration_depth,
            overlap_hit.hit_contact.x,
            overlap_hit.hit_contact.y,
            overlap_hit.hit_contact.z,
        );
    }
    assert!(!overlap_hits.is_empty(), "overlap should find the middle box");

    let sweep_probe = QueryProbe::Sphere { probe_radius: 0.25 };
    let sweep_hits = physics_world.cast_shape_all(
        sweep_probe,
        Vec3::new(-6.0, 0.5, 0.0),
        Vec3::new(12.0, 0.0, 0.0),
    );
    println!("sphere sweep down the row: {} hits", sweep_hits.len());
    for sweep_hit in &sweep_hits {
        println!(
            "  body={} fraction={:.3} contact=({:.2},{:.2},{:.2})",
            sweep_hit.hit_body_id,
            sweep_hit.hit_fraction,
            sweep_hit.hit_contact.x,
            sweep_hit.hit_contact.y,
            sweep_hit.hit_contact.z,
        );
    }
    assert!(sweep_hits.len() >= 3, "sweep should tap all three boxes");

    println!("All spatial queries behaved.");
    app_exit.write(AppExit::Success);
}
