//! Cooked shapes beside their compounds: a table and its shrink-wrapped hull
//! twin stand side by side (same stance, filled void), a hammer drops while
//! its exact-triangle mesh twin stands as a wireframe statue. Console prints
//! resting poses, then it exits.

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
    hull_table_body: Entity,
    hammer_body: Entity,
    mesh_hammer_body: Entity,
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
    // Compound table: 1.6 x 0.1 x 1.0 top at 0.75 plus four legs. The
    // original: hollow under-top a ball could roll through.
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
            Transform::from_xyz(-3.5, 2.0, 0.0),
            JoltBody::dynamic(0),
            JoltShape::compound(table_parts),
        ))
        .id();
    // Hull table: same corners shrink-wrapped. Feet are hull extremes so it
    // stands at 0 like the compound; the void inside is filled.
    let table_hull_points = vec![
        Vec3::new(-0.8, 0.7, -0.5),
        Vec3::new(0.8, 0.7, -0.5),
        Vec3::new(0.8, 0.7, 0.5),
        Vec3::new(-0.8, 0.7, 0.5),
        Vec3::new(-0.8, 0.8, -0.5),
        Vec3::new(0.8, 0.8, -0.5),
        Vec3::new(0.8, 0.8, 0.5),
        Vec3::new(-0.8, 0.8, 0.5),
        Vec3::new(-0.7, 0.0, -0.4),
        Vec3::new(0.7, 0.0, -0.4),
        Vec3::new(0.7, 0.0, 0.4),
        Vec3::new(-0.7, 0.0, 0.4),
    ];
    let hull_table_body = commands
        .spawn((
            Transform::from_xyz(-1.0, 2.0, 0.0),
            JoltBody::dynamic(0),
            JoltShape::hull(table_hull_points),
        ))
        .id();
    // Compound hammer: head on a handle, drops and lies on the floor.
    let hammer_body = commands
        .spawn((
            Transform::from_xyz(1.5, 2.0, 0.0),
            JoltBody::dynamic(0),
            JoltShape::compound(vec![
                box_part(Vec3::new(0.0, 0.5, 0.0), Vec3::new(0.25, 0.1, 0.1)),
                box_part(Vec3::new(0.0, 0.0, 0.0), Vec3::new(0.06, 0.4, 0.06)),
            ]),
        ))
        .id();
    // Mesh hammer: same head + handle as exact triangles. Static only, so a
    // wireframe statue beside the dropped compound.
    let (hammer_vertices, hammer_triangles) = hammer_mesh();
    let mesh_hammer_body = commands
        .spawn((
            Transform::from_xyz(3.5, 0.4, 0.0),
            JoltBody::fixed(0),
            JoltShape::mesh(hammer_vertices, hammer_triangles),
        ))
        .id();
    commands.insert_resource(CompoundDemo {
        table_body,
        hull_table_body,
        hammer_body,
        mesh_hammer_body,
    });
}

/// 12 outward-facing triangles per box, corners listed low-to-high.
fn box_faces(box_min: Vec3, box_max: Vec3) -> (Vec<Vec3>, Vec<[u32; 3]>) {
    let box_corners = vec![
        Vec3::new(box_min.x, box_min.y, box_min.z), // 0
        Vec3::new(box_max.x, box_min.y, box_min.z), // 1
        Vec3::new(box_max.x, box_min.y, box_max.z), // 2
        Vec3::new(box_min.x, box_min.y, box_max.z), // 3
        Vec3::new(box_min.x, box_max.y, box_min.z), // 4
        Vec3::new(box_max.x, box_max.y, box_min.z), // 5
        Vec3::new(box_max.x, box_max.y, box_max.z), // 6
        Vec3::new(box_min.x, box_max.y, box_max.z), // 7
    ];
    let box_triangles = vec![
        [0, 2, 1], [0, 3, 2], // bottom (down-facing)
        [4, 5, 6], [4, 6, 7], // top (up-facing)
        [0, 1, 5], [0, 5, 4], // back
        [3, 6, 2], [3, 7, 6], // front
        [0, 4, 7], [0, 7, 3], // left
        [1, 2, 6], [1, 6, 5], // right
    ];
    (box_corners, box_triangles)
}

/// Hammer head + handle as one triangle soup: head box offset up, handle
/// box below it. Indices shifted for the second box.
fn hammer_mesh() -> (Vec<Vec3>, Vec<[u32; 3]>) {
    let (head_vertices, head_triangles) = box_faces(
        Vec3::new(-0.25, 0.4, -0.1),
        Vec3::new(0.25, 0.6, 0.1),
    );
    let (handle_vertices, handle_triangles) = box_faces(
        Vec3::new(-0.06, -0.4, -0.06),
        Vec3::new(0.06, 0.4, 0.06),
    );
    let mut hammer_vertices = head_vertices;
    let handle_base = hammer_vertices.len() as u32;
    hammer_vertices.extend(handle_vertices);
    let mut hammer_triangles = head_triangles;
    hammer_triangles.extend(
        handle_triangles
            .iter()
            .map(|handle_triangle| {
                [
                    handle_triangle[0] + handle_base,
                    handle_triangle[1] + handle_base,
                    handle_triangle[2] + handle_base,
                ]
            }),
    );
    (hammer_vertices, hammer_triangles)
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
    let Ok(hull_table_pose) = transform_query.get(demo.hull_table_body) else {
        return;
    };
    let Ok(hammer_pose) = transform_query.get(demo.hammer_body) else {
        return;
    };
    let Ok(mesh_hammer_pose) = transform_query.get(demo.mesh_hammer_body) else {
        return;
    };
    println!(
        "compound table y={:.3}, hull table y={:.3}, hammer y={:.3}, mesh hammer y={:.3}",
        table_pose.translation.y,
        hull_table_pose.translation.y,
        hammer_pose.translation.y,
        mesh_hammer_pose.translation.y
    );
    // Both tables stand on feet at 0: the compound on its legs, the hull on
    // its feet points (void filled inside, same stance outside).
    assert!(
        table_pose.translation.y.abs() < 0.1,
        "compound table should stand on its legs, got y={:.3}",
        table_pose.translation.y
    );
    assert!(
        hull_table_pose.translation.y.abs() < 0.1,
        "hull table should stand on its feet points, got y={:.3}",
        hull_table_pose.translation.y
    );
    // Compound hammer lies below its 2.0 spawn. Mesh hammer is static at
    // its (3.5, 0.4, 0) spawn pose.
    assert!(
        hammer_pose.translation.y < 1.0,
        "hammer should lie on the floor, got y={:.3}",
        hammer_pose.translation.y
    );
    assert!(
        (mesh_hammer_pose.translation - Vec3::new(3.5, 0.4, 0.0)).length() < 0.01,
        "mesh hammer should stand still, got {:?}",
        mesh_hammer_pose.translation
    );
    println!("Both tables stand; hammer lies down, mesh hammer stands.");
    app_exit.write(AppExit::Success);
}
