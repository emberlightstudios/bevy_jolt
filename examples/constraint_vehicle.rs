//! Vehicle: a tuned `VehicleSpec` car with WASD driving (W gas,
//! S brake/reverse, A/D steer) over bumpy heightfield terrain. No keys: sits
//! still. Wheels are render-only spheres parented to the chassis at the
//! spec's mount points: they yaw with steering and spin with throttle so the
//! drive state reads visually (Jolt wheels are raycasts, there are no wheel
//! bodies to follow).
//!
//! Speed tuning vs `VehicleSpec::default` (see commit message for old values):
//! higher engine torque and rev limit, later upshifts, quicker gear changes.

use bevy::asset::RenderAssetUsages;
use bevy::mesh::{Indices, PrimitiveTopology};
use bevy::prelude::*;
use bevy_jolt::{
    JoltBody, JoltDebugPlugin, JoltPlugin, JoltShape, JoltVehicleDrive, VehicleEngine,
    VehicleSpec,
};

const CAR_SPAWN: Vec3 = Vec3::new(0.0, 1.2, 0.0);
const TERRAIN_WIDTH: u32 = 32;
const TERRAIN_CELL: f32 = 2.0;
/// Bumps fade to flat inside this radius so the car spawns on stable ground.
const SPAWN_FLAT_RADIUS: f32 = 8.0;
const BUMP_AMPLITUDE: f32 = 0.9;
/// Visual wheel spin at full throttle (radians per second).
const WHEEL_SPIN_RATE: f32 = 14.0;

fn main() {
    App::new()
        .add_plugins(DefaultPlugins)
        .add_plugins(JoltPlugin::new().with_physics_hz(60.0))
        .add_plugins(JoltDebugPlugin)
        .add_systems(Startup, spawn_scene)
        .add_systems(PreUpdate, drive_car)
        .add_systems(Update, (spin_wheel_visuals, follow_car_camera))
        .run();
}

/// Rolling bumps, flat near the origin so the car spawns on stable ground.
fn terrain_height(world_x: f32, world_z: f32) -> f32 {
    let spawn_distance = world_x.hypot(world_z);
    let blend = ((spawn_distance - SPAWN_FLAT_RADIUS * 0.5) / (SPAWN_FLAT_RADIUS * 0.5))
        .clamp(0.0, 1.0);
    let smooth_blend = blend * blend * (3.0 - 2.0 * blend);
    ((world_x * 0.35).sin() + (world_z * 0.3).cos()) * BUMP_AMPLITUDE * smooth_blend
}

fn spawn_scene(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    commands.spawn((
        Camera3d::default(),
        Transform::from_xyz(0.0, 7.0, 14.0).looking_at(Vec3::new(0.0, 1.0, 0.0), Vec3::Y),
    ));
    commands.spawn((
        DirectionalLight {
            illuminance: 3000.0,
            ..default()
        },
        Transform::from_xyz(6.0, 12.0, 8.0).looking_at(Vec3::ZERO, Vec3::Y),
    ));
    // Bumpy heightfield: same samples drive physics and the render mesh.
    let grid_extent = (TERRAIN_WIDTH - 1) as f32 * TERRAIN_CELL;
    let mut field_heights = Vec::with_capacity((TERRAIN_WIDTH * TERRAIN_WIDTH) as usize);
    for grid_z in 0..TERRAIN_WIDTH {
        for grid_x in 0..TERRAIN_WIDTH {
            let world_x = grid_x as f32 * TERRAIN_CELL - grid_extent * 0.5;
            let world_z = grid_z as f32 * TERRAIN_CELL - grid_extent * 0.5;
            field_heights.push(terrain_height(world_x, world_z));
        }
    }
    commands.spawn((
        Mesh3d(meshes.add(terrain_mesh(&field_heights))),
        MeshMaterial3d(materials.add(Color::srgb(0.35, 0.42, 0.3))),
        Transform::IDENTITY,
        JoltBody::fixed(0),
        JoltShape::heightfield(field_heights, TERRAIN_WIDTH, TERRAIN_CELL),
    ));
    // Tuned car: stronger engine, higher rev limit, later upshifts, quicker
    // gear changes than `VehicleSpec::default`.
    let mut car_spec = VehicleSpec::new(0);
    car_spec.engine = VehicleEngine {
        max_torque: 900.0,
        max_rpm: 7000.0,
        ..VehicleEngine::default()
    };
    car_spec.transmission.shift_up_rpm = 5200.0;
    car_spec.transmission.switch_time = 0.25;
    let wheel_mounts: Vec<WheelMount> = car_spec
        .wheels
        .iter()
        .map(|spec_wheel| WheelMount {
            mount_position: spec_wheel.mount_position,
            wheel_radius: spec_wheel.wheel_radius,
            max_steer_angle: spec_wheel.max_steer_angle,
        })
        .collect();
    let car_entity = commands
        .spawn((
            Car,
            Mesh3d(meshes.add(Cuboid::new(1.8, 1.2, 4.4))),
            MeshMaterial3d(materials.add(Color::srgb(0.8, 0.15, 0.2))),
            Transform::from_translation(CAR_SPAWN),
            car_spec,
            JoltVehicleDrive {
                forward: 0.0,
                steer: 0.0,
                brake: 0.0,
                hand_brake: 0.0,
            },
        ))
        .id();
    // Render-only wheel spheres parented to the chassis at the spec's mount
    // points. Each carries a bright spoke child so the spin reads visually
    // (a plain sphere's rotation is invisible).
    let wheel_mesh = meshes.add(Sphere::new(1.0));
    let tire_material = materials.add(Color::srgb(0.08, 0.08, 0.1));
    let spoke_material = materials.add(Color::srgb(0.9, 0.85, 0.2));
    let mut wheel_visuals = Vec::with_capacity(wheel_mounts.len());
    for wheel_mount in &wheel_mounts {
        let spoke_half = wheel_mount.wheel_radius * 0.8;
        let wheel_entity = commands
            .spawn((
                Mesh3d(wheel_mesh.clone()),
                MeshMaterial3d(tire_material.clone()),
                Transform::from_translation(wheel_mount.mount_position)
                    .with_scale(Vec3::splat(wheel_mount.wheel_radius)),
                ChildOf(car_entity),
            ))
            .with_children(|wheel_children| {
                wheel_children.spawn((
                    Mesh3d(meshes.add(Cuboid::new(
                        1.3,
                        spoke_half * 2.0,
                        wheel_mount.wheel_radius * 0.35,
                    ))),
                    MeshMaterial3d(spoke_material.clone()),
                    // Spoke sized in unit-sphere space: the parent's radius
                    // scale brings it out to the tire tread.
                    Transform::IDENTITY,
                ));
            })
            .id();
        wheel_visuals.push(WheelVisual {
            wheel_entity,
            max_steer_angle: wheel_mount.max_steer_angle,
            spin_angle: 0.0,
        });
    }
    commands.insert_resource(WheelVisuals { wheel_visuals });
}

/// Render mesh matching the heightfield samples: grid quads with computed
/// normals so the bumps catch the light.
fn terrain_mesh(field_heights: &[f32]) -> Mesh {
    let grid_width = TERRAIN_WIDTH as usize;
    let grid_extent = (TERRAIN_WIDTH - 1) as f32 * TERRAIN_CELL;
    let mut mesh_positions = Vec::with_capacity(field_heights.len());
    for grid_z in 0..grid_width {
        for grid_x in 0..grid_width {
            mesh_positions.push([
                grid_x as f32 * TERRAIN_CELL - grid_extent * 0.5,
                field_heights[grid_z * grid_width + grid_x],
                grid_z as f32 * TERRAIN_CELL - grid_extent * 0.5,
            ]);
        }
    }
    let mut mesh_triangles = Vec::with_capacity((grid_width - 1) * (grid_width - 1) * 6);
    for grid_z in 0..grid_width - 1 {
        for grid_x in 0..grid_width - 1 {
            let corner = (grid_z * grid_width + grid_x) as u32;
            let edge = corner + 1;
            let below = corner + grid_width as u32;
            let diagonal = below + 1;
            mesh_triangles.extend([corner, below, edge, edge, below, diagonal]);
        }
    }
    Mesh::new(
        PrimitiveTopology::TriangleList,
        RenderAssetUsages::MAIN_WORLD | RenderAssetUsages::RENDER_WORLD,
    )
    .with_inserted_attribute(Mesh::ATTRIBUTE_POSITION, mesh_positions)
    .with_inserted_indices(Indices::U32(mesh_triangles))
    .with_computed_normals()
}

/// Marker for the driven car entity (the chase camera's target).
#[derive(Component)]
struct Car;

/// One wheel's chassis-space mount, copied out of the `VehicleSpec` before
/// spawn so the visuals match the simulated wheels.
struct WheelMount {
    mount_position: Vec3,
    wheel_radius: f32,
    max_steer_angle: f32,
}

/// One render-only wheel: yaw with steering, roll with throttle.
struct WheelVisual {
    wheel_entity: Entity,
    max_steer_angle: f32,
    spin_angle: f32,
}

#[derive(Resource)]
struct WheelVisuals {
    wheel_visuals: Vec<WheelVisual>,
}

/// WASD driving: W gas, S brake/reverse, A/D steer. Car sits still
/// with no keys held.
fn drive_car(keyboard: Res<ButtonInput<KeyCode>>, mut drive_query: Query<&mut JoltVehicleDrive>) {
    for mut drive in &mut drive_query {
        drive.forward = if keyboard.pressed(KeyCode::KeyW) {
            1.0
        } else if keyboard.pressed(KeyCode::KeyS) {
            -0.6
        } else {
            0.0
        };
        drive.steer = if keyboard.pressed(KeyCode::KeyA) {
            -0.6
        } else if keyboard.pressed(KeyCode::KeyD) {
            0.6
        } else {
            0.0
        };
    }
}

/// Yaws the wheel visuals with steering and rolls them with throttle. Reads
/// the same drive input the physics step consumes, so the visuals never
/// disagree with the sim.
fn spin_wheel_visuals(
    time: Res<Time>,
    drive_query: Query<&JoltVehicleDrive, With<Car>>,
    mut wheel_visuals: ResMut<WheelVisuals>,
    mut transform_query: Query<&mut Transform>,
) {
    let Ok(drive) = drive_query.single() else {
        // Car despawned mid-teardown: nothing to pose.
        return;
    };
    for wheel_visual in &mut wheel_visuals.wheel_visuals {
        wheel_visual.spin_angle += drive.forward * WHEEL_SPIN_RATE * time.delta_secs();
        let Ok(mut wheel_transform) = transform_query.get_mut(wheel_visual.wheel_entity) else {
            // Wheel despawned mid-teardown: skip it.
            continue;
        };
        let wheel_translation = wheel_transform.translation;
        let wheel_scale = wheel_transform.scale;
        *wheel_transform = Transform::from_translation(wheel_translation)
            .with_scale(wheel_scale)
            .with_rotation(
                Quat::from_rotation_y(drive.steer * wheel_visual.max_steer_angle)
                    * Quat::from_rotation_x(wheel_visual.spin_angle),
            );
    }
}

/// Chase camera: follows the car from behind/above so the speed reads.
fn follow_car_camera(
    car_query: Query<&Transform, With<Car>>,
    mut camera_query: Query<&mut Transform, (With<Camera3d>, Without<Car>)>,
) {
    let Ok(car_pose) = car_query.single() else {
        // Car not spawned yet: hold the opening framing.
        return;
    };
    let Ok(mut camera_pose) = camera_query.single_mut() else {
        // No camera (headless test): nothing to move.
        return;
    };
    let chase_target = car_pose.translation + Vec3::new(0.0, 7.0, 14.0);
    camera_pose.translation = camera_pose.translation.lerp(chase_target, 0.05);
    camera_pose.look_at(car_pose.translation + Vec3::Y, Vec3::Y);
}
