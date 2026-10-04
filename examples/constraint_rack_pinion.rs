//! Rack-and-pinion (jack) joint: a spinning pinion drives a sliding rack.
//! Alternating kicks keep the jack pumping up and down.
use bevy::prelude::*;
use bevy_jolt::{
    CollisionLayers, JoltBody, JoltDebugPlugin, JoltImpulse, JoltJoint, JoltJointId, JoltPlugin,
    JoltShape, JointSpace,
};

const PINION_POS: Vec3 = Vec3::new(0.0, 3.0, 0.0);
const RACK_POS: Vec3 = Vec3::new(0.7, 4.6, 0.0);
const KICK_EVERY_N_TICKS: u32 = 100;
const KICK_IMPULSE: f32 = 800.0;

fn main() {
    App::new()
        .add_plugins(DefaultPlugins)
        .add_plugins(JoltPlugin::new().with_physics_hz(60.0))
        .add_plugins(JoltDebugPlugin)
        .add_systems(Startup, spawn_scene)
        .add_systems(FixedUpdate, kick_rack)
        .add_systems(PostUpdate, draw_link)
        .run();
}

#[derive(Resource)]
struct Demo {
    rack: Entity,
    joint: Entity,
    kick_in: u32,
    push_down: bool,
}

fn spawn_scene(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    commands.spawn((
        Camera3d::default(),
        Transform::from_xyz(5.0, 3.5, 10.0).looking_at(Vec3::new(0.0, 3.0, 0.0), Dir3::Y),
    ));
    commands.spawn((
        DirectionalLight {
            illuminance: 3000.0,
            ..default()
        },
        Transform::from_xyz(4.0, 8.0, 6.0).looking_at(Vec3::ZERO, Vec3::Y),
    ));
    commands.spawn((
        Mesh3d(meshes.add(Cuboid::new(200.0, 2.0, 200.0))),
        MeshMaterial3d(materials.add(Color::srgb(0.35, 0.35, 0.38))),
        Transform::from_xyz(0.0, -1.0, 0.0),
        JoltBody::fixed(CollisionLayers::NON_MOVING), JoltShape::box_shape(Vec3::new(100.0, 1.0, 100.0)),
    ));
    commands.spawn((
        Text::new("rack-and-pinion (jack)"),
        Node {
            position_type: PositionType::Absolute,
            top: Val::Px(12.0),
            left: Val::Px(16.0),
            ..default()
        },
    ));

    // Invisible anchors: the hinge and slider each need a static side, but no
    // mount geometry. The pinion spins about its own center on Z so the disc
    // faces the camera; the rack rides alongside it with teeth meshed.
    let face_rotation = Quat::from_rotation_x(core::f32::consts::FRAC_PI_2);
    let hinge_anchor = commands
        .spawn((
            Transform::from_translation(PINION_POS),
            JoltBody::fixed(CollisionLayers::MOVING),
            JoltShape::box_shape(Vec3::splat(0.1)),
        ))
        .id();
    let pinion = commands
        .spawn((
            Mesh3d(meshes.add(Cylinder::new(0.5, 0.2))),
            MeshMaterial3d(materials.add(Color::srgb(0.8, 0.6, 0.2))),
            Transform::from_translation(PINION_POS).with_rotation(face_rotation),
            JoltBody::dynamic(CollisionLayers::MOVING),
            JoltShape::cylinder(0.1, 0.5),
            children![(
                Mesh3d(meshes.add(Cuboid::new(0.12, 0.9, 0.22))),
                MeshMaterial3d(materials.add(Color::srgb(0.15, 0.15, 0.2))),
                Transform::IDENTITY,
            )],
        ))
        .id();
    let slider_anchor = commands
        .spawn((
            Transform::from_translation(RACK_POS + Vec3::new(0.0, 1.5, 0.)),
            JoltBody::fixed(CollisionLayers::MOVING),
            JoltShape::box_shape(Vec3::splat(0.1)),
        ))
        .id();
    let rack = commands
        .spawn((
            Mesh3d(meshes.add(Cuboid::new(0.25, 1.8, 0.25))),
            MeshMaterial3d(materials.add(Color::srgb(0.3, 0.6, 0.9))),
            Transform::from_translation(RACK_POS),
            JoltBody::dynamic(CollisionLayers::MOVING),
            JoltShape::box_shape(Vec3::new(0.125, 0.9, 0.125)),
        ))
        .id();
    let hinge = commands
        .spawn(JoltJoint::hinge(
            hinge_anchor,
            pinion,
            PINION_POS,
            Dir3::Z,
            Dir3::X,
            JointSpace::World,
        ))
        .id();
    let slider = commands
        .spawn(JoltJoint::slider(
            slider_anchor,
            rack,
            Dir3::Y,
            Dir3::X,
            -1.5,
            0.5,
            JointSpace::World,
        ))
        .id();
    let joint = commands
        .spawn(JoltJoint::rack_pinion(
            pinion,
            rack,
            Dir3::Z,
            Dir3::Y,
            1.0,
            hinge,
            slider,
            JointSpace::World,
        ))
        .id();
    commands.insert_resource(Demo {
        rack,
        joint,
        kick_in: 60,
        push_down: false,
    });
}

fn kick_rack(mut demo: ResMut<Demo>, joint_query: Query<(), With<JoltJointId>>, mut commands: Commands) {
    if joint_query.get(demo.joint).is_err() {
        return;
    }
    demo.kick_in = demo.kick_in.saturating_sub(1);
    if demo.kick_in == 0 {
        let push = if demo.push_down {
            Vec3::new(0.0, -KICK_IMPULSE, 0.0)
        } else {
            Vec3::new(0.0, KICK_IMPULSE, 0.0)
        };
        commands.trigger(JoltImpulse::linear(demo.rack, push));
        demo.push_down = !demo.push_down;
        demo.kick_in = KICK_EVERY_N_TICKS;
    }
}

fn draw_link(
    demo: Res<Demo>,
    transform_query: Query<&Transform>,
    joint_query: Query<(), With<JoltJointId>>,
    mut gizmos: Gizmos,
) {
    if joint_query.get(demo.joint).is_err() {
        return;
    }
    let Ok(rack) = transform_query.get(demo.rack) else {
        return;
    };
    gizmos.line(
        rack.translation + Vec3::new(0.0, 0.8, 0.0),
        rack.translation - Vec3::new(0.0, 0.8, 0.0),
        Color::srgb(0.6, 0.9, 1.0),
    );
}
