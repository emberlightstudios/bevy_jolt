//! Full vehicle authoring: chassis, wheels, engine, gearbox, and drive.
//!
//! Game code spawns an entity with [`JoltVehicle`] (a [`VehicleSpec`] plus
//! the spawn `Transform`); the plugin builds the Jolt chassis body plus
//! wheels once, stores the resulting [`JoltVehicleId`], and destroys both
//! when the entity leaves. Held input lives as [`JoltVehicleDrive`] on the
//! same entity: attach once, then change its fields to drive. The sync
//! system pushes it into Jolt before the physics step, so gameplay never
//! touches the world directly.
//!
//! Three controller kinds: [`VehicleKind::Wheeled`] (cars, slip-curve
//! tires), [`VehicleKind::Tracked`] (tanks, per-track multipliers instead
//! of steering), [`VehicleKind::Motorcycle`] (wheeled pair plus a
//! lean-spring balance controller). Manual gearboxes take a
//! [`JoltVehicleShift`]: gear plus clutch, pushed like drive input.

use bevy::prelude::*;
use std::f32::consts::PI;

use crate::plugin::JoltPhysicsWorld;
use jolt_sys::{
    MAX_VEHICLE_GEARS, MAX_VEHICLE_WHEELS, VehicleDifferentialFfi, VehicleEngineFfi, VehicleLeanFfi,
    VehicleRollBarFfi, VehicleTrackFfi, VehicleTransmissionFfi, VehicleWheelFfi,
};

/// One wheel: mount + suspension + tire. Friction curves stay Jolt defaults
/// (tire profile); tracked wheels use the plain friction pair instead.
#[derive(Clone, Copy, Debug)]
pub struct VehicleWheel {
    /// Suspension attachment in chassis space (meters).
    pub mount_position: Vec3,
    /// Where tire forces apply; defaults to the mount point.
    pub force_point: Vec3,
    /// Suspension travel band (meters).
    pub suspension_min_length: f32,
    pub suspension_max_length: f32,
    pub suspension_preload_length: f32,
    /// Suspension spring: frequency (Hz) + damping ratio.
    pub suspension_frequency: f32,
    pub suspension_damping: f32,
    pub wheel_radius: f32,
    pub wheel_width: f32,
    /// Max steer (radians). Zero for non-steered wheels.
    pub max_steer_angle: f32,
    /// Spin inertia (kg m^2) + angular damping.
    pub wheel_inertia: f32,
    pub wheel_damping: f32,
    pub max_brake_torque: f32,
    pub max_hand_brake_torque: f32,
    /// Tracked wheels only: plain friction pair.
    pub track_longitudinal_friction: f32,
    pub track_lateral_friction: f32,
    /// False = slip-curve car tire, true = tracked friction pair.
    pub tracked: bool,
}

impl Default for VehicleWheel {
    fn default() -> Self {
        Self {
            mount_position: Vec3::ZERO,
            force_point: Vec3::ZERO,
            suspension_min_length: 0.3,
            suspension_max_length: 0.5,
            suspension_preload_length: 0.0,
            suspension_frequency: 1.5,
            suspension_damping: 0.5,
            wheel_radius: 0.3,
            wheel_width: 0.1,
            max_steer_angle: 0.0,
            wheel_inertia: 0.9,
            wheel_damping: 0.2,
            max_brake_torque: 1500.0,
            max_hand_brake_torque: 4000.0,
            track_longitudinal_friction: 4.0,
            track_lateral_friction: 2.0,
            tracked: false,
        }
    }
}

impl VehicleWheel {
    pub(crate) fn to_ffi(self) -> VehicleWheelFfi {
        VehicleWheelFfi {
            pos_x: self.mount_position.x,
            pos_y: self.mount_position.y,
            pos_z: self.mount_position.z,
            susp_force_x: self.force_point.x,
            susp_force_y: self.force_point.y,
            susp_force_z: self.force_point.z,
            susp_min_length: self.suspension_min_length,
            susp_max_length: self.suspension_max_length,
            susp_preload_length: self.suspension_preload_length,
            susp_frequency: self.suspension_frequency,
            susp_damping: self.suspension_damping,
            wheel_radius: self.wheel_radius,
            wheel_width: self.wheel_width,
            max_steer_angle_rad: self.max_steer_angle,
            wheel_inertia: self.wheel_inertia,
            wheel_damping: self.wheel_damping,
            max_brake_torque: self.max_brake_torque,
            max_hand_brake_torque: self.max_hand_brake_torque,
            track_longitudinal_friction: self.track_longitudinal_friction,
            track_lateral_friction: self.track_lateral_friction,
            kind: u8::from(self.tracked),
        }
    }
}

/// Engine: torque + rev band + spin. Torque curve stays Jolt default
/// (0.8 / 1.0 / 0.8 across the rev fraction).
#[derive(Clone, Copy, Debug)]
pub struct VehicleEngine {
    pub max_torque: f32,
    pub min_rpm: f32,
    pub max_rpm: f32,
    pub engine_inertia: f32,
    pub engine_damping: f32,
}

impl Default for VehicleEngine {
    fn default() -> Self {
        Self {
            max_torque: 500.0,
            min_rpm: 1000.0,
            max_rpm: 6000.0,
            engine_inertia: 0.5,
            engine_damping: 0.2,
        }
    }
}

impl VehicleEngine {
    pub(crate) fn to_ffi(self) -> VehicleEngineFfi {
        VehicleEngineFfi {
            max_torque: self.max_torque,
            min_rpm: self.min_rpm,
            max_rpm: self.max_rpm,
            engine_inertia: self.engine_inertia,
            engine_damping: self.engine_damping,
        }
    }
}

/// Gearbox: auto or manual plus ratio tables and auto-shift tuning.
#[derive(Clone, Debug)]
pub struct VehicleTransmission {
    pub auto_mode: bool,
    pub gear_ratios: Vec<f32>,
    pub reverse_gear_ratios: Vec<f32>,
    pub switch_time: f32,
    pub clutch_release_time: f32,
    pub switch_latency: f32,
    pub shift_up_rpm: f32,
    pub shift_down_rpm: f32,
    pub clutch_strength: f32,
}

impl Default for VehicleTransmission {
    fn default() -> Self {
        Self {
            auto_mode: true,
            gear_ratios: vec![2.66, 1.78, 1.3, 1.0, 0.74],
            reverse_gear_ratios: vec![-2.90],
            switch_time: 0.5,
            clutch_release_time: 0.3,
            switch_latency: 0.5,
            shift_up_rpm: 4000.0,
            shift_down_rpm: 2000.0,
            clutch_strength: 10.0,
        }
    }
}

impl VehicleTransmission {
    pub(crate) fn to_ffi(&self) -> VehicleTransmissionFfi {
        let mut gear_ratios = [0.0; MAX_VEHICLE_GEARS];
        let mut reverse_gear_ratios = [0.0; MAX_VEHICLE_GEARS];
        let gear_count = self.gear_ratios.len().min(MAX_VEHICLE_GEARS);
        gear_ratios[..gear_count].copy_from_slice(&self.gear_ratios[..gear_count]);
        let reverse_count = self
            .reverse_gear_ratios
            .len()
            .min(MAX_VEHICLE_GEARS);
        reverse_gear_ratios[..reverse_count]
            .copy_from_slice(&self.reverse_gear_ratios[..reverse_count]);
        VehicleTransmissionFfi {
            auto_mode: u8::from(self.auto_mode),
            gear_count: gear_count as u8,
            reverse_gear_count: reverse_count as u8,
            gear_ratios,
            reverse_gear_ratios,
            switch_time: self.switch_time,
            clutch_release_time: self.clutch_release_time,
            switch_latency: self.switch_latency,
            shift_up_rpm: self.shift_up_rpm,
            shift_down_rpm: self.shift_down_rpm,
            clutch_strength: self.clutch_strength,
        }
    }
}

/// One differential: which wheels share torque and how it splits.
#[derive(Clone, Copy, Debug)]
pub struct VehicleDifferential {
    pub left_wheel: i32,
    pub right_wheel: i32,
    pub differential_ratio: f32,
    pub left_right_split: f32,
    pub limited_slip_ratio: f32,
    pub engine_torque_ratio: f32,
}

impl Default for VehicleDifferential {
    fn default() -> Self {
        Self {
            left_wheel: -1,
            right_wheel: -1,
            differential_ratio: 3.42,
            left_right_split: 0.5,
            limited_slip_ratio: 1.4,
            engine_torque_ratio: 1.0,
        }
    }
}

impl VehicleDifferential {
    pub(crate) fn to_ffi(self) -> VehicleDifferentialFfi {
        VehicleDifferentialFfi {
            left_wheel: self.left_wheel,
            right_wheel: self.right_wheel,
            differential_ratio: self.differential_ratio,
            left_right_split: self.left_right_split,
            limited_slip_ratio: self.limited_slip_ratio,
            engine_torque_ratio: self.engine_torque_ratio,
        }
    }
}

/// Anti-roll bar between a wheel pair: stiffness in N/m, 0 disables.
#[derive(Clone, Copy, Debug)]
pub struct VehicleRollBar {
    pub left_wheel: i32,
    pub right_wheel: i32,
    pub stiffness: f32,
}

impl Default for VehicleRollBar {
    fn default() -> Self {
        Self {
            left_wheel: 0,
            right_wheel: 1,
            stiffness: 1000.0,
        }
    }
}

impl VehicleRollBar {
    pub(crate) fn to_ffi(self) -> VehicleRollBarFfi {
        VehicleRollBarFfi {
            left_wheel: self.left_wheel,
            right_wheel: self.right_wheel,
            stiffness: self.stiffness,
        }
    }
}

/// One tank track side: member wheel indices (into the spec's wheel list)
/// plus which member is engine-driven (index into `wheel_indices`).
#[derive(Clone, Debug)]
pub struct VehicleTrack {
    pub wheel_indices: Vec<u8>,
    pub driven_wheel: u8,
    pub track_inertia: f32,
    pub track_damping: f32,
    pub max_brake_torque: f32,
    pub differential_ratio: f32,
}

impl Default for VehicleTrack {
    fn default() -> Self {
        Self {
            wheel_indices: Vec::new(),
            driven_wheel: 0,
            track_inertia: 10.0,
            track_damping: 0.5,
            max_brake_torque: 15000.0,
            differential_ratio: 6.0,
        }
    }
}

impl VehicleTrack {
    pub(crate) fn to_ffi(&self) -> VehicleTrackFfi {
        let mut wheel_indices = [0u8; MAX_VEHICLE_WHEELS];
        let count = self.wheel_indices.len().min(MAX_VEHICLE_WHEELS);
        wheel_indices[..count].copy_from_slice(&self.wheel_indices[..count]);
        VehicleTrackFfi {
            wheel_count: count as u8,
            wheel_indices,
            driven_wheel: self.driven_wheel,
            track_inertia: self.track_inertia,
            track_damping: self.track_damping,
            max_brake_torque: self.max_brake_torque,
            differential_ratio: self.differential_ratio,
        }
    }
}

/// Motorcycle lean spring: balance PID + smoothing. Jolt flags the
/// controller as still in development: expect tuning.
#[derive(Clone, Copy, Debug)]
pub struct VehicleLean {
    pub max_lean_angle: f32,
    pub lean_spring_constant: f32,
    pub lean_spring_damping: f32,
    pub lean_integration_coefficient: f32,
    pub lean_integration_decay: f32,
    pub lean_smoothing: f32,
}

impl Default for VehicleLean {
    fn default() -> Self {
        Self {
            max_lean_angle: 45.0f32.to_radians(),
            lean_spring_constant: 5000.0,
            lean_spring_damping: 1000.0,
            lean_integration_coefficient: 0.0,
            lean_integration_decay: 4.0,
            lean_smoothing: 0.8,
        }
    }
}

impl VehicleLean {
    pub(crate) fn to_ffi(self) -> VehicleLeanFfi {
        VehicleLeanFfi {
            max_lean_angle_rad: self.max_lean_angle,
            lean_spring_constant: self.lean_spring_constant,
            lean_spring_damping: self.lean_spring_damping,
            lean_integration_coefficient: self.lean_integration_coefficient,
            lean_integration_decay: self.lean_integration_decay,
            lean_smoothing: self.lean_smoothing,
        }
    }
}

/// Controller-specific tuning: wheeled (differentials + roll bars),
/// tracked (two track sides), or motorcycle (one diff set + lean spring).
#[derive(Clone, Debug)]
pub enum VehicleKind {
    Wheeled {
        differentials: Vec<VehicleDifferential>,
        roll_bars: Vec<VehicleRollBar>,
        limited_slip_ratio: f32,
    },
    Tracked {
        tracks: [VehicleTrack; 2],
    },
    Motorcycle {
        differentials: Vec<VehicleDifferential>,
        lean: VehicleLean,
    },
}

impl Default for VehicleKind {
    fn default() -> Self {
        Self::Wheeled {
            differentials: vec![VehicleDifferential {
                left_wheel: 0,
                right_wheel: 1,
                ..VehicleDifferential::default()
            }],
            roll_bars: Vec::new(),
            limited_slip_ratio: 1.4,
        }
    }
}

/// Full vehicle spec: chassis box + wheels + powertrain. Body frame X
/// right, Y up, Z forward (matches Jolt's vehicle sample). Creation reads
/// the entity's `Transform` as the spawn pose.
#[derive(Component, Clone, Debug)]
pub struct VehicleSpec {
    pub object_layer: u16,
    /// Chassis half extents (meters).
    pub half_extents: Vec3,
    /// Center-of-mass offset from the body origin (meters).
    pub center_of_mass_offset: Vec3,
    pub mass_kg: f32,
    /// Anti-flip cone half angle (radians); PI turns it off.
    pub max_pitch_roll_angle: f32,
    /// Wheel ray-cast sphere radius; ~half a wheel width.
    pub tester_radius: f32,
    pub wheels: Vec<VehicleWheel>,
    pub engine: VehicleEngine,
    pub transmission: VehicleTransmission,
    pub kind: VehicleKind,
}

impl Default for VehicleSpec {
    /// The old demo car: 1.8 x 1.2 x 4.4 box, 1500 kg, 4 wheels, front
    /// steering + rear handbrake, single front differential.
    fn default() -> Self {
        let wheel_radius = 0.35;
        let mut wheels = Vec::with_capacity(4);
        for side in [1.0, -1.0] {
            for (front, z) in [(true, 2.2 - 2.0 * wheel_radius), (false, -2.2 + 2.0 * wheel_radius)] {
                wheels.push(VehicleWheel {
                    mount_position: Vec3::new(0.9 * side, -0.9 * 0.6, z),
                    suspension_min_length: 0.2,
                    suspension_max_length: 0.4,
                    wheel_radius,
                    wheel_width: 0.25,
                    max_steer_angle: if front { 30.0f32.to_radians() } else { 0.0 },
                    max_hand_brake_torque: if front { 0.0 } else { 4000.0 },
                    ..VehicleWheel::default()
                });
            }
        }
        Self {
            object_layer: 1,
            half_extents: Vec3::new(0.9, 0.6, 2.2),
            center_of_mass_offset: Vec3::new(0.0, -0.9, 0.0),
            mass_kg: 1500.0,
            max_pitch_roll_angle: 60.0f32.to_radians(),
            tester_radius: 0.125,
            wheels,
            engine: VehicleEngine::default(),
            transmission: VehicleTransmission::default(),
            kind: VehicleKind::default(),
        }
    }
}

impl VehicleSpec {
    pub fn new(object_layer: u16) -> Self {
        Self {
            object_layer,
            ..Self::default()
        }
    }
}

/// Back-compat constructor: `JoltVehicle::new(layer)` spawns the default
/// demo car spec. Prefer [`VehicleSpec`] for new code.
#[derive(Component, Clone, Debug)]
pub struct JoltVehicle {
    pub object_layer: u16,
}

impl JoltVehicle {
    pub fn new(object_layer: u16) -> Self {
        Self { object_layer }
    }
}

/// Jolt chassis body + vehicle constraint owned by an entity. Inserted by
/// the creation system; game code reads it but never writes it.
#[derive(Component, Clone, Copy, Debug)]
pub struct JoltVehicleId {
    pub body_id_raw: u32,
    pub constraint_id_raw: u32,
}

/// Held vehicle input, pushed into Jolt every tick while present. Change
/// the fields to drive, remove the component to coast on the last input.
///
/// Wheeled/motorcycle: `forward` gas [-1, 1], `steer` right-positive
/// [-1, 1], `brake` foot brake [0, 1], `hand_brake` [0, 1]. Tracked: the
/// `steer`/`hand_brake` slots become `left_ratio`/`right_ratio` track
/// multipliers (1 full, -1 reversed); use `JoltTrackedDrive` instead.
#[derive(Component, Clone, Copy, Debug)]
pub struct JoltVehicleDrive {
    pub forward: f32,
    pub steer: f32,
    pub brake: f32,
    pub hand_brake: f32,
}

/// Held tank input: gas plus per-track multipliers. Same push semantics
/// as [`JoltVehicleDrive`]; only meaningful on tracked vehicles.
#[derive(Component, Clone, Copy, Debug)]
pub struct JoltTrackedDrive {
    pub forward: f32,
    pub left_ratio: f32,
    pub right_ratio: f32,
    pub brake: f32,
}

/// Held manual-gear input: gear (-1 reverse, 0 neutral, 1+ forward) plus
/// clutch friction [0, 1]. Auto boxes ignore it.
#[derive(Component, Clone, Copy, Debug)]
pub struct JoltVehicleShift {
    pub gear: i32,
    pub clutch_friction: f32,
}

/// Creates the Jolt chassis body plus wheels for each vehicle entity
/// missing a [`JoltVehicleId`]. Polls instead of using `On<Add>`: like
/// joints, creation waits a flush and must run before the physics step so
/// the vehicle exists for its first tick.
pub fn create_jolt_vehicles(
    mut commands: Commands,
    pending_specs: Query<(Entity, &VehicleSpec, &Transform), Without<JoltVehicleId>>,
    pending_legacy: Query<(Entity, &JoltVehicle, &Transform), Without<JoltVehicleId>>,
    mut physics_world: ResMut<JoltPhysicsWorld>,
) {
    for (vehicle_entity, spec, spawn_transform) in &pending_specs {
        let Some(vehicle_id) =
            physics_world.create_vehicle(spec, spawn_transform.translation)
        else {
            panic!("Jolt rejected vehicle creation on {vehicle_entity:?}: {spec:?}");
        };
        insert_vehicle_id(&mut commands, vehicle_entity, vehicle_id, spec, spawn_transform);
    }
    for (vehicle_entity, legacy, spawn_transform) in &pending_legacy {
        let spec = VehicleSpec::new(legacy.object_layer);
        let Some(vehicle_id) =
            physics_world.create_vehicle(&spec, spawn_transform.translation)
        else {
            panic!("Jolt rejected vehicle creation on {vehicle_entity:?}: {spec:?}");
        };
        insert_vehicle_id(&mut commands, vehicle_entity, vehicle_id, &spec, spawn_transform);
    }
}

fn insert_vehicle_id(
    commands: &mut Commands,
    vehicle_entity: Entity,
    vehicle_id: JoltVehicleId,
    spec: &VehicleSpec,
    spawn_transform: &Transform,
) {
    let half_extents = spec.half_extents;
    commands.entity(vehicle_entity).insert((
        vehicle_id,
        // Joins the shared transform sync + debug draw: the chassis box
        // matches the C++ shape, so the vehicle entity carries its own
        // pose like any body.
        crate::body_sync::JoltBodyId {
            body_id_raw: vehicle_id.body_id_raw,
        },
        crate::body_sync::PreviousBodyTransform {
            previous_position: spawn_transform.translation,
            previous_rotation: spawn_transform.rotation,
        },
    ));
    let body_id_raw = vehicle_id.body_id_raw;
    commands.queue(move |world: &mut World| {
        let mut physics_world = world.resource_mut::<JoltPhysicsWorld>();
        physics_world.register_shape(
            body_id_raw,
            crate::physics_world::PhysicsShape::Box { half_extents },
        );
    });
}

/// Pushes every drive/shift component into its vehicle constraint before
/// the physics step. Vehicles missing their id (not baked yet) are skipped
/// for the tick.
pub fn apply_jolt_vehicle_drives(
    drive_query: Query<(&JoltVehicleId, &JoltVehicleDrive)>,
    tracked_query: Query<(&JoltVehicleId, &JoltTrackedDrive)>,
    shift_query: Query<(&JoltVehicleId, &JoltVehicleShift)>,
    mut physics_world: ResMut<JoltPhysicsWorld>,
) {
    for (vehicle_id, drive) in &drive_query {
        physics_world.vehicle_drive(
            vehicle_id.constraint_id_raw,
            drive.forward,
            drive.steer,
            drive.brake,
            drive.hand_brake,
        );
    }
    for (vehicle_id, drive) in &tracked_query {
        physics_world.tracked_drive(
            vehicle_id.constraint_id_raw,
            drive.forward,
            drive.left_ratio,
            drive.right_ratio,
            drive.brake,
        );
    }
    for (vehicle_id, shift) in &shift_query {
        physics_world.vehicle_shift(
            vehicle_id.constraint_id_raw,
            shift.gear,
            shift.clutch_friction,
        );
    }
}

/// Removes the Jolt vehicle constraint when its entity is despawned. The
/// chassis body is owned by `despawn_jolt_body` via the shared `JoltBodyId`:
/// splitting ownership this way keeps teardown order-independent
/// (constraint removal never touches the body, body destroy never touches
/// the constraint registry).
pub fn despawn_jolt_vehicle(
    trigger: On<Remove, JoltVehicleId>,
    vehicle_query: Query<&JoltVehicleId>,
    mut physics_world: ResMut<JoltPhysicsWorld>,
) {
    let trigger_entity = trigger.event().entity;
    let Ok(vehicle_id) = vehicle_query.get(trigger_entity) else {
        panic!("JoltVehicleId gone on {trigger_entity:?} before despawn ran");
    };
    physics_world.remove_constraint(vehicle_id.constraint_id_raw);
}

/// Max pitch/roll angle with PI turning the anti-flip cone off.
pub const VEHICLE_NO_FLIP_LIMIT: f32 = PI;
