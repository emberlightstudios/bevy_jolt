//! Joint ↔ constraint sync: spec component, creation polling, despawn.
//!
//! Game code spawns an entity with [`JoltJoint`] (two body entities plus a
//! [`JointKind`]). The plugin creates the Jolt constraint once both bodies
//! own a [`JoltBodyId`](crate::JoltBodyId), stores the resulting
//! [`JoltJointId`], and removes the constraint when the entity leaves. If
//! either body is destroyed first, dependent joints go with it so the solver
//! never holds a dangling `BodyID`.

use bevy::prelude::*;

use crate::body_sync::{JoltBody, JoltBodyId};
use crate::physics_world::JointSpace;
use crate::plugin::JoltPhysicsWorld;

/// Joint spec: which two bodies to link plus what kind of constraint.
/// Creation params (anchors, axes, limits) live here; the untyped
/// [`JoltJointId`] holds the runtime handle. No `Transform`: joints aren't
/// spatial. `joint_space` says which frame those anchors/axes live in:
/// `World` (global) or `LocalToBodyCom` (relative to each body's center
/// of mass). Local is NOT the Bevy `Transform` origin: subtract the
/// shape's center-of-mass offset first.
#[derive(Component, Clone, Copy, Debug)]
pub struct JoltJoint {
    pub body_a: Entity,
    pub body_b: Entity,
    pub joint_space: JointSpace,
    pub kind: JointKind,
}

/// Velocity-motor tuning for a driven joint axis. Mirrors Jolt's single
/// `MotorSettings` type (spring frequency + damping) plus the force cap.
/// Stiffer (higher frequency) chases the target speed harder; damping
/// settles overshoot.
#[derive(Clone, Copy, Debug)]
pub struct JointMotor {
    /// Spring stiffness in Hz. Higher corrects speed errors faster.
    pub frequency_hz: f32,
    /// Spring damping ratio. 1 is critically damped, no overshoot.
    pub damping: f32,
    /// Max force (sliders/paths) or torque (hinges) the motor may apply.
    pub force_limit: f32,
}

impl Default for JointMotor {
    fn default() -> Self {
        Self {
            frequency_hz: 8.0,
            damping: 1.0,
            force_limit: 1.0e6,
        }
    }
}
/// Max Hermite knots per path track. Matches `BJOLT_MAX_PATH_POINTS` /
/// `MAX_PATH_POINTS` in the FFI.
pub const MAX_PATH_KNOTS: usize = 16;

/// One Hermite spline knot for a [`JointKind::PathCart`] track: position
/// plus tangent (arrival direction scaled by segment weight) and normal
/// (track up). Jolt interpolates all three between knots.
#[derive(Clone, Copy, Debug, Default)]
pub struct PathKnot {
    pub knot_position: Vec3,
    pub knot_tangent: Vec3,
    pub knot_normal: Vec3,
}

impl PathKnot {
    /// Pair two of these for the old two-stop shuttle.
    pub fn straight(knot_position: Vec3, segment: Vec3, track_up: Vec3) -> Self {
        Self {
            knot_position,
            knot_tangent: segment,
            knot_normal: track_up,
        }
    }
}

/// Builds Hermite knots from waypoints the Jolt-sample way: central
/// differences for tangents (`0.5 * (next - prev)`), constant up normal.
/// Endpoints use one-sided differences. Looping wraps the neighbor lookup.
/// Caps at `MAX_PATH_KNOTS`; needs at least 2 points.
pub fn path_knots_from_waypoints(waypoints: &[Vec3], looping: bool) -> Vec<PathKnot> {
    let point_count = waypoints.len().min(MAX_PATH_KNOTS);
    if point_count < 2 {
        return Vec::new();
    }
    (0..point_count)
        .map(|knot_index| {
            let prev_index = if knot_index == 0 {
                if looping { point_count - 1 } else { knot_index }
            } else {
                knot_index - 1
            };
            let next_index = if knot_index + 1 == point_count {
                if looping { 0 } else { knot_index }
            } else {
                knot_index + 1
            };
            PathKnot {
                knot_position: waypoints[knot_index],
                knot_tangent: 0.5 * (waypoints[next_index] - waypoints[prev_index]),
                knot_normal: Vec3::Y,
            }
        })
        .collect()
}

/// Reference frames for a [`JointKind::SixDof`] joint: anchor + basis per
/// body. Differing body2 vectors offset the rest pose without moving the
#[derive(Clone, Copy, Debug)]
pub struct SixDofFrame {
    pub position1: Vec3,
    pub axis_x1: Dir3,
    pub axis_y1: Dir3,
    pub position2: Vec3,
    pub axis_x2: Dir3,
    pub axis_y2: Dir3,
}

/// Per-axis travel bands for a [`JointKind::SixDof`] joint, in frame units
/// (meters for translation, radians for rotation). Per axis: min > max fixes
/// it, +-large (≈FLT_MAX) frees it, otherwise the band clamps travel.
#[derive(Clone, Copy, Debug)]
pub struct SixDofLimits {
    pub translation_min: Vec3,
    pub translation_max: Vec3,
    pub rotation_min: Vec3,
    pub rotation_max: Vec3,
}

/// Bit index per six-DOF axis in the `motor_axes` mask: 0 = TX .. 5 = RZ.
#[derive(Clone, Copy, Debug)]
pub enum SixDofAxis {
    TranslationX,
    TranslationY,
    TranslationZ,
    RotationX,
    RotationY,
    RotationZ,
}

impl SixDofAxis {
    pub const fn bit(self) -> u8 {
        1 << (self as u8)
    }
}

/// Constraint flavor plus its creation params. Mirrors the
/// `JoltWorld::create_*_constraint` set one-to-one.
#[derive(Clone, Copy, Debug)]
pub enum JointKind {
    Fixed,
    Distance {
        point1: Vec3,
        point2: Vec3,
        min_distance: f32,
        max_distance: f32,
    },
    Hinge {
        hinge_point: Vec3,
        hinge_axis1: Dir3,
        normal_axis1: Dir3,
        hinge_axis2: Dir3,
        normal_axis2: Dir3,
        limits_min: f32,
        limits_max: f32,
        motor: JointMotor,
    },
    Point {
        constraint_point: Vec3,
    },
    Slider {
        slider_axis1: Dir3,
        normal_axis1: Dir3,
        slider_axis2: Dir3,
        normal_axis2: Dir3,
        limits_min: f32,
        limits_max: f32,
        motor: JointMotor,
    },
    Cone {
        constraint_point: Vec3,
        twist_axis1: Dir3,
        twist_axis2: Dir3,
        half_cone_angle: f32,
    },
    SwingTwist {
        constraint_position: Vec3,
        twist_axis1: Dir3,
        plane_axis1: Dir3,
        twist_axis2: Dir3,
        plane_axis2: Dir3,
        normal_half_cone_angle: f32,
        plane_half_cone_angle: f32,
        twist_min_angle: f32,
        twist_max_angle: f32,
    },
    SixDof {
        frame: SixDofFrame,
        limits: SixDofLimits,
        motor_axes: u8,
        motor: JointMotor,
    },
    Pulley {
        body_point1: Vec3,
        fixed_point1: Vec3,
        body_point2: Vec3,
        fixed_point2: Vec3,
        ratio: f32,
        min_length: f32,
        max_length: f32,
    },
    Gear {
        hinge_axis: Dir3,
        ratio: f32,
        hinge_a: Entity,
        hinge_b: Entity,
    },
    RackPinion {
        hinge_axis: Dir3,
        slider_axis: Dir3,
        ratio: f32,
        pinion_hinge: Entity,
        rack_slider: Entity,
    },
    PathCart {
        track_knots: [PathKnot; MAX_PATH_KNOTS],
        knot_count: u8,
        looping: bool,
        motor: JointMotor,
    },
}

impl JoltJoint {
    pub fn fixed(body_a: Entity, body_b: Entity, joint_space: JointSpace) -> Self {
        Self {
            body_a,
            body_b,
            joint_space,
            kind: JointKind::Fixed,
        }
    }
    pub fn distance(
        body_a: Entity,
        body_b: Entity,
        point1: Vec3,
        point2: Vec3,
        min_distance: f32,
        max_distance: f32,
        joint_space: JointSpace,
    ) -> Self {
        Self {
            body_a,
            body_b,
            joint_space,
            kind: JointKind::Distance {
                point1,
                point2,
                min_distance,
                max_distance,
            },
        }
    }
    pub fn hinge(
        body_a: Entity,
        body_b: Entity,
        hinge_point: Vec3,
        hinge_axis1: Dir3,
        normal_axis1: Dir3,
        hinge_axis2: Dir3,
        normal_axis2: Dir3,
        joint_space: JointSpace,
    ) -> Self {
        Self {
            body_a,
            body_b,
            joint_space,
            kind: JointKind::Hinge {
                hinge_point,
                hinge_axis1,
                normal_axis1,
                hinge_axis2,
                normal_axis2,
                limits_min: -core::f32::consts::PI,
                limits_max: core::f32::consts::PI,
                motor: JointMotor::default(),
            },
        }
    }

    /// Hinge with swing limits in radians: min in [-pi, 0], max in [0, pi].
    /// The motor clamps inside the band, so a ping-pong drive bounces off the
    /// stops instead of spinning through.
    #[allow(clippy::too_many_arguments)]
    pub fn hinge_limited(
        body_a: Entity,
        body_b: Entity,
        hinge_point: Vec3,
        hinge_axis1: Dir3,
        normal_axis1: Dir3,
        hinge_axis2: Dir3,
        normal_axis2: Dir3,
        limits_min: f32,
        limits_max: f32,
        joint_space: JointSpace,
    ) -> Self {
        assert!(
            (-core::f32::consts::PI..=0.0).contains(&limits_min),
            "hinge min {} outside [-pi, 0]",
            limits_min
        );
        assert!(
            (0.0..=core::f32::consts::PI).contains(&limits_max),
            "hinge max {} outside [0, pi]",
            limits_max
        );
        assert!(
            limits_min <= limits_max,
            "hinge min {} above max {}",
            limits_min,
            limits_max
        );
        Self {
            body_a,
            body_b,
            joint_space,
            kind: JointKind::Hinge {
                hinge_point,
                hinge_axis1,
                normal_axis1,
                hinge_axis2,
                normal_axis2,
                limits_min,
                limits_max,
                motor: JointMotor::default(),
            },
        }
    }

    pub fn point(
        body_a: Entity,
        body_b: Entity,
        constraint_point: Vec3,
        joint_space: JointSpace,
    ) -> Self {
        Self {
            body_a,
            body_b,
            joint_space,
            kind: JointKind::Point { constraint_point },
        }
    }

    pub fn slider(
        body_a: Entity,
        body_b: Entity,
        slider_axis1: Dir3,
        normal_axis1: Dir3,
        slider_axis2: Dir3,
        normal_axis2: Dir3,
        limits_min: f32,
        limits_max: f32,
        joint_space: JointSpace,
    ) -> Self {
        Self {
            body_a,
            body_b,
            joint_space,
            kind: JointKind::Slider {
                slider_axis1,
                normal_axis1,
                slider_axis2,
                normal_axis2,
                limits_min,
                limits_max,
                motor: JointMotor::default(),
            },
        }
    }

    pub fn cone(
        body_a: Entity,
        body_b: Entity,
        constraint_point: Vec3,
        twist_axis1: Dir3,
        twist_axis2: Dir3,
        half_cone_angle: f32,
        joint_space: JointSpace,
    ) -> Self {
        Self {
            body_a,
            body_b,
            joint_space,
            kind: JointKind::Cone {
                constraint_point,
                twist_axis1,
                twist_axis2,
                half_cone_angle,
            },
        }
    }
    #[allow(clippy::too_many_arguments)]
    pub fn swing_twist(
        body_a: Entity,
        body_b: Entity,
        constraint_position: Vec3,
        twist_axis1: Dir3,
        plane_axis1: Dir3,
        twist_axis2: Dir3,
        plane_axis2: Dir3,
        normal_half_cone_angle: f32,
        plane_half_cone_angle: f32,
        twist_min_angle: f32,
        twist_max_angle: f32,
        joint_space: JointSpace,
    ) -> Self {
        Self {
            body_a,
            body_b,
            joint_space,
            kind: JointKind::SwingTwist {
                constraint_position,
                twist_axis1,
                plane_axis1,
                twist_axis2,
                plane_axis2,
                normal_half_cone_angle,
                plane_half_cone_angle,
                twist_min_angle,
                twist_max_angle,
            },
        }
    }

    /// Fully general six-DOF joint: reference frames, per-axis limits, and a
    /// velocity motor on any axis subset (`motor_axes` bit `i`, see
    /// [`SixDofAxis::bit`]). Pair with [`JoltMotorDrive`] to retarget every
    /// driven axis each tick.
    #[allow(clippy::too_many_arguments)]
    pub fn six_dof(
        body_a: Entity,
        body_b: Entity,
        frame: SixDofFrame,
        limits: SixDofLimits,
        motor_axes: u8,
        motor: JointMotor,
        joint_space: JointSpace,
    ) -> Self {
        Self {
            body_a,
            body_b,
            joint_space,
            kind: JointKind::SixDof {
                frame,
                limits,
                motor_axes,
                motor,
            },
        }
    }

    #[allow(clippy::too_many_arguments)]
    pub fn pulley(
        body_a: Entity,
        body_b: Entity,
        body_point1: Vec3,
        fixed_point1: Vec3,
        body_point2: Vec3,
        fixed_point2: Vec3,
        ratio: f32,
        min_length: f32,
        max_length: f32,
        joint_space: JointSpace,
    ) -> Self {
        Self {
            body_a,
            body_b,
            joint_space,
            kind: JointKind::Pulley {
                body_point1,
                fixed_point1,
                body_point2,
                fixed_point2,
                ratio,
                min_length,
                max_length,
            },
        }
    }

    pub fn gear(
        body_a: Entity,
        body_b: Entity,
        hinge_axis: Dir3,
        ratio: f32,
        hinge_a: Entity,
        hinge_b: Entity,
        joint_space: JointSpace,
    ) -> Self {
        Self {
            body_a,
            body_b,
            joint_space,
            kind: JointKind::Gear {
                hinge_axis,
                ratio,
                hinge_a,
                hinge_b,
            },
        }
    }

    #[allow(clippy::too_many_arguments)]
    pub fn rack_pinion(
        body_a: Entity,
        body_b: Entity,
        hinge_axis: Dir3,
        slider_axis: Dir3,
        ratio: f32,
        pinion_hinge: Entity,
        rack_slider: Entity,
        joint_space: JointSpace,
    ) -> Self {
        Self {
            body_a,
            body_b,
            joint_space,
            kind: JointKind::RackPinion {
                hinge_axis,
                slider_axis,
                ratio,
                pinion_hinge,
                rack_slider,
            },
        }
    }

    /// General Hermite spline track: knot slice plus a looping flag.
    /// `joint_space` is accepted for uniformity but ignored: track points
    /// are path-local to the static body's frame.
    pub fn path_cart(
        static_body: Entity,
        cart_body: Entity,
        track_knots: &[PathKnot],
        looping: bool,
        motor: JointMotor,
        joint_space: JointSpace,
    ) -> Self {
        let mut knots = [PathKnot::default(); MAX_PATH_KNOTS];
        let knot_count = track_knots.len().min(MAX_PATH_KNOTS);
        knots[..knot_count].copy_from_slice(&track_knots[..knot_count]);
        Self {
            body_a: static_body,
            body_b: cart_body,
            joint_space,
            kind: JointKind::PathCart {
                track_knots: knots,
                knot_count: knot_count as u8,
                looping,
                motor,
            },
        }
    }

    /// Waypoint track: builds Hermite knots via
    /// [`path_knots_from_waypoints`] (central-difference tangents, up
    /// normals) and calls [`JoltJoint::path_cart`].
    pub fn path_waypoints(
        static_body: Entity,
        cart_body: Entity,
        waypoints: &[Vec3],
        looping: bool,
        motor: JointMotor,
        joint_space: JointSpace,
    ) -> Self {
        let track_knots = path_knots_from_waypoints(waypoints, looping);
        Self::path_cart(
            static_body,
            cart_body,
            &track_knots,
            looping,
            motor,
            joint_space,
        )
    }

    /// True when this joint's creation depends on another joint entity
    /// (gear / rack-pinion sub-joints).
    fn depends_on_joint(&self, joint_entity: Entity) -> bool {
        match self.kind {
            JointKind::Gear { hinge_a, hinge_b, .. } => {
                hinge_a == joint_entity || hinge_b == joint_entity
            }
            JointKind::RackPinion {
                pinion_hinge,
                rack_slider,
                ..
            } => pinion_hinge == joint_entity || rack_slider == joint_entity,
            _ => false,
        }
    }
}

/// Jolt constraint id owned by a joint entity. Inserted by the creation
/// system; game code reads it but never writes it. Untyped on purpose:
/// [`JoltJoint`]`.kind` already says what the joint is.
#[derive(Component, Clone, Copy, Debug)]
pub struct JoltJointId {
    pub constraint_id_raw: u32,
}

/// Creates the Jolt constraint for each [`JoltJoint`] whose endpoints are
/// ready, and stores the resulting [`JoltJointId`]. Polls instead of using
/// `On<Add>`: bodies usually gain their `JoltBodyId` a flush after the joint
/// spawns, and gear-style joints wait on sub-joint ids the same way. Runs
/// before the physics step so a joint exists for its first tick.
pub fn create_jolt_joints(
    mut commands: Commands,
    pending: Query<(Entity, &JoltJoint), Without<JoltJointId>>,
    joint_ids: Query<&JoltJointId>,
    body_ids: Query<&JoltBodyId>,
    bodies: Query<(), With<JoltBody>>,
    mut physics_world: ResMut<JoltPhysicsWorld>,
) {
    let body_raw = |entity: Entity| {
        body_ids
            .get(entity)
            .ok()
            .map(|body_id| body_id.body_id_raw)
    };
    let sub_joint_raw = |entity: Entity| {
        joint_ids
            .get(entity)
            .ok()
            .map(|joint_id| joint_id.constraint_id_raw)
    };
    for (joint_entity, joint) in &pending {
        // An endpoint with neither body marker nor body id is gone for good
        // (despawned, or never a body): drop the joint instead of retrying
        // forever. Missing ids on live bodies just mean "not baked yet".
        let endpoint_gone =
            |entity: Entity| body_ids.get(entity).is_err() && !bodies.contains(entity);
        if endpoint_gone(joint.body_a) || endpoint_gone(joint.body_b) {
            commands.entity(joint_entity).despawn();
            continue;
        }
        let (Some(body_a_raw), Some(body_b_raw)) =
            (body_raw(joint.body_a), body_raw(joint.body_b))
        else {
            continue;
        };
        let world = &mut **physics_world;
        let constraint_id = match joint.kind {
            JointKind::Fixed => {
                world.create_fixed_constraint(body_a_raw, body_b_raw, joint.joint_space)
            }
            JointKind::Distance {
                point1,
                point2,
                min_distance,
                max_distance,
            } => world.create_distance_constraint(
                body_a_raw,
                body_b_raw,
                point1,
                point2,
                min_distance,
                max_distance,
                joint.joint_space,
            ),
            JointKind::Hinge {
                hinge_point,
                hinge_axis1,
                normal_axis1,
                hinge_axis2,
                normal_axis2,
                limits_min,
                limits_max,
                motor,
            } => world.create_hinge_constraint(
                body_a_raw,
                body_b_raw,
                hinge_point,
                hinge_axis1,
                normal_axis1,
                hinge_axis2,
                normal_axis2,
                limits_min,
                limits_max,
                motor,
                joint.joint_space,
            ),
            JointKind::Point { constraint_point } => world.create_point_constraint(
                body_a_raw,
                body_b_raw,
                constraint_point,
                joint.joint_space,
            ),
            JointKind::Slider {
                slider_axis1,
                normal_axis1,
                slider_axis2,
                normal_axis2,
                limits_min,
                limits_max,
                motor,
            } => world.create_slider_constraint(
                body_a_raw,
                body_b_raw,
                slider_axis1,
                normal_axis1,
                slider_axis2,
                normal_axis2,
                limits_min,
                limits_max,
                motor,
                joint.joint_space,
            ),
            JointKind::Cone {
                constraint_point,
                twist_axis1,
                twist_axis2,
                half_cone_angle,
            } => world.create_cone_constraint(
                body_a_raw,
                body_b_raw,
                constraint_point,
                twist_axis1,
                twist_axis2,
                half_cone_angle,
                joint.joint_space,
            ),
            JointKind::SwingTwist {
                constraint_position,
                twist_axis1,
                plane_axis1,
                twist_axis2,
                plane_axis2,
                normal_half_cone_angle,
                plane_half_cone_angle,
                twist_min_angle,
                twist_max_angle,
            } => world.create_swing_twist_constraint(
                body_a_raw,
                body_b_raw,
                constraint_position,
                twist_axis1,
                plane_axis1,
                twist_axis2,
                plane_axis2,
                normal_half_cone_angle,
                plane_half_cone_angle,
                twist_min_angle,
                twist_max_angle,
                joint.joint_space,
            ),
            JointKind::SixDof {
                frame,
                limits,
                motor_axes,
                motor,
            } => world.create_six_dof(
                body_a_raw,
                body_b_raw,
                frame,
                limits,
                motor_axes,
                motor,
                joint.joint_space,
            ),
            JointKind::Pulley {
                body_point1,
                fixed_point1,
                body_point2,
                fixed_point2,
                ratio,
                min_length,
                max_length,
            } => world.create_pulley_constraint(
                body_a_raw,
                body_b_raw,
                body_point1,
                fixed_point1,
                body_point2,
                fixed_point2,
                ratio,
                min_length,
                max_length,
                joint.joint_space,
            ),
            JointKind::Gear {
                hinge_axis,
                ratio,
                hinge_a,
                hinge_b,
            } => {
                let (Some(hinge_a_raw), Some(hinge_b_raw)) =
                    (sub_joint_raw(hinge_a), sub_joint_raw(hinge_b))
                else {
                    continue;
                };
                world.create_gear_constraint(
                    body_a_raw,
                    body_b_raw,
                    hinge_axis,
                    ratio,
                    hinge_a_raw,
                    hinge_b_raw,
                    joint.joint_space,
                )
            }
            JointKind::RackPinion {
                hinge_axis,
                slider_axis,
                ratio,
                pinion_hinge,
                rack_slider,
            } => {
                let (Some(hinge_raw), Some(slider_raw)) =
                    (sub_joint_raw(pinion_hinge), sub_joint_raw(rack_slider))
                else {
                    continue;
                };
                world.create_rack_pinion_constraint(
                    body_a_raw,
                    body_b_raw,
                    hinge_axis,
                    slider_axis,
                    ratio,
                    hinge_raw,
                    slider_raw,
                    joint.joint_space,
                )
            }
            JointKind::PathCart {
                track_knots,
                knot_count,
                looping,
                motor,
            } => world.create_path_cart(
                body_a_raw,
                body_b_raw,
                &track_knots[..knot_count as usize],
                looping,
                motor,
            ),
        };
        assert!(
            constraint_id != 0,
            "Jolt rejected joint creation on {joint_entity:?}: {joint:?}",
        );
        commands
            .entity(joint_entity)
            .insert(JoltJointId { constraint_id_raw: constraint_id });
    }
}

/// Held motor target: retargets the joint's velocity motor every tick while
/// present. Attach alongside [`JoltJoint`]; the sync system pushes the value
/// into Jolt before the physics step. Change the value to change speed,
/// remove the component to leave the last target in place.
/// Target motor speed: m/s for sliders/paths, rad/s for hinges.
#[derive(Component, Clone, Copy, Debug, Deref, DerefMut)]
pub struct JoltMotorDrive(pub f32);

/// Pushes every [`JoltMotorDrive`] into its joint's motor before the physics
/// step. Joints missing their id (not baked yet) are skipped for the tick.
pub fn apply_jolt_motor_drives(
    drive_query: Query<(&JoltJointId, &JoltMotorDrive)>,
    mut physics_world: ResMut<JoltPhysicsWorld>,
) {
    for (joint_id, drive) in &drive_query {
        physics_world.constraint_drive_at(joint_id.constraint_id_raw, drive.0);
    }
}

/// Removes the Jolt constraint when its joint entity is despawned, and drops
/// gear-style joints whose sub-joint died with it.
pub fn despawn_jolt_joint(
    trigger: On<Remove, JoltJointId>,
    mut commands: Commands,
    joint_query: Query<&JoltJointId>,
    dependents: Query<(Entity, &JoltJoint)>,
    mut physics_world: ResMut<JoltPhysicsWorld>,
) {
    let trigger_entity = trigger.event().entity;
    let Ok(joint_id) = joint_query.get(trigger_entity) else {
        panic!(
            "JoltJointId gone on {:?} before despawn ran",
            trigger_entity
        );
    };
    physics_world.remove_constraint(joint_id.constraint_id_raw);
    for (entity, joint) in &dependents {
        if entity != trigger_entity && joint.depends_on_joint(trigger_entity) {
            commands.entity(entity).despawn();
        }
    }
}

/// Destroys joints attached to a body before the body itself goes away.
/// Registered before `despawn_jolt_body`: the constraint removal here is
/// synchronous, while the joint-entity despawn is deferred, so the solver
/// never sees a constraint pointing at a destroyed body. `remove_constraint`
/// is idempotent, making the later joint-despawn observer a safe no-op.
pub fn cascade_body_remove_to_joints(
    trigger: On<Remove, JoltBodyId>,
    joints: Query<(Entity, &JoltJoint)>,
    joint_ids: Query<&JoltJointId>,
    mut commands: Commands,
    mut physics_world: ResMut<JoltPhysicsWorld>,
) {
    let dead_body = trigger.event().entity;
    for (joint_entity, joint) in &joints {
        if joint.body_a == dead_body || joint.body_b == dead_body {
            if let Ok(joint_id) = joint_ids.get(joint_entity) {
                physics_world.remove_constraint(joint_id.constraint_id_raw);
            }
            commands.entity(joint_entity).despawn();
        }
    }
}
