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
use crate::plugin::JoltPhysicsWorld;

/// Joint spec: which two bodies to link plus what kind of constraint.
/// Creation params (anchors, axes, limits) live here; the untyped
/// [`JoltJointId`] holds the runtime handle. No `Transform`: joints aren't
/// spatial, anchors bake in world space on the creation tick.
#[derive(Component, Clone, Copy, Debug)]
pub struct JoltJoint {
    pub body_a: Entity,
    pub body_b: Entity,
    pub kind: JointKind,
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
        hinge_axis: Vec3,
        normal_axis: Vec3,
    },
    Point {
        constraint_point: Vec3,
    },
    Slider {
        slider_axis: Vec3,
        normal_axis: Vec3,
        limits_min: f32,
        limits_max: f32,
    },
    Cone {
        constraint_point: Vec3,
        twist_axis1: Vec3,
        twist_axis2: Vec3,
        half_cone_angle: f32,
    },
    SwingTwist {
        constraint_position: Vec3,
        twist_axis: Vec3,
        plane_axis: Vec3,
        normal_half_cone_angle: f32,
        plane_half_cone_angle: f32,
        twist_min_angle: f32,
        twist_max_angle: f32,
    },
    SixDofSlider {
        limit_y_min: f32,
        limit_y_max: f32,
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
        hinge_axis: Vec3,
        ratio: f32,
        hinge_a: Entity,
        hinge_b: Entity,
    },
    RackPinion {
        hinge_axis: Vec3,
        slider_axis: Vec3,
        ratio: f32,
        pinion_hinge: Entity,
        rack_slider: Entity,
    },
    PathCart {
        track_from: Vec3,
        track_to: Vec3,
    },
}

impl JoltJoint {
    pub fn fixed(body_a: Entity, body_b: Entity) -> Self {
        Self {
            body_a,
            body_b,
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
    ) -> Self {
        Self {
            body_a,
            body_b,
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
        hinge_axis: Vec3,
        normal_axis: Vec3,
    ) -> Self {
        Self {
            body_a,
            body_b,
            kind: JointKind::Hinge {
                hinge_point,
                hinge_axis,
                normal_axis,
            },
        }
    }

    pub fn point(body_a: Entity, body_b: Entity, constraint_point: Vec3) -> Self {
        Self {
            body_a,
            body_b,
            kind: JointKind::Point { constraint_point },
        }
    }

    pub fn slider(
        body_a: Entity,
        body_b: Entity,
        slider_axis: Vec3,
        normal_axis: Vec3,
        limits_min: f32,
        limits_max: f32,
    ) -> Self {
        Self {
            body_a,
            body_b,
            kind: JointKind::Slider {
                slider_axis,
                normal_axis,
                limits_min,
                limits_max,
            },
        }
    }

    pub fn cone(
        body_a: Entity,
        body_b: Entity,
        constraint_point: Vec3,
        twist_axis1: Vec3,
        twist_axis2: Vec3,
        half_cone_angle: f32,
    ) -> Self {
        Self {
            body_a,
            body_b,
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
        twist_axis: Vec3,
        plane_axis: Vec3,
        normal_half_cone_angle: f32,
        plane_half_cone_angle: f32,
        twist_min_angle: f32,
        twist_max_angle: f32,
    ) -> Self {
        Self {
            body_a,
            body_b,
            kind: JointKind::SwingTwist {
                constraint_position,
                twist_axis,
                plane_axis,
                normal_half_cone_angle,
                plane_half_cone_angle,
                twist_min_angle,
                twist_max_angle,
            },
        }
    }

    pub fn six_dof_slider(
        body_a: Entity,
        body_b: Entity,
        limit_y_min: f32,
        limit_y_max: f32,
    ) -> Self {
        Self {
            body_a,
            body_b,
            kind: JointKind::SixDofSlider {
                limit_y_min,
                limit_y_max,
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
    ) -> Self {
        Self {
            body_a,
            body_b,
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
        hinge_axis: Vec3,
        ratio: f32,
        hinge_a: Entity,
        hinge_b: Entity,
    ) -> Self {
        Self {
            body_a,
            body_b,
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
        hinge_axis: Vec3,
        slider_axis: Vec3,
        ratio: f32,
        pinion_hinge: Entity,
        rack_slider: Entity,
    ) -> Self {
        Self {
            body_a,
            body_b,
            kind: JointKind::RackPinion {
                hinge_axis,
                slider_axis,
                ratio,
                pinion_hinge,
                rack_slider,
            },
        }
    }

    pub fn path_cart(
        static_body: Entity,
        cart_body: Entity,
        track_from: Vec3,
        track_to: Vec3,
    ) -> Self {
        Self {
            body_a: static_body,
            body_b: cart_body,
            kind: JointKind::PathCart {
                track_from,
                track_to,
            },
        }
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
            JointKind::Fixed => world.create_fixed_constraint(body_a_raw, body_b_raw),
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
            ),
            JointKind::Hinge {
                hinge_point,
                hinge_axis,
                normal_axis,
            } => world.create_hinge_constraint(
                body_a_raw,
                body_b_raw,
                hinge_point,
                hinge_axis,
                normal_axis,
            ),
            JointKind::Point { constraint_point } => {
                world.create_point_constraint(body_a_raw, body_b_raw, constraint_point)
            }
            JointKind::Slider {
                slider_axis,
                normal_axis,
                limits_min,
                limits_max,
            } => world.create_slider_constraint(
                body_a_raw,
                body_b_raw,
                slider_axis,
                normal_axis,
                limits_min,
                limits_max,
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
            ),
            JointKind::SwingTwist {
                constraint_position,
                twist_axis,
                plane_axis,
                normal_half_cone_angle,
                plane_half_cone_angle,
                twist_min_angle,
                twist_max_angle,
            } => world.create_swing_twist_constraint(
                body_a_raw,
                body_b_raw,
                constraint_position,
                twist_axis,
                plane_axis,
                normal_half_cone_angle,
                plane_half_cone_angle,
                twist_min_angle,
                twist_max_angle,
            ),
            JointKind::SixDofSlider {
                limit_y_min,
                limit_y_max,
            } => world.create_six_dof_slider(body_a_raw, body_b_raw, limit_y_min, limit_y_max),
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
                )
            }
            JointKind::PathCart {
                track_from,
                track_to,
            } => world.create_path_cart(body_a_raw, body_b_raw, track_from, track_to),
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
