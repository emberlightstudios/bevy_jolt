//! Bevy schedule wiring for the Jolt physics world.

use bevy::prelude::*;

use crate::physics_world::JoltWorld;

/// Bevy resource owning the Jolt physics world.
#[derive(Resource)]
pub struct JoltPhysicsWorld {
    pub physics_world: JoltWorld,
}

impl FromWorld for JoltPhysicsWorld {
    fn from_world(_world: &mut World) -> Self {
        Self {
            physics_world: JoltWorld::new(),
        }
    }
}

/// Adds the Jolt physics world resource to the app.
pub struct JoltPlugin;

impl Plugin for JoltPlugin {
    fn build(&self, app: &mut App) {
        app.init_resource::<JoltPhysicsWorld>();
    }
}
