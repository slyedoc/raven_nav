use bevy::prelude::*;

use crate::{agent::Velocity, nav::WaymapCharacters};

#[derive(Component, Reflect)]
#[require(Transform, Velocity, CharacterSettings)]
pub struct Character;

/// Ref to Waymap, added if not present when Character is added
#[derive(Component, Debug, Reflect)]
#[relationship(relationship_target = WaymapCharacters)]
pub struct CharacterWaymap(pub Entity);

/// A character's settings. See [`crate::CharacterBundle`] for required related
/// components.
#[derive(Component, Debug, Reflect)]
pub struct CharacterSettings {
    /// The radius of the character.
    pub radius: f32,
}

impl Default for CharacterSettings {
    fn default() -> Self {
        Self { radius: 0.5 }
    }
}
