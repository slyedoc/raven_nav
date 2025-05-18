use std::sync::{Arc, RwLock};

use bevy::{
    platform::collections::{HashMap, HashSet},
    prelude::*,
    tasks::Task,
};

use crate::tiles::NavMeshTiles;

#[derive(Default, Resource, Deref, DerefMut)]
pub struct TileAffectors(pub HashMap<UVec2, HashSet<Entity>>);

/// Set of all tiles that need to be rebuilt.
#[derive(Default, Resource)]
pub struct DirtyTiles(pub HashSet<UVec2>);

#[derive(Resource, Default)]
pub struct ActiveGenerationTasks(pub Vec<Task<Option<UVec2>>>);
impl ActiveGenerationTasks {
    pub fn len(&self) -> usize {
        self.0.len()
    }
    pub fn is_empty(&self) -> bool {
        self.0.is_empty()
    }
}


/// Generation ticker for tiles.
///
/// Used to keep track of if the existing tile is newer than the one we are trying to insert in [build_tile]. This could happen if we go from having a lot of triangles to very few.
#[derive(Default, Resource)]
pub struct GenerationTicker(pub u64);
