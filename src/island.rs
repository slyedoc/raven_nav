use bevy::prelude::*;

use crate::{nav_mesh::LandmassNavMesh, prelude::ArchipelagoIslands};

#[derive(Component, Reflect)]
#[require(Transform)]
#[relationship(relationship_target = ArchipelagoIslands)]
#[component(immutable)]
pub struct Island(pub Entity);

#[derive(Component, Clone, Debug)]
pub struct NavMeshHandle(pub Handle<LandmassNavMesh>);

