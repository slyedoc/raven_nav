use bevy::prelude::*;
use smallvec::SmallVec;

#[derive(Component, Reflect)]
#[require(NavMeshAffectorRelations)]
pub struct NavMeshAffector;

#[derive(Component, Default, Deref, DerefMut)]
pub struct NavMeshAffectorRelations(pub SmallVec<[UVec2; 4]>);

// Optional component to define the area type of an entity. Setting this to ``None`` means that the entity isn't walkable.
///
/// Any part of the nav-mesh generated from this entity will have this area type. Overlapping areas will prefer the higher area type.
#[derive(Component, Reflect, Default)]
#[reflect(Component)]
pub struct NavMeshAreaType(pub Option<Area>);

#[derive(Default, Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Reflect)]
pub struct Area(pub u16);

/*
*   Neighbours:
*   0: (-1, 0),
*   1: (0, 1),
*   2: (1, 0),
*   3: (0, -1)
*/
