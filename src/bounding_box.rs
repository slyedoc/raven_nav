use bevy::{math::bounding::Aabb3d, prelude::*};


#[derive(Component, Clone, Debug, Deref, DerefMut)]
pub(crate) struct Bounding(pub Aabb3d);
