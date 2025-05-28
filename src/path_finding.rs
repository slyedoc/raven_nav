use bevy::prelude::*;

#[derive(Default, Resource, Deref, DerefMut)]
pub struct PathingResults(pub Vec<PathingResult>);

pub struct PathingResult {
    /// The agent that searched for the path.
    pub _agent: Entity,
    /// Whether the pathing succeeded or failed.
    pub _success: bool,
    /// The number of "nodes" explored while finding the path. Note this may be
    /// zero if the start and end point are known to be disconnected.
    pub _explored_nodes: u32,
}
