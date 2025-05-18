use bevy::prelude::*;

use crate::archipelago::ArchipelagoAgents;


#[derive(Component, Reflect)]
#[require(
    Transform,
    Velocity,
    AgentTarget,
    AgentState,
    AgentDesiredVelocity,
    AgentSettings,
)]
pub struct Agent;

/// Ref to Archipelago, added if not present when Agent is added
#[derive(Component, Debug, Reflect)]
#[relationship(relationship_target = ArchipelagoAgents)]
pub struct AgentArchipelago(pub Entity);

/// The settings for an agent. See [`crate::AgentBundle`] for required related
/// components.
#[derive(Component, Reflect, Debug)]
pub struct AgentSettings {
    /// The radius of the agent.
    pub radius: f32,
    /// The speed the agent prefers to move at. This should often be set lower
    /// than the [`Self::max_speed`] to allow the agent to "speed up" in order to
    /// get out of another agent's way.
    pub desired_speed: f32,
    /// The max speed of an agent.
    pub max_speed: f32,
}

impl Default for AgentSettings {
    fn default() -> Self {
        Self {
            radius: 0.5,
            desired_speed: 1.0,
            max_speed: 1.0,
        }
    }
}

#[derive(Component, Default)]
pub struct Velocity(pub Vec3);

#[derive(Component, Default)]
pub enum AgentTarget {
    #[default]
    None,
    Point(Vec3),
    Entity(Entity),
}

/// The state of an agent.
///
/// This does not control an agent's state and is just used to report the
/// agent's state.
#[derive(Component, Clone, Copy, PartialEq, Eq, Debug, Default)]
pub enum AgentState {
    /// The agent is idle, due to not having a target. Note this does not mean
    /// that they are motionless. An agent will still avoid nearby agents.
    #[default]
    Idle,
    /// The agent has reached their target. The agent may resume moving if the
    /// target moves or otherwise changes.
    ReachedTarget,
    /// The agent has a path and is moving towards their target.
    Moving,
    /// The agent is not on a nav mesh.
    AgentNotOnNavMesh,
    /// The target is not on a nav mesh.
    TargetNotOnNavMesh,
    /// The agent has a target but cannot find a path to it.
    NoPath,
}

#[derive(Component, Default)]
pub struct AgentDesiredVelocity(pub Vec3);





