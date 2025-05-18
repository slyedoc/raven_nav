use crate::agent::*;
use crate::character::*;
use crate::island::Island;

use bevy::prelude::*;

/// Will have [`Agents`] and [`Characters`] updated when agents and characters are added
#[derive(Component, Reflect)]
#[require(ArchipelagoAgents, ArchipelagoCharacters, ArchipelagoIslands)]
pub struct Archipelago {
    /// The options for sampling agent and target points.
    pub point_sample_distance: PointSampleDistance,
    /// The distance that an agent will consider avoiding another agent.
    pub neighbourhood: f32,
    // The time into the future that collisions with other agents should be
    /// avoided.
    pub avoidance_time_horizon: f32,
    /// The time into the future that collisions with obstacles should be
    /// avoided.
    pub obstacle_avoidance_time_horizon: f32,
    /// The avoidance responsibility to use when an agent has reached its target.
    /// A value of 1.0 is the default avoidance responsibility. A value of 0.0
    /// would mean no avoidance responsibility, but a value of 0.0 is invalid and
    /// may panic. This should be a value between 0.0 and 1.0.
    pub reached_destination_avoidance_responsibility: f32,
}

impl Default for Archipelago {
    fn default() -> Self {
        Self::from_agent_radius(0.5)
    }
}

impl Archipelago {
    pub fn from_agent_radius(radius: f32) -> Self {
        Self {
            neighbourhood: 10.0 * radius,
            avoidance_time_horizon: 1.0,
            obstacle_avoidance_time_horizon: 0.5,
            reached_destination_avoidance_responsibility: 0.1,
            point_sample_distance: PointSampleDistance {
                horizontal_distance: 0.2 * radius,
                distance_above: 0.5 * radius,
                distance_below: radius,
                vertical_preference_ratio: 2.0,
            },
        }
    }
}

/// Managed list of agents in the archipelago.
#[derive(Component, Default, Debug, Reflect)]
#[relationship_target(relationship = AgentArchipelago)]
pub struct ArchipelagoAgents(Vec<Entity>);

/// Managed list of characters in the archipelago.
#[derive(Component, Default, Debug, Reflect)]
#[relationship_target(relationship = CharacterArchipelago)]
pub struct ArchipelagoCharacters(Vec<Entity>);

/// Managed list of agents in the archipelago.
#[derive(Component, Default, Debug, Reflect)]
#[relationship_target(relationship = Island)]
pub struct ArchipelagoIslands(Vec<Entity>);


#[derive(Reflect, Debug, PartialEq, Clone)]
pub struct PointSampleDistance {
    /// The horizontal distance that a node may be sampled. If a sample point is
    /// further than this distance away horizontally, it will be ignored.
    pub horizontal_distance: f32,

    /// The vertical distance above the query point that a node may be sampled.
    ///
    /// If a sample point is further above than this distance, it will be
    /// ignored. This value must be greater than [`Self::distance_below`].
    pub distance_above: f32,

    /// The vertical distance below the query point that a node may be sampled.
    ///
    /// If a sample point is further below than this distance, it will be
    /// ignored. This value must be greater than [`Self::distance_above`].
    pub distance_below: f32,

    /// The ratio between the vertical and the horizontal distances to prefer.
    /// For example, if this value is 2.0, then a sample point directly below
    /// the query point 1.9 units away will be selected over a sample point 1.0
    /// unit away horizontally. This value must be positive.
    pub vertical_preference_ratio: f32,
}
