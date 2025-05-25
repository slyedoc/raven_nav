use std::time::Instant;

#[cfg(feature = "debug_draw")]
use bevy::color::palettes::tailwind;
#[cfg(feature = "hot")]
use bevy_simple_subsecond_system::hot;

use crate::{
    Bounding,
    agent::{self, *},
    archipelago::{self, *},
    bounding_box::BoundingAabb3d,
    nav_mesh::NavigationMesh,
    tile::{self, *},
};
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

#[derive(PartialEq, Eq, Debug, Clone, Copy, Hash, PartialOrd, Ord)]
pub(crate) struct NodeRef {
    /// The tile of the node.
    pub(crate) tile_entity: Entity,
    /// The index of the node in the tile.
    pub(crate) polygon_index: usize,
}

#[hot]
pub fn update_navigation(
    mut pathing_results: ResMut<PathingResults>,
    mut archepelago_query: Query<(
        &Archipelago,
        &ArchipelagoAgents,
        &TileLookup,
        &ArchipelagoTiles,
    )>,
    mut agent_query: Query<(&GlobalTransform, &mut AgentTarget, &AgentState), With<Agent>>,
    tile_query: Query<(&Name, &GlobalTransform, &Bounding, &TileNavMesh)>,
    nav_mesh: Res<Assets<NavigationMesh>>,
    #[cfg(feature = "debug_draw")] mut gizmos: Gizmos<crate::debug_draw::RavenGizmos>,
    time: Res<Time>,
) {
    use bevy::math::bounding::Aabb3d;

    use crate::debug_draw;

    let _dt = time.delta_secs();
    pathing_results.clear();

    let t1 = Instant::now();

    info!("Update Navigation");
    //let point = Vec3::new(0.0, 0.0, 0.0);
    // Sample Point

    for (archipelago, agents, _tile_lookup, archipelago_tiles) in archepelago_query.iter_mut() {
        for agent_entity in agents.iter() {
            let (agent_transform, mut agent_target, _agent_state) =
                agent_query.get_mut(agent_entity).unwrap();

            let point = agent_transform.translation();

            #[cfg(feature = "debug_draw")]
            gizmos.line(
                point,
                match *agent_target {
                    AgentTarget::None => Vec3::ZERO,
                    AgentTarget::Point(vec3) => vec3,
                    AgentTarget::Entity(_entity) => Vec3::Y * 100.0, // jupst point up for now
                },
                tailwind::RED_800,
            );

            let mut best_point = None;

            // each each tile in the archipelago
            for tile_entity in archipelago_tiles.iter() {
                let Ok((tile_name, tile_transform, tile_bounding, tile_nav_mesh)) =
                    tile_query.get(tile_entity)
                else {
                    // tile has no nav mesh yet
                    info_once!("Tile {tile_entity} has no nav mesh yet");
                    continue;
                };
                let nav_mesh = nav_mesh.get(&tile_nav_mesh.0).unwrap();
                            
                // find the closest point on the nav mesh
                let Some((world_point, sampled_node)) =
                    nav_mesh.sample_point(point, tile_transform, tile_bounding, &archipelago.agent_options, &mut gizmos)
                else {
                    continue;
                };                

                let distance = point.distance_squared(world_point);
                // skip if we got a worse point
                match best_point {
                    Some((best_distance, _)) if distance < best_distance => {
                        best_point = Some((
                            distance,
                            (
                                world_point,
                                NodeRef {
                                    tile_entity,
                                    polygon_index: sampled_node,
                                },
                            ),
                        ));
                    },
                    _ => {}
                };
                
            }

            if let Some((dist, (best, node))) = best_point {
                info!(
                    "Best point: {:?} to {:?} on {:?}: {:?}",
                    point, best, node, dist
                );
                *agent_target = AgentTarget::Point(best);

                // found the best point
                #[cfg(feature = "debug_draw")]
                gizmos.line(point, best, tailwind::GREEN_800);
            }
        }

        let t2 = Instant::now();
        let elapsed = t2.duration_since(t1);
        info!("Update Nav: {:?}", elapsed);
    }

    // TODO: make the edge_link_distance configurable.
    // let (invalidated_boundary_links, invalidated_islands) =
    //  self.nav_data.update(/* edge_link_distance= */ 0.01);

    // let mut agent_id_to_agent_node = HashMap::new();
    // let mut agent_id_to_target_node = HashMap::new();

    // for (agent_id, agent) in self.agents.iter() {
    //   let agent_node_and_point = match self.nav_data.sample_point(
    //     CS::to_landmass(&agent.position),
    //     &self.agent_options.point_sample_distance,
    //   ) {
    //     None => continue,
    //     Some(node_and_point) => node_and_point,
    //   };
    //   let inserted =
    //     agent_id_to_agent_node.insert(agent_id, agent_node_and_point).is_none();
    //   debug_assert!(inserted);

    //   if let Some(target) = &agent.current_target {
    //     let target_node_and_point = match self.nav_data.sample_point(
    //       CS::to_landmass(target),
    //       &self.agent_options.point_sample_distance,
    //     ) {
    //       None => continue,
    //       Some(node_and_point) => node_and_point,
    //     };

    //     let inserted = agent_id_to_target_node
    //       .insert(agent_id, target_node_and_point)
    //       .is_none();
    //     debug_assert!(inserted);
    //   }
    // }

    // let mut character_id_to_nav_mesh_point = HashMap::new();
    // for (character_id, character) in self.characters.iter() {
    //   let character_point = match self.nav_data.sample_point(
    //     CS::to_landmass(&character.position),
    //     &self.agent_options.point_sample_distance,
    //   ) {
    //     None => continue,
    //     Some(point_and_node) => point_and_node.0,
    //   };
    //   character_id_to_nav_mesh_point.insert(character_id, character_point);
    // }

    // let mut agent_id_to_follow_path_indices = HashMap::new();

    // for (agent_id, agent) in self.agents.iter_mut() {
    //   let agent_node = agent_id_to_agent_node
    //     .get(&agent_id)
    //     .map(|node_and_point| node_and_point.1);
    //   let target_node = agent_id_to_target_node
    //     .get(&agent_id)
    //     .map(|node_and_point| node_and_point.1);
    //   match does_agent_need_repath(
    //     agent,
    //     agent_node,
    //     target_node,
    //     &invalidated_boundary_links,
    //     &invalidated_islands,
    //   ) {
    //     RepathResult::DoNothing => {}
    //     RepathResult::FollowPath(
    //       agent_node_in_corridor,
    //       target_node_in_corridor,
    //     ) => {
    //       agent_id_to_follow_path_indices.insert(
    //         agent_id,
    //         (agent_node_in_corridor, target_node_in_corridor),
    //       );
    //     }
    //     RepathResult::ClearPathNoTarget => {
    //       agent.state = AgentState::Idle;
    //       agent.current_path = None;
    //     }
    //     RepathResult::ClearPathBadAgent => {
    //       agent.state = AgentState::AgentNotOnNavMesh;
    //       agent.current_path = None;
    //     }
    //     RepathResult::ClearPathBadTarget => {
    //       agent.state = AgentState::TargetNotOnNavMesh;
    //       agent.current_path = None;
    //     }
    //     RepathResult::NeedsRepath => {
    //       agent.current_path = None;

    //       let path_result = pathfinding::find_path(
    //         &self.nav_data,
    //         agent_node.unwrap(),
    //         target_node.unwrap(),
    //         &agent.override_node_type_to_cost,
    //       );

    //       self.pathing_results.push(PathingResult {
    //         agent: agent_id,
    //         success: path_result.path.is_some(),
    //         explored_nodes: path_result.stats.explored_nodes,
    //       });

    //       let Some(new_path) = path_result.path else {
    //         agent.state = AgentState::NoPath;
    //         continue;
    //       };

    //       agent_id_to_follow_path_indices.insert(
    //         agent_id,
    //         (PathIndex::from_corridor_index(0, 0), new_path.last_index()),
    //       );
    //       agent.current_path = Some(new_path);
    //     }
    //   }
    // }

    // for (agent_id, agent) in self.agents.iter_mut() {
    //   let path = match &agent.current_path {
    //     None => {
    //       agent.current_desired_move = CS::from_landmass(&Vec3::ZERO);
    //       continue;
    //     }
    //     Some(path) => path,
    //   };

    //   let agent_point = agent_id_to_agent_node
    //     .get(&agent_id)
    //     .expect("Agent has a path, so should have a valid start node")
    //     .0;
    //   let target_point = agent_id_to_target_node
    //     .get(&agent_id)
    //     .expect("Agent has a path, so should have a valid target node")
    //     .0;

    //   let &(agent_node_index_in_corridor, target_node_index_in_corridor) =
    //     agent_id_to_follow_path_indices.get(&agent_id).expect(
    //       "Any agent with a path must have its follow path indices filled out.",
    //     );

    //   let next_waypoint = path.find_next_point_in_straight_path(
    //     &self.nav_data,
    //     agent_node_index_in_corridor,
    //     agent_point,
    //     target_node_index_in_corridor,
    //     target_point,
    //   );

    //   if agent.has_reached_target(
    //     path,
    //     &self.nav_data,
    //     next_waypoint,
    //     (target_node_index_in_corridor, target_point),
    //   ) {
    //     agent.current_desired_move = CS::from_landmass(&Vec3::ZERO);
    //     agent.state = AgentState::ReachedTarget;
    //   } else {
    //     let desired_move = (next_waypoint.1 - CS::to_landmass(&agent.position))
    //       .xy()
    //       .normalize_or_zero()
    //       * agent.desired_speed;

    //     agent.current_desired_move =
    //       CS::from_landmass(&desired_move.extend(0.0));
    //     agent.state = AgentState::Moving;
    //   }
}
