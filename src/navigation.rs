use core::arch;

use bevy::{color::palettes::tailwind, gizmos, math::bounding::BoundingVolume, prelude::*};
use crate::{
    agent::*, archipelago::*, nav_mesh::{self, NavigationMesh}, prelude::RavenGizmos, tile::*, Bounding
};

#[derive(Default, Resource, Deref, DerefMut)]
pub struct PathingResults(pub Vec<PathingResult>);

pub struct PathingResult {
  /// The agent that searched for the path.
  pub agent: Entity,
  /// Whether the pathing succeeded or failed.
  pub success: bool,
  /// The number of "nodes" explored while finding the path. Note this may be
  /// zero if the start and end point are known to be disconnected.
  pub explored_nodes: u32,
}

#[derive(PartialEq, Eq, Debug, Clone, Copy, Hash, PartialOrd, Ord)]
pub(crate) struct NodeRef {
  /// The tile of the node.
  pub(crate) tile_entity: Entity,
  /// The index of the node in the tile.
  pub(crate) polygon_index: usize,
}


///
pub fn update_navigation(
    mut pathing_results: ResMut<PathingResults>,
    mut archepelago_query: Query<(&Archipelago, &ArchipelagoAgents, &TileLookup, &ArchipelagoTiles)>,
    mut agent_query: Query<(&GlobalTransform, &AgentTarget, &AgentState ), With<Agent>>,
    tile_query: Query<(&Bounding, &GlobalTransform, &TileNavMesh)>,
    nav_mesh: Res<Assets<NavigationMesh>>,
    mut gizmos: Gizmos<RavenGizmos>,
    time: Res<Time>,
) {
    let _dt = time.delta_secs();
    pathing_results.clear();

    for (archipelago, agents, _tile_lookup, archipelago_tiles) in archepelago_query.iter_mut() {
        for agent_entity in agents.iter() {
            let (agent_transform, _agent_target, _agent_state) = agent_query.get_mut(agent_entity).unwrap();
            let agent_position = agent_transform.translation();

            // Sample Point

            let point = Vec3::new(0.0, 0.0, 0.0);

            let mut best_point = None;
            for tile_entity in archipelago_tiles.iter() {
                let Ok((_tile_bounding, tile_transform, tile_nav_mesh)) = tile_query.get(tile_entity) else {
                    // tile has no nav mesh yet
                    continue;
                };
                let nav_mesh = nav_mesh.get(&tile_nav_mesh.0).unwrap();                                                
                let relative_point = tile_transform.affine().inverse().transform_point(point);                

                // find the closest point on the nav mesh
                let Some((sampled_point, sampled_node)) = nav_mesh.sample_point(relative_point, &archipelago.agent_options) else {
                    continue;
                };

                let distance = relative_point.distance_squared(sampled_point);
                println!("Sampled point: {:?}, distance: {:?}", sampled_point, distance);

                match best_point {
                    Some((best_distance, _)) if distance >= best_distance => continue,
                    _ => {}
                }

                best_point = Some((
                    distance,
                    (
                        tile_transform.transform_point(sampled_point),
                        NodeRef { tile_entity, polygon_index: sampled_node },
                    ),
                ));
            }

            if let Some((_, (best, _))) = best_point {
                // found the best point
                gizmos.line(agent_position, best, tailwind::GREEN_800);
            }


        }
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