use std::time::Instant;

#[cfg(feature = "debug_draw")]
use crate::debug_draw::*;
#[cfg(feature = "debug_draw")]
use bevy::color::palettes::tailwind;
use bevy_inspector_egui::egui::debug_text::print;
#[cfg(feature = "hot")]
use bevy_simple_subsecond_system::hot;

use crate::{
    agent::{self, *},
    archipelago::{self, *},
    bounding_box::*,
    nav_mesh::{self, NavigationMesh},
    tile::{self, *},
    tiles::NavMeshTile,
};
use bevy::{
    ecs::{
        system::{SystemParam, lifetimeless::Read},
        world,
    },
    gizmos,
    math::{FloatOrd, bounding::*},
    picking::mesh_picking::ray_cast::ray_aabb_intersection_3d,
    prelude::*,
};

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

#[derive(Debug, Clone, Reflect)]
#[reflect(Clone)]
pub struct RayNavHit {
    /// The point of intersection in world space.
    pub point: Vec3,
    /// The normal vector of the triangle at the point of intersection. Not guaranteed to be normalized for scaled meshes.
    // pub normal: Vec3,
    // /// The barycentric coordinates of the intersection.
    // pub barycentric_coords: Vec3,
    /// The distance from the ray origin to the intersection point.
    pub distance: f32,
    // The vertices of the triangle that was hit.
    //pub triangle: Option<[Vec3; 3]>,
    // The index of the triangle that was hit.
    //pub triangle_index: Option<usize>,
}

// Based on Bevy's MeshRayCast
#[derive(SystemParam)]
pub struct NavRayCast<'w, 's> {
    #[doc(hidden)]
    pub meshes: Res<'w, Assets<NavigationMesh>>,
    #[cfg(feature = "debug_draw")]
    pub gizmos: Gizmos<'w, 's, RavenGizmos>,
    #[doc(hidden)]
    pub hits: Local<'s, Vec<(FloatOrd, (Entity, RayNavHit))>>,
    #[doc(hidden)]
    pub output: Local<'s, Vec<(Entity, RayNavHit)>>,
    #[doc(hidden)]
    pub culled_list: Local<'s, Vec<(FloatOrd, Entity)>>,
    #[doc(hidden)]
    pub culled_query: Query<'w, 's, (Entity, Read<Bounding>, Read<GlobalTransform>), With<Tile>>,
    #[doc(hidden)]
    pub tile_query: Query<'w, 's, (Read<TileNavMesh>, Read<GlobalTransform>), With<Tile>>,
}

impl<'w, 's> NavRayCast<'w, 's> {
    /// Casts the `ray` into the world and returns a sorted list of intersections, nearest first.
    pub fn cast_ray(&mut self, ray: RayCast3d) -> Option<&(Entity, RayNavHit)> {
        self.hits.clear();
        self.output.clear();

        #[cfg(feature = "debug_draw")]
        self.gizmos.line(
            ray.origin.into(),
            ray.get_point(ray.max).into(),
            tailwind::RED_500,
        );

        // Check all entities to see if the ray intersects the AABB. Use this to build a short list
        // of entities that are in the path of the ray.
        let (aabb_hits_tx, aabb_hits_rx) = crossbeam_channel::unbounded::<(FloatOrd, Entity)>();
        self.culled_query
            .par_iter()
            .for_each(|(e, aabb, transform)| {
                let ray = ray.to_space(transform);
                // test vs tile bounding box
                if let Some(distance) = ray.aabb_intersection_at(aabb) {
                    aabb_hits_tx.send((FloatOrd(distance), e)).ok();
                }
            });
        *self.culled_list = aabb_hits_rx.try_iter().collect();

        // Sort by the distance along the ray.
        self.culled_list.sort_by_key(|(aabb_near, _)| *aabb_near);

        // Perform ray casts against the culled entities.
        let mut nearest_blocking_hit = FloatOrd(f32::INFINITY);
        self.culled_list.iter().for_each(|(aabb_near, entity)| {
            let Ok((tile_nav_mesh, transform)) = self.tile_query.get(*entity) else {
                return;
            };
            let ray = ray.to_space(transform);
            // Is it even possible the mesh could be closer than the current best?
            if *aabb_near > nearest_blocking_hit {
                return;
            }

            // Does the mesh handle resolve?
            let Some(mesh) = self.meshes.get(&tile_nav_mesh.0) else {
                return;
            };
            // test vs mesh bounding box
            if let Some(distance) = ray.aabb_intersection_at(&mesh.mesh_bounds) {
                let d = FloatOrd(distance);
                if d < nearest_blocking_hit {
                    #[cfg(feature = "debug_draw")]
                    self.gizmos.cuboid(
                        aabb3d_transform(
                            &mesh.mesh_bounds.clone().grow(Vec3::splat(0.01)),
                            transform,
                        ),
                        tailwind::YELLOW_400,
                    );
                    // TODO: test vs mesh polygons
                    // let Some((world_point, sampled_node)) = nav_mesh.sample_point(
                    //     point,
                    //     tile_transform,
                    //     tile_bounding,
                    //     &archipelago.agent_options,
                    //     &mut gizmos,
                    // ) else {
                    //     continue;
                    // };

                    let point = ray.get_point(distance);
                    // The ray does not intersect the mesh bounding box.
                    self.hits.push((
                        d,
                        (
                            *entity,
                            RayNavHit {
                                point: transform.affine().transform_point3a(point).into(),
                                distance,
                            },
                        ),
                    ));
                    nearest_blocking_hit = d.min(nearest_blocking_hit);
                }
            };

            // Perform the actual ray cast.

            // let intersection = ray_intersection_over_mesh(mesh, &transform, ray, backfaces);
            // if let Some(intersection) = intersection {
            //     let distance = FloatOrd(intersection.distance);
            //     if (settings.early_exit_test)(*entity) && distance < nearest_blocking_hit {
            //         // The reason we don't just return here is because right now we are
            //         // going through the AABBs in order, but that doesn't mean that an
            //         // AABB that starts further away can't end up with a closer hit than
            //         // an AABB that starts closer. We need to keep checking AABBs that
            //         // could possibly contain a nearer hit.
            //         nearest_blocking_hit = distance.min(nearest_blocking_hit);
            //     }
            //     self.hits.push((distance, (*entity, intersection)));
            // };
        });

        self.hits.retain(|(dist, _)| *dist <= nearest_blocking_hit);
        self.hits.sort_by_key(|(k, _)| *k);
        let hits = self.hits.iter().map(|(_, (e, i))| (*e, i.to_owned()));
        self.output.extend(hits);
        self.output.iter().next()
    }
}

pub fn update_navigation(
    mut pathing_results: ResMut<PathingResults>,
    mut arch_query: Query<&ArchipelagoAgents, With<Archipelago>>,
    agent_query: Query<(&GlobalTransform, &AgentState), With<Agent>>,
    mut nav_mesh_cast: NavRayCast,
    mut gizmos: Gizmos<RavenGizmos>,
) {
    pathing_results.clear();

    for archipelago_agents in arch_query.iter_mut() {
        for agent_entity in archipelago_agents.iter() {
            if let Ok((agent_transform, _agent_state)) = agent_query.get(agent_entity) {
                let point = agent_transform.translation();
                let ray = RayCast3d::new(point, Dir3::NEG_Y, 2.);
                if let Some((entity, hit)) = nav_mesh_cast.cast_ray(ray) {
                    gizmos.line(point, hit.point, tailwind::FUCHSIA_500);
                }
            }
        }
    }

    
}

    // TODO: make the edge_link_distance configurable.
    //  let (invalidated_boundary_links, invalidated_islands) =
    //   self.nav_data.update(/* edge_link_distance= */ 0.01);

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

