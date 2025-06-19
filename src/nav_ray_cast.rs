// Based on RayMeshHit in bevy_picking

#[cfg(feature = "debug_draw")]
use crate::debug_draw::*;
#[cfg(feature = "debug_draw")]
use bevy::color::palettes::tailwind;
use raven_bvh::prelude::*;
use strum::IntoEnumIterator;

use crate::{
    tile::{mesher::*, nav_mesh::*, *},
    nav::{TileLookup, Nav},
};
use bevy::{
    ecs::system::{
        SystemParam,
        lifetimeless::{Read, Write},
    },
    math::{FloatOrd, bounding::*},
    prelude::*,
};

#[derive(Debug, Clone, Reflect)]
#[reflect(Clone)]
pub struct RayNavHit {
    /// The point of intersection in world space.
    pub point: Vec3,
    /// The normal vector of the triangle at the point of intersection. Not guaranteed to be normalized for scaled meshes.
    // pub normal: Vec3,
    /// The barycentric coordinates of the intersection.
    // pub barycentric_coords: Vec3,
    /// The distance from the ray origin to the intersection point.
    pub distance: f32,
    // The vertices of the triangle that was hit.
    pub tile_coord: UVec2,

    pub polygon_index: u16,
    // The index of the triangle that was hit.
    //pub triangle_index: Option<usize>,
    // The index of the node in the tile.
    //pub(crate) polygon_index: usize,
}

// Based on Bevy's MeshRayCast
// #[derive(SystemParam)]
// pub struct NavRayCast<'w, 's> {
//     pub hits: Local<'s, Vec<(FloatOrd, (Entity, RayNavHit))>>,
//     pub output: Local<'s, Vec<(Entity, RayNavHit)>>,
//     pub culled_list: Local<'s, Vec<(FloatOrd, Entity)>>,

//     pub culled_query: Query<
//         'w,
//         's,
//         (
//             Entity,
//             Read<TileMeshAabb>,
//             Read<TileWaymap>,
//             Read<GlobalTransform>,
//         ),
//         With<Tile>,
//     >,
//     pub tile_query: Query<
//         'w,
//         's,
//         (
//             Read<Tile>,
//             Read<TileNavMesh>,
//             Read<MeshBvh>,
//             Read<GlobalTransform>,
//         ),
//     >,
//     pub waymap_query: Query<'w, 's, (Read<Nav>, Read<GlobalTransform>)>,
//     pub bvhs: Res<'w, Assets<Bvh>>,
//     #[cfg(feature = "debug_draw")]
//     pub gizmos: Gizmos<'w, 's, RavenGizmos>,
// }

// impl<'w, 's> NavRayCast<'w, 's> {
//     /// Casts the `ray` into the world and returns a sorted list of intersections, nearest first.
//     /// Note: Currently returns all Waymap, not limited to 1
//     // TODO: could filter by waymap
//     pub fn cast_aabb(&mut self, aabb_cast: AabbCast3d) -> Option<&(Entity, RayNavHit)> {
//         self.hits.clear();
//         self.output.clear();

//         let ray = aabb_cast.ray;
//         #[cfg(feature = "debug_draw")]
//         {
//             self.gizmos
//                 .cuboid(aabb3d_global(&aabb_cast.aabb), tailwind::BLUE_800);
//             self.gizmos.line(
//                 ray.origin.into(),
//                 ray.get_point(ray.max).into(),
//                 tailwind::RED_500,
//             );
//         }

//         // Check all entities to see if the ray intersects the AABB. Use this to build a short list
//         // of entities that are in the path of the ray.
//         let (aabb_hits_tx, aabb_hits_rx) = crossbeam_channel::unbounded::<(FloatOrd, Entity)>();
//         self.culled_query
//             .par_iter()
//             .for_each(|(e, aabb, tile_waymap, transform)| {
//                 let (ray, dir_scale) = ray.to_local(transform);
//                 // test vs tile bounding box
//                 if let Some(mut hit) =
//                     ray.aabb_intersection_at(&aabb.grow(aabb_cast.aabb.half_size()))
//                 {
//                     hit /= dir_scale; // Convert back to world-space distance                                                 
//                     aabb_hits_tx.send((FloatOrd(hit), e)).ok();
//                 }
//             });
//         *self.culled_list = aabb_hits_rx.try_iter().collect();

//         // Sort by the distance along the ray.
//         self.culled_list.sort_by_key(|(aabb_near, _)| *aabb_near);

//         // Perform ray casts against the culled entities.
//         let mut nearest_blocking_hit = FloatOrd(f32::INFINITY);
//         self.culled_list.iter().for_each(|(aabb_near, entity)| {
//             let Ok((tile_coord, tile, bvh, transform)) = self.tile_query.get(*entity) else {
//                 unreachable!("Tile should exist in the tile query")
//             };

//             // Is it even possible the mesh could be closer than the current best?
//             if *aabb_near > nearest_blocking_hit {
//                 return;
//             }

//             let mut out_polygon = None;
//             let mut out_distance = f32::INFINITY;
//             //let (ray, dir_scale) = ray.to_local(transform);
//             // test vs poly boxs
//             // TODO: not testing along the ray, but in the box here
//             for (poly_i, poly) in tile.polygons.iter().enumerate() {
//                 let center = aabb_cast.aabb.center().into();
//                 let closest_point = tile.get_closest_point_in_polygon(poly, center);
//                 let closest_distance = closest_point.distance_squared(center);
//                 if closest_distance < out_distance {
//                     out_distance = closest_distance;
//                     out_polygon = Some((tile_coord, poly_i as u16, closest_point));
//                 }
//             }

//             // if let Some(distance) = ray.aabb_intersection_at(&poly.bounds) {
//             //     let d = FloatOrd(distance);
//             //     if d < nearest_blocking_hit {
//             //         // #[cfg(feature = "debug_draw")]
//             //         // self.gizmos.cuboid(aabb3d_transform(&poly.bounds.grow(Vec3::splat(0.1)), transform, ), tailwind::GREEN_300);

//             //         let point = ray.get_point(distance);
//             //         // The ray does not intersect the mesh bounding box.
//             //         self.hits.push((
//             //             d,
//             //             (
//             //                 *entity,
//             //                 RayNavHit {
//             //                     point: transform.affine().transform_point3a(point).into(),
//             //                     distance,
//             //                 },
//             //             ),
//             //         ));
//             //         nearest_blocking_hit = d.min(nearest_blocking_hit);
//             //     }
//             // }
//         });

//         self.hits.retain(|(dist, _)| *dist <= nearest_blocking_hit);
//         self.hits.sort_by_key(|(k, _)| *k);
//         let hits = self.hits.iter().map(|(_, (e, i))| (*e, i.to_owned()));
//         self.output.extend(hits);
//         self.output.iter().next()
//     }

//     /// Casts the `ray` into the world and returns a sorted list of intersections, nearest first.
//     /// Note: Currently returns all Waymap, not limited to 1
//     pub fn cast_ray(&mut self, ray: &RayCast3d) -> Option<&(Entity, RayNavHit)> {
//         self.hits.clear();
//         self.output.clear();

//         #[cfg(feature = "debug_draw")]
//         self.gizmos.line(
//             ray.origin.into(),
//             ray.get_point(ray.max).into(),
//             tailwind::RED_500,
//         );

//         // TODO: Test this vs a tlas that only updates when any tile in the waymap changes.

//         // Check all entities to see if the ray intersects the AABB. Use this to build a short list
//         // of entities that are in the path of the ray.
//         let (aabb_hits_tx, aabb_hits_rx) = crossbeam_channel::unbounded::<(FloatOrd, Entity)>();
//         self.culled_query
//             .par_iter()
//             .for_each(|(e, aabb, tile_waymap, transform)| {
//                 let (ray, dir_scale) = ray.to_local(transform);
//                 // test vs tile bounding box
//                 if let Some(mut distance) = ray.aabb_intersection_at(aabb) {
//                     distance /= dir_scale; // Convert back to world-space distance
//                     aabb_hits_tx.send((FloatOrd(distance), e)).ok();
//                 }
//             });
//         *self.culled_list = aabb_hits_rx.try_iter().collect();

//         // Sort by the distance along the ray.
//         self.culled_list.sort_by_key(|(aabb_near, _)| *aabb_near);

//         // Perform ray casts against the culled entities.
//         let mut nearest_blocking_hit = FloatOrd(f32::INFINITY);
//         self.culled_list.iter().for_each(|(aabb_near, entity)| {
//             let Ok((tile_coord, tile, bvh, transform)) = self.tile_query.get(*entity) else {
//                 return;
//             };

//             // Is it even possible the mesh could be closer than the current best?
//             if *aabb_near > nearest_blocking_hit {
//                 return;
//             }

//             let (tile_ray, dir_scale) = ray.to_local(transform);

//             // test vs bvh
//             let bvh = self.bvhs.get(&bvh.0).unwrap();
//             if let Some(mut hit) = ray.intersect_bvh(bvh) {
//                 hit.distance /= dir_scale; // Convert back to world-space distance

//                 self.hits.push((
//                     FloatOrd(hit.distance),
//                     (
//                         *entity,
//                         RayNavHit {
//                             point: ray.get_point(hit.distance).into(),
//                             distance: hit.distance,
//                             tile_coord: tile_coord.0,
//                             polygon_index: hit.tri_index as u16,
//                         },
//                     ),
//                 ));
//             }

//             // test vs poly boxs
//             // TODO: not testing along the ray, but in the box here
//             // for (poly_i, poly) in tile.polygons.iter().enumerate() {
//             //     let p = tile_ray.origin.into();
//             //     let closest_point = tile.get_closest_point_in_polygon(poly, p);
//             //     let closest_distance = FloatOrd(closest_point.distance_squared(p));
//             //     if closest_distance < nearest_blocking_hit {
//             //         let distance = p.distance(closest_point);
//             //         // out_polygon = Some((tile_coord, poly_i as u16, closest_point));

//             //         self.hits.push((
//             //             closest_distance,
//             //             (
//             //                 *entity,
//             //                 RayNavHit {
//             //                     point: ray.get_point(distance).into(),
//             //                     distance,
//             //                     tile_coord: tile_coord.0,
//             //                     polygon_index: poly_i as u16,
//             //                 },
//             //             ),
//             //         ));
//             //         nearest_blocking_hit = closest_distance.min(nearest_blocking_hit);
//             //     }

//             //     // #[cfg(feature = "debug_draw")]
//             //     // self.gizmos.cuboid(aabb3d_transform(&poly.bounds.grow(Vec3::splat(0.1)), transform, ), tailwind::GREEN_300);

//             //     // let point = ray.get_point(distance);
//             //     // The ray does not intersect the mesh bounding box.
//             // }
//         });

//         self.hits.retain(|(dist, _)| *dist <= nearest_blocking_hit);
//         self.hits.sort_by_key(|(k, _)| *k);
//         let hits = self.hits.iter().map(|(_, (e, i))| (*e, i.to_owned()));
//         self.output.extend(hits);
//         self.output.iter().next()
//     }
// }

/// Utility functions for connecting and removing links between tiles.
#[derive(SystemParam)]
pub struct NavLinks<'w, 's> {
    #[doc(hidden)]
    pub tile_info_query: Query<
        'w,
        's,
        (
            Read<Tile>,
            Read<TileWaymap>,
            Read<GlobalTransform>,
            Has<TileNavMesh>,
        ),
        With<Tile>,
    >,
    #[doc(hidden)]
    pub tile_nav_mesh_query: Query<'w, 's, (Write<TileNavMesh>, Read<GlobalTransform>), With<Tile>>,
    #[doc(hidden)]
    pub waymap_query: Query<'w, 's, (Read<Nav>, Read<TileLookup>, Read<GlobalTransform>)>,
}

impl<'w, 's> NavLinks<'w, 's> {
    /// Connects the links of the tile to its neighbours.    
    pub fn update_tile_links(&mut self, entity: Entity, tile: &mut TileNavMesh) {
        let (tile_coord, TileWaymap(waymap_entity), tile_trans, previous_tile_existed) =
            self.tile_info_query.get_mut(entity).unwrap();
        let (waymap, lookup, _trans) = self.waymap_query.get(*waymap_entity).unwrap();

        // Connect neighbours.
        let step_height = waymap.step_height as f32 * waymap.cell_height;
        for direction in EdgeConnectionDirection::iter() {
            if let Some(neighbour_coord) = direction.offset(**tile_coord) {
                if let Some(neighbour_entity) = lookup.get(&neighbour_coord) {
                    if let Ok((mut neighbour, neighbour_trans)) =
                        self.tile_nav_mesh_query.get_mut(*neighbour_entity)
                    {
                        let opposite_direction = direction.flip();
                        connect_external_links(
                            tile,
                            tile_trans,
                            &neighbour,
                            neighbour_trans,
                            direction,
                            opposite_direction,
                            false,
                            step_height,
                        );
                        connect_external_links(
                            &mut neighbour,
                            neighbour_trans,
                            &tile,
                            tile_trans,
                            opposite_direction,
                            direction,
                            previous_tile_existed,
                            step_height,
                        );
                    }
                }
            }
        }
    }

    /// remove ref tile from neighbours
    pub fn remove_tile(&mut self, entity: Entity) {
        let (tile_coord, TileWaymap(waymap_entity), _, previous_tile_existed) =
            self.tile_info_query.get_mut(entity).unwrap();
        let (_waymap, lookup, _waymap_trans) = self.waymap_query.get(*waymap_entity).unwrap();

        if !previous_tile_existed {
            // If the tile did not exist before, we do not need to remove links.
            return;
        }

        for direction in EdgeConnectionDirection::iter() {
            if let Some(neighbour_coord) = direction.offset(**tile_coord) {
                if let Some(neighbour_entity) = lookup.get(&neighbour_coord) {
                    if let Ok((mut neighbour, _)) =
                        self.tile_nav_mesh_query.get_mut(*neighbour_entity)
                    {
                        neighbour.remove_links_to_direction(direction.flip());
                    }
                }
            }
        }
    }

    pub fn find_closest_polygon_in_box(
        &self,
        waymap: Entity,
        center: Vec3,
        half_extents: f32,
    ) -> Option<(UVec2, u16, Vec3)> {
        let (waymap, lookup, waymap_trans) = self.waymap_query.get(waymap).unwrap();
        let min = center - half_extents;
        let max = center + half_extents;

        let min_tile = waymap.get_tile_containing_position(min.xz(), waymap_trans);
        let max_tile = waymap.get_tile_containing_position(max.xz(), waymap_trans);

        let mut out_polygon = None;
        let mut out_distance = f32::INFINITY;
        for x in min_tile.x..=max_tile.x {
            for y in min_tile.y..=max_tile.y {
                let tile_coords = UVec2::new(x, y);
                let Some(tile_entity) = lookup.get(&tile_coords) else {
                    continue;
                };
                let Ok((tile, _tile_trans)) = self.tile_nav_mesh_query.get(*tile_entity) else {
                    continue;
                };
                for (poly_i, polygon) in tile.polygons.iter().enumerate() {
                    let closest_point = tile.get_closest_point_in_polygon(polygon, center);
                    let closest_distance = closest_point.distance_squared(center);

                    if closest_distance < out_distance {
                        out_distance = closest_distance;
                        out_polygon = Some((tile_coords, poly_i as u16, closest_point));
                    }
                }
            }
        }

        out_polygon
    }
}
