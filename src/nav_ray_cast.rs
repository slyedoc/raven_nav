

#[cfg(feature = "debug_draw")]
use crate::debug_draw::*;
#[cfg(feature = "debug_draw")]
use bevy::color::palettes::tailwind;

use crate::{
    utils::RayCast3dToSpace, nav_mesh::NavigationMesh, tile::*
};
use bevy::{
    ecs::{
        system::{SystemParam, lifetimeless::Read},
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
    // /// The barycentric coordinates of the intersection.
    // pub barycentric_coords: Vec3,
    /// The distance from the ray origin to the intersection point.
    pub distance: f32,
    // The vertices of the triangle that was hit.
    //pub triangle: Option<[Vec3; 3]>,
    // The index of the triangle that was hit.
    //pub triangle_index: Option<usize>,
    // The index of the node in the tile.
    //pub(crate) polygon_index: usize,
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
    pub culled_query:
        Query<'w, 's, (Entity, Read<TileMeshAabb>, Read<GlobalTransform>), With<Tile>>,
    #[doc(hidden)]
    pub tile_query: Query<'w, 's, (Read<TileNavMesh>, Read<GlobalTransform>), With<Tile>>,
}

impl<'w, 's> NavRayCast<'w, 's> {
    /// Casts the `ray` into the world and returns a sorted list of intersections, nearest first.
    pub fn cast_ray(&mut self, ray: RayCast3d) -> Option<&(Entity, RayNavHit)> {
        self.hits.clear();
        self.output.clear();
        
        // #[cfg(feature = "debug_draw")]
        // self.gizmos.line(
        //     ray.origin.into(),
        //     ray.get_point(ray.max).into(),
        //     tailwind::RED_500,
        // );

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
                        
            // Is it even possible the mesh could be closer than the current best?
            if *aabb_near > nearest_blocking_hit {
                return;
            }

            // Does the mesh handle resolve?
            let Some(mesh) = self.meshes.get(&tile_nav_mesh.0) else {
                return;
            };

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

            let ray = ray.to_space(transform);
            // test vs poly boxs
            for poly in &mesh.polygons {
                if let Some(distance) = ray.aabb_intersection_at(&poly.bounds) {
                    let d = FloatOrd(distance);
                    if d < nearest_blocking_hit {
                        // #[cfg(feature = "debug_draw")]
                        // self.gizmos.cuboid(aabb3d_transform(&poly.bounds.grow(Vec3::splat(0.1)), transform, ), tailwind::GREEN_300);

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
                }
            }

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