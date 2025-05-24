use std::sync::Arc;

use avian3d::parry::{
    math::Isometry, na::Point3, shape::{self,  TypedShape}
};
use bevy::{ecs::entity::EntityHashMap, prelude::*};

/// Add this to any compoent with a Collider to indicate that it is a nav-mesh affector.
#[derive(Component, Default, Reflect)]
pub struct NavMeshAffector(pub Area);

/// The area type of the nav-mesh affector.
#[derive(Default, Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Reflect)]
pub struct Area(pub u16);

/// Marker added to colliders to trigger updating tile affectors, will be removed once updated
#[derive(Component, Reflect)]
#[reflect(Component)]
pub struct UpdateTileAffectors;

// Rest of this file is utility functions for converting colliders to triangles


pub enum GeometryResult<'a> {
    Compound(Vec<(Isometry<f32>, GeometryResult<'a>)>),
    GeometryToConvert(GeometryToConvert),
    Heightfield(&'a shape::HeightField),
    Unsupported,
}

impl From<GeometryToConvert> for GeometryResult<'_> {
    fn from(value: GeometryToConvert) -> Self {
        GeometryResult::GeometryToConvert(value)
    }
}

pub struct GeometryCollection {
    pub transform: GlobalTransform,
    pub geometry_to_convert: GeometryToConvert,
    pub area: Option<Area>,
}

pub enum ColliderType {
    Cuboid(shape::Cuboid),
    Ball(shape::Ball),
    Capsule(shape::Capsule),
    Cylinder(shape::Cylinder),
    Cone(shape::Cone),
    Triangle(shape::Triangle),
    Compound(Vec<ColliderType>),
}

pub enum GeometryToConvert {
    Collider(ColliderType),
    ParryTriMesh(Box<[Point3<f32>]>, Box<[[u32; 3]]>),
}

pub(super) struct TriangleCollection {
    pub(super) transform: GlobalTransform,
    pub(super) triangles: Triangles,
    pub(super) area: Option<Area>,
}

pub struct HeightFieldCollection {
    pub transform: GlobalTransform,
    pub heightfield: Arc<shape::HeightField>, //
    pub area: Option<Area>,
}

pub(super) enum Triangles {
    Triangle([Vec3; 3]),
    TriMesh(Box<[Vec3]>, Box<[[u32; 3]]>),
}

impl Default for Triangles {
    fn default() -> Self {
        Self::TriMesh(Box::new([]), Box::new([]))
    }
}

impl Triangles {
    fn extend(self, other: Triangles) -> Self {
        match (self, other) {
            (Self::Triangle(a), Self::Triangle(b)) => Self::TriMesh(
                Box::new([a[0], a[1], a[2], b[0], b[1], b[2]]),
                Box::new([[0, 1, 2], [3, 4, 5]]),
            ),
            (Self::TriMesh(verts, tris), Self::Triangle(other_verts))
            | (Self::Triangle(other_verts), Self::TriMesh(verts, tris)) => {
                let mut verts = verts.into_vec();
                let mut tris = tris.into_vec();

                let next_index = verts.len() as u32;
                tris.push([next_index, next_index + 1, next_index + 2]);
                verts.extend(other_verts);

                Self::TriMesh(verts.into_boxed_slice(), tris.into_boxed_slice())
            }
            (Self::TriMesh(verts, tris), Self::TriMesh(other_verts, other_tris)) => {
                let mut verts = verts.into_vec();
                verts.extend(other_verts);
                let mut tris = tris.into_vec();
                tris.extend(other_tris);

                Self::TriMesh(verts.into_boxed_slice(), tris.into_boxed_slice())
            }
        }
    }
}

const SUBDIVISIONS: u32 = 5;

/// Rasterizes a collider into a collection of triangles.
pub(crate) fn rasterize_collider_inner(collider: ColliderType, memoized_triangles: Triangles) -> Triangles {
    let (vertices, triangles) = match collider {
        ColliderType::Cuboid(cuboid) => cuboid.to_trimesh(),
        ColliderType::Ball(ball) => ball.to_trimesh(SUBDIVISIONS, SUBDIVISIONS),
        ColliderType::Capsule(capsule) => capsule.to_trimesh(SUBDIVISIONS, SUBDIVISIONS),
        ColliderType::Cylinder(cylinder) => cylinder.to_trimesh(SUBDIVISIONS),
        ColliderType::Cone(cone) => cone.to_trimesh(SUBDIVISIONS),
        ColliderType::Triangle(triangle) => {
            let triangle = Triangles::Triangle(
                triangle
                    .vertices()
                    .map(Vec3::from),
            );

            return memoized_triangles.extend(triangle);
        }
        ColliderType::Compound(colliders) => {
            let mut memoized_triangles = memoized_triangles;
            for collider in colliders {
                memoized_triangles = rasterize_collider_inner(collider, memoized_triangles);
            }
            return memoized_triangles;
        }
    };

    let vertices = vertices
        .into_iter()
        .map(Vec3::from)
        .collect();

    let trimesh = Triangles::TriMesh(vertices, triangles.into_boxed_slice());
    memoized_triangles.extend(trimesh)
}


/// Handle the geometry result and add it to the appropriate collections.
pub(crate) fn handle_geometry_result(
    type_to_convert: GeometryResult,
    entity: Entity,
    collider_transform: GlobalTransform,
    area: Option<Area>,
    geometry_collections: &mut Vec<GeometryCollection>,
    heightfield_collections: &mut Vec<HeightFieldCollection>,
    entity_heightfield_map: &mut EntityHashMap<Arc<shape::HeightField>>,
) {
    match type_to_convert {
        GeometryResult::GeometryToConvert(geometry_to_convert) => {
            geometry_collections.push(GeometryCollection {
                transform: collider_transform,
                geometry_to_convert,
                area,
            });
        }
        GeometryResult::Heightfield(heightfield) => {
            // Don't Duplicate heightfields.
            let heightfield = if let Some(heightfield_arc) = entity_heightfield_map.get(&entity) {
                HeightFieldCollection {
                    transform: collider_transform,
                    heightfield: heightfield_arc.clone(),
                    area,
                }
            } else {
                let height_field_arc = Arc::new(heightfield.clone());
                entity_heightfield_map.insert(entity, height_field_arc.clone());
                HeightFieldCollection {
                    transform: collider_transform,
                    heightfield: height_field_arc,
                    area,
                }
            };
            heightfield_collections.push(heightfield);
        }
        GeometryResult::Compound(results) => {
            for (isometry, result) in results {
                // TODO: not sure this is correct, havent tested compound yet
                let (trans, rot) = isometry.into();
                let local_trans =
                    GlobalTransform::from(Transform::from_translation(trans).with_rotation(rot));

                handle_geometry_result(
                    result,
                    entity,
                    local_trans * collider_transform,
                    area,
                    geometry_collections,
                    heightfield_collections,
                    entity_heightfield_map,
                );
            }
        }
        GeometryResult::Unsupported => {}
    }
}

pub fn get_geometry_type(collider: TypedShape) -> GeometryResult {
    match collider {
        TypedShape::Ball(ball) => GeometryToConvert::Collider(ColliderType::Ball(*ball)).into(),
        TypedShape::Cuboid(cuboid) => {
            GeometryToConvert::Collider(ColliderType::Cuboid(*cuboid)).into()
        }
        TypedShape::Capsule(capsule) => {
            GeometryToConvert::Collider(ColliderType::Capsule(*capsule)).into()
        }
        TypedShape::TriMesh(trimesh) => GeometryToConvert::ParryTriMesh(
            // Can't turn these into boxed slices instantly because they are references.. So we need a copy.
            trimesh.vertices().to_vec().into_boxed_slice(),
            trimesh.indices().to_vec().into_boxed_slice(),
        )
        .into(),
        TypedShape::HeightField(heightfield) => GeometryResult::Heightfield(heightfield),
        TypedShape::ConvexPolyhedron(polyhedron) => {
            let tri = polyhedron.to_trimesh();

            GeometryToConvert::ParryTriMesh(tri.0.into_boxed_slice(), tri.1.into_boxed_slice())
                .into()
        }
        TypedShape::Cylinder(cylinder) => {
            GeometryToConvert::Collider(ColliderType::Cylinder(*cylinder)).into()
        }
        TypedShape::Cone(cone) => GeometryToConvert::Collider(ColliderType::Cone(*cone)).into(),
        TypedShape::RoundCuboid(round_cuboid) => {
            GeometryToConvert::Collider(ColliderType::Cuboid(round_cuboid.inner_shape)).into()
        }
        TypedShape::RoundCylinder(round_cylinder) => {
            GeometryToConvert::Collider(ColliderType::Cylinder(round_cylinder.inner_shape)).into()
        }
        TypedShape::RoundCone(round_cone) => {
            GeometryToConvert::Collider(ColliderType::Cone(round_cone.inner_shape)).into()
        }
        TypedShape::RoundConvexPolyhedron(round_polyhedron) => {
            let tri = round_polyhedron.inner_shape.to_trimesh();

            GeometryToConvert::ParryTriMesh(tri.0.into_boxed_slice(), tri.1.into_boxed_slice())
                .into()
        }
        TypedShape::Triangle(triangle) => {
            GeometryToConvert::Collider(ColliderType::Triangle(*triangle)).into()
        }
        TypedShape::RoundTriangle(triangle) => {
            GeometryToConvert::Collider(ColliderType::Triangle(triangle.inner_shape)).into()
        }
        TypedShape::Compound(colliders) => {
            let results = colliders
                .shapes()
                .iter()
                .map(|(isometry, shape)| (*isometry, get_geometry_type(shape.0.as_typed_shape())))
                .collect();

            GeometryResult::Compound(results)
        }
        // These ones do not make sense in this.
        TypedShape::HalfSpace(_) => GeometryResult::Unsupported, /* This is like an infinite plane? We don't care. */
        TypedShape::Polyline(_) => GeometryResult::Unsupported,  /* This is a line. */
        TypedShape::Segment(_) => GeometryResult::Unsupported,   /* This is a line segment. */
        TypedShape::Custom(_) => {
            warn!(
                "Custom shapes are not yet supported for nav-mesh generation, skipping for now.."
            );
            GeometryResult::Unsupported
        }
    }
}
