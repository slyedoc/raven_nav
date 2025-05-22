use crate::conversion::{ColliderType, GeometryToConvert};
use avian3d::parry::{
    math::Isometry,
    shape::{HeightField, TypedShape},
};
use bevy::prelude::*;

/// Add this to any compoent with a Collider to indicate that it is a nav-mesh affector.
#[derive(Component, Default, Reflect)]
pub struct NavMeshAffector(pub Area);

#[derive(Default, Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Reflect)]
pub struct Area(pub u16);

/// Added to colliders when they need to update tile affectors, will be removed after
#[derive(Component, Reflect)]
#[reflect(Component)]
pub struct UpdateTileAffectors;

pub enum GeometryResult<'a> {
    Compound(Vec<(Isometry<f32>, GeometryResult<'a>)>),
    GeometryToConvert(GeometryToConvert),
    Heightfield(&'a HeightField),
    Unsupported,
}

impl From<GeometryToConvert> for GeometryResult<'_> {
    fn from(value: GeometryToConvert) -> Self {
        GeometryResult::GeometryToConvert(value)
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
