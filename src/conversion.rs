use avian3d::parry::{
    math::Real,
    na::Point3,
    shape::{Ball, Capsule, Cone, Cuboid, Cylinder, Triangle},
};
use bevy::prelude::*;

use crate::{collider::Area, heightfields::TriangleCollection};

pub struct GeometryCollection {
    pub transform: GlobalTransform,
    pub geometry_to_convert: GeometryToConvert,
    pub area: Option<Area>,
}

pub enum ColliderType {
    Cuboid(Cuboid),
    Ball(Ball),
    Capsule(Capsule),
    Cylinder(Cylinder),
    Cone(Cone),
    Triangle(Triangle),
    Compound(Vec<ColliderType>),
}

pub enum GeometryToConvert {
    Collider(ColliderType),
    ParryTriMesh(Box<[Point3<Real>]>, Box<[[u32; 3]]>),
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
