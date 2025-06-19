#![allow(dead_code)]
use bevy::{math::bounding::{Aabb3d, RayCast3d}, prelude::*};


pub trait Aabb3dExt {
    /// Test if the Aabb3d contains a point.
    fn contains_point(&self, point: impl Into<Vec3A>) -> bool;

    /// Constructs the smallest Aabb3d containing all the given points.
    fn from_points(points: impl Iterator<Item = impl Into<Vec3A>>) -> Result<Aabb3d, Aabb3dExtError>;
}

#[derive(Debug)]
pub enum Aabb3dExtError {
    NoPoints,
}
impl Aabb3dExt for Aabb3d {
    fn contains_point(&self, point: impl Into<Vec3A>) -> bool {
        let p = point.into();
        p.cmpge(self.min).all() && p.cmple(self.max).all()
    }

    /// Constructs the smallest Aabb3d containing all the given points.
    /// Returns an error if the iterator is empty.
    fn from_points( points: impl Iterator<Item = impl Into<Vec3A>>,) -> Result<Aabb3d, Aabb3dExtError> {
        let mut iter = points.map(|p| p.into());
        let first: Vec3A = match iter.next() {
            Some(p) => p,
            None => return Err(Aabb3dExtError::NoPoints),
        };
        let (min, max) = iter.fold((first, first), |(min, max), p| {
            (min.min(p), max.max(p))
        });
        Ok(Aabb3d { min: min.into(), max: max.into() })
    }
}

#[test]
fn test_contains_point() {
    let aabb = Aabb3d {
        min: Vec3A::new(-9., -2.0, -9.55),
        max: Vec3A::new(9.0, 0.5, 9.34),
    };
    assert_eq!(aabb.contains_point(Vec3A::new(0.0, 0.0, 0.0)), true);
}


// pub trait RayCast3dToSpace {
//     /// Converting ray into another space
//     fn to_space(&self, transform: &GlobalTransform) -> RayCast3d;

//     /// Get the point at a given distance along the ray.    
//     fn get_point(&self, distance: f32) -> Vec3A;
// }

// impl RayCast3dToSpace for RayCast3d {
//     // TODO: figured this be built in to bevy
//     #[inline]
//     fn to_space(&self, transform: &GlobalTransform) -> RayCast3d {
//         let world_to = transform.affine().inverse();
//         RayCast3d::new(
//             world_to.transform_point3a(self.origin),            
//             Dir3A::new(world_to.transform_vector3a(self.direction.as_vec3a())).unwrap(),
//             self.max,
//         )
//     }

//     #[inline]
//     fn get_point(&self, distance: f32) -> Vec3A {
//         self.origin + *self.direction * distance
//     }
// }

    