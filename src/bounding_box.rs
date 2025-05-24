use bevy::{math::bounding::{Aabb3d}, prelude::*};

#[derive(Component, Clone, Debug, Deref, DerefMut)]
pub(crate) struct Bounding(pub Aabb3d);


pub trait BoundingAabb3d {

    fn contains_point(&self, point: impl Into<Vec3A>) -> bool;

    fn from_points(points: impl Iterator<Item = impl Into<Vec3A>>) -> Result<Aabb3d, BoundingBoxError>;
}

#[derive(Debug)]
pub enum BoundingBoxError {
    EmptyPoints,
}
impl BoundingAabb3d for Aabb3d {
    fn contains_point(&self, point: impl Into<Vec3A>) -> bool {
        let p = point.into();
        p.cmpge(self.min).all() && p.cmple(self.max).all()
    }

    /// Constructs the smallest Aabb3d containing all the given points.
    /// Returns an error if the iterator is empty.
    fn from_points( points: impl Iterator<Item = impl Into<Vec3A>>,) -> Result<Aabb3d, BoundingBoxError> {
        let mut iter = points.map(|p| p.into());
        let first: Vec3A = match iter.next() {
            Some(p) => p,
            None => return Err(BoundingBoxError::EmptyPoints),
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
        min: Vec3A::new(-9., 2.0, -9.55),
        max: Vec3A::new(9.0, 0.5, 9.34),
    };
    assert_eq!(aabb.contains_point(Vec3A::new(0.0, 0.0, 0.0)), true);
}