use bevy::{math::bounding::Aabb3d, prelude::*};


#[derive(Component, Clone, Debug, Deref, DerefMut)]
pub(crate) struct Bounding(pub Aabb3d);
// TODO: can most likely replace with boudning box with aabb, Trans i dont get yet, was called Transform

/// A transform that can be applied to Vec3's.
#[derive(Default)]
pub struct Trans {
  /// The translation to apply.
  pub translation: Vec3,
  /// The rotation to apply around the "up" direction. Specifically, the up
  /// direction is perpendicular to the plane of movement.
  pub rotation: f32,
}

impl Trans {
  /// Applies the transformation.
  pub(crate) fn apply(&self, point: Vec3) -> Vec3 {
    Quat::from_rotation_z(self.rotation) * point
      + self.translation
  }

  /// Inverses the transformation.
  pub(crate) fn apply_inverse(&self, point: Vec3) -> Vec3 {
    Quat::from_rotation_z(-self.rotation)
      * (point - self.translation)
  }
}

/// A bounding box.
#[derive(PartialEq, Clone, Copy, Debug)]
pub(crate) enum BoundingBox {
  /// The bounding box has no points in it.
  Empty,
  /// The bounding box has some points in it.
  Box {
    /// The minimum bounds of the bounding box.
    min: Vec3,
    /// The maximum bounds of the bounding box. Must be component-wise greather
    /// than or equal to `min`.
    max: Vec3,
  },
}

impl BoundingBox {
  /// Creates a box already with some data in it. `min` and `max` must already
  /// be valid - this is unchecked.
  pub(crate) fn new_box(min: Vec3, max: Vec3) -> Self {
    Self::Box { min, max }
  }

  /// Returns whether the box is empty or not.
  pub(crate) fn is_empty(&self) -> bool {
    matches!(self, Self::Empty)
  }

  /// Returns the bounds of the box, assuming it is non-empty.
  #[allow(unused)] // Used by tests.
  pub(crate) fn as_box(&self) -> (Vec3, Vec3) {
    match self {
      Self::Empty => panic!("BoundingBox is not a box."),
      &Self::Box { min, max } => (min, max),
    }
  }

  pub(crate) fn center(&self) -> Option<Vec3> {
    match self {
      Self::Empty => None,
      &Self::Box { min, max } => Some((min + max) * 0.5),
    }
  }

  /// Computes the size of the bounding box. Returns 0 if the bounds are empty.
  pub(crate) fn size(&self) -> Vec3 {
    match self {
      Self::Empty => Vec3::ZERO,
      &Self::Box { min, max } => max - min,
    }
  }

  /// Determines if the bounding box is valid (min <= max).
  pub(crate) fn is_valid(&self) -> bool {
    let size = self.size();
    size.x >= 0.0 && size.y >= 0.0 && size.z >= 0.0
  }

  /// Expands the bounding box to contain the `other`.
  pub(crate) fn expand_to_bounds(&self, other: &Self) -> Self {
    match (self, other) {
      (Self::Empty, Self::Empty) => Self::Empty,
      (Self::Box { .. }, Self::Empty) => *self,
      (Self::Empty, Self::Box { .. }) => *other,
      (
        Self::Box { min, max },
        Self::Box { min: other_min, max: other_max },
      ) => Self::Box { min: min.min(*other_min), max: max.max(*other_max) },
    }
  }

  /// Expands the bounding box to contain `point`. If the box was empty, it will
  /// now hold only the `point`.
  pub(crate) fn expand_to_point(&self, point: Vec3) -> Self {
    match self {
      Self::Empty => Self::Box { min: point, max: point },
      &Self::Box { min, max } => {
        Self::Box { min: min.min(point), max: max.max(point) }
      }
    }
  }

  /// Expands the bounding box by `size`. An empty bounding box will still be
  /// empty after this.
  pub(crate) fn expand_by_size(&self, size: Vec3) -> BoundingBox {
    self.add_to_corners(-size, size)
  }

  /// Adds `delta_min` to the min corner, and `delta_max` to the max corner. An
  /// empty bounding box will still be empty after this.
  pub(crate) fn add_to_corners(
    &self,
    delta_min: Vec3,
    delta_max: Vec3,
  ) -> BoundingBox {
    let expanded_box = match self {
      BoundingBox::Empty => BoundingBox::Empty,
      &BoundingBox::Box { min, max } => {
        BoundingBox::Box { min: min + delta_min, max: max + delta_max }
      }
    };

    if !expanded_box.is_valid() {
      return BoundingBox::Empty;
    }

    expanded_box
  }

  /// Determines if `point` is in `self`.
  pub(crate) fn contains_point(&self, point: Vec3) -> bool {
    match self {
      Self::Empty => false,
      Self::Box { min, max } => {
        min.x <= point.x
          && point.x <= max.x
          && min.y <= point.y
          && point.y <= max.y
          && min.z <= point.z
          && point.z <= max.z
      }
    }
  }

  /// Determines if `other` is fully contained by `self`.
  #[allow(unused)] // Used by tests.
  pub(crate) fn contains_bounds(&self, other: &Self) -> bool {
    let (other_min, other_max) = match other {
      Self::Empty => return false,
      Self::Box { min, max } => (min, max),
    };
    match self {
      Self::Empty => false,
      Self::Box { min, max } => {
        min.x <= other_min.x
          && other_max.x <= max.x
          && min.y <= other_min.y
          && other_max.y <= max.y
          && min.z <= other_min.z
          && other_max.z <= max.z
      }
    }
  }

  /// Detemrines if `other` intersects `self` at all.
  pub(crate) fn intersects_bounds(&self, other: &Self) -> bool {
    let (other_min, other_max) = match other {
      Self::Empty => return false,
      Self::Box { min, max } => (min, max),
    };
    match self {
      Self::Empty => false,
      Self::Box { min, max } => {
        min.x <= other_max.x
          && other_min.x <= max.x
          && min.y <= other_max.y
          && other_min.y <= max.y
          && min.z <= other_max.z
          && other_min.z <= max.z
      }
    }
  }

  /// Creates a conservative bounding box around `self` after transforming it by
  /// [`Trans`].
  pub(crate) fn transform(
    &self,
    transform: &Trans,
  ) -> Self {
    let (min, max) = match self {
      BoundingBox::Empty => return BoundingBox::Empty,
      BoundingBox::Box { min, max } => (min, max),
    };
    let flat_max = Vec3::new(max.x, max.y, min.z);

    let points = [
      transform.apply(*min),
      transform.apply(flat_max),
      transform.apply(Vec3::new(min.x, max.y, min.z)),
      transform.apply(Vec3::new(max.x, min.y, min.z)),
    ];

    BoundingBox::Box {
      min: Vec3::new(
        points
          .iter()
          .map(|p| p.x)
          .min_by(|a, b| a.partial_cmp(b).unwrap())
          .unwrap(),
        points
          .iter()
          .map(|p| p.y)
          .min_by(|a, b| a.partial_cmp(b).unwrap())
          .unwrap(),
        points[0].z,
      ),
      max: Vec3::new(
        points
          .iter()
          .map(|p| p.x)
          .max_by(|a, b| a.partial_cmp(b).unwrap())
          .unwrap(),
        points
          .iter()
          .map(|p| p.y)
          .max_by(|a, b| a.partial_cmp(b).unwrap())
          .unwrap(),
        points[0].z + (max.z - min.z),
      ),
    }
  }
}
