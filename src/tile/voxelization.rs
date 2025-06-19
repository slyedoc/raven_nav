use std::{cmp::Ordering, ops::Div};

use avian3d::parry::{bounding_volume::Aabb, math::Isometry, na::Point3};
use bevy::{math::Vec3A, prelude::*};
use smallvec::SmallVec;

use crate::{Area, nav::Nav, collider::*};

use super::get_neighbour_index;

#[derive(Default, Clone, Debug)]
struct HeightSpan {
    min: u16,
    max: u16,
    traversable: bool,
    area: Option<Area>,
}

#[derive(Default, Clone)]
struct VoxelCell {
    spans: SmallVec<[HeightSpan; 2]>, // Bottom to top.
}

#[derive(Default)]
pub struct VoxelizedTile {
    cells: Vec<VoxelCell>, // len = tiles_along_width^2. Laid out X to Y
}

#[derive(Default, Clone, Debug)]
pub(super) struct OpenCell {
    pub(super) spans: SmallVec<[OpenSpan; 1]>,
}

// Like a HeightSpan but representing open walkable areas (empty space with floor & height >= walkable_height
#[derive(Default, Clone, Copy, Debug)]
pub(super) struct OpenSpan {
    pub(super) min: u16,
    pub(super) max: Option<u16>,
    pub(super) neighbours: [Option<u16>; 4],
    pub(super) tile_index: usize, // The index of this span in the whole tile.
    pub(super) region: u16,       // Region if non-zero.
    area: Option<Area>, // TODO: Ideally we don't want store this here. It's only here to be copied over to [OpenTile::areas] & bumps up the OpenSpan size from 32b to 40b.
}

#[derive(Default, Debug)]
pub struct OpenTile {
    pub(super) cells: Vec<OpenCell>, // len = tiles_along_width^2. Laid out X to Y
    pub(super) distances: Box<[u16]>, // Distances used in watershed. One per span. Use tile_index to go from span to distance.
    pub(super) areas: Box<[Option<Area>]>,
    pub(super) max_distance: u16,
    pub(super) span_count: usize, // Total spans in all cells.
    pub(super) max_regions: u16,
}



pub(super) fn build_heightfield_tile(
    config: &Nav,
    triangle_collections: Vec<TriangleCollection>,
    heightfields: Vec<HeightFieldCollection>,
) -> VoxelizedTile {
    #[cfg(feature = "trace")]
    let _span = info_span!("raven::build_heightfield_tile").entered();

    //let t1 = Instant::now();

    // Allocate the tile.
    let tile_side = config.get_tile_side_with_border();
    let mut voxel_tile = VoxelizedTile {
        cells: vec![VoxelCell::default(); tile_side.pow(2)],
    };
    let tile_max_bound = IVec3::new((tile_side - 1) as i32, 0, (tile_side - 1) as i32);
    let tile_min_bound = Vec3A::from(config.get_tile_minimum_bound_with_border());

    // Build triangles list
    for TriangleCollection {
        transform,
        triangles,
        area,
    } in triangle_collections.iter()
    {
        // TODO: This might be wrong for avian or custom parry3d colliders, but I can't figure out a nice way to know whether or not we're actually dealing with a rapier3d collider.
        //let transform = transform.with_scale(Vec3::ONE).compute_affine(); // The collider returned from rapier already has scale applied to it, so we reset it here.

        match triangles {
            Triangles::Triangle(vertices) => {
                let v = vertices.map(|vertex| {
                    transform.affine().transform_point3a(Vec3A::from(vertex)) - tile_min_bound
                });
                process_triangle(
                    v[0],
                    v[1],
                    v[2],
                    config,
                    tile_max_bound,
                    tile_side,
                    &mut voxel_tile.cells,
                    *area,
                );
            }
            Triangles::TriMesh(vertices, triangles) => {
                let v = vertices
                    .iter()
                    .map(|vertex| {
                        transform.affine().transform_point3a(Vec3A::from(*vertex)) - tile_min_bound
                    })
                    .collect::<Vec<_>>();

                for triangle in triangles.iter() {
                    process_triangle(
                        v[triangle[0] as usize],
                        v[triangle[1] as usize],
                        v[triangle[2] as usize],
                        config,
                        tile_max_bound,
                        tile_side,
                        &mut voxel_tile.cells,
                        *area,
                    );
                }
            }
        }
    }

    for hf in heightfields.iter() {
        // process_triangle is slow, so we are goign to filter out triangles we can before hand
        // build tile aabb
        let title_size = config.get_tile_size();
        let half = title_size * 0.5;
        let pad = config.cell_width; // add padding to the aabb to avoid slight gap between tiles
        let min = Point3::new(-half - pad, -config.world_half_extents.y, -half - pad);
        let max = Point3::new(half + pad, config.world_half_extents.y, half + pad);
        let tile_aabb = Aabb::new(min, max);

        // use hf transform which is already in tile local space to create a tile to hf aabb
        let collider_trans = hf.transform.compute_transform();
        let hf_to_tile_iso = Isometry::new(
            collider_trans.translation.into(),
            collider_trans.rotation.to_scaled_axis().into(),
        );
        let tile_to_hf_iso = hf_to_tile_iso.inverse();
        let hf_local_aabb = tile_aabb.transform_by(&tile_to_hf_iso);

        // only use triangles in the aabb
        hf.heightfield
            .map_elements_in_local_aabb(&hf_local_aabb, &mut |_i, t| {
                process_triangle(
                    hf.transform.affine().transform_point3a(Vec3A::from(t.a)) - tile_min_bound,
                    hf.transform.affine().transform_point3a(Vec3A::from(t.b)) - tile_min_bound,
                    hf.transform.affine().transform_point3a(Vec3A::from(t.c)) - tile_min_bound,
                    config,
                    tile_max_bound,
                    tile_side,
                    &mut voxel_tile.cells,
                    hf.area,
                );
            });
    }

    // let t2 = Instant::now();
    // let time1 = t2.duration_since(t1);
    // println!("Voxelize time: {time1:?}");

    voxel_tile
}

#[cfg(test)]

mod tests {
    use std::sync::Arc;

    use super::*;
    use test::Bencher;

    #[test]
    fn test_build_heightfield_tile() {
        // build heightfield
        let resolution = 250;
        let oct = 10.0;
        let height_scale = 8.0;
        let scale = Vec3::new(resolution as f32, height_scale, resolution as f32);

        let mut heightfield = vec![vec![0.0; resolution]; resolution];
        for x in 0..resolution {
            for y in 0..resolution {
                heightfield[x][y] = (x as f32 / oct).sin() + (y as f32 / oct).cos();
            }
        }

        let height_collider = avian3d::prelude::Collider::heightfield(heightfield, scale);
        // Get the geometry type
        let geometry_result = height_collider.shape_scaled().as_typed_shape();

        // Convert the collider's transform to the tile's local space
        // let transform = GlobalTransform::from(
        //     tile_transform.affine().inverse() * collider_transform.affine(),
        // );
        let heightfield = match geometry_result {
            avian3d::parry::shape::TypedShape::HeightField(hf) => HeightFieldCollection {
                transform: GlobalTransform::IDENTITY,
                heightfield: Arc::new(hf.clone()),
                area: Some(Area(0)),
            },
            _ => unreachable!(),
        };
        let mut heightfield_collections = Vec::new();
        heightfield_collections.push(heightfield);

        let config = Nav::new(0.5, 1.9, Vec3::splat(50.0));
        let _voxel_tile = build_heightfield_tile(&config, vec![], heightfield_collections);
    }

    #[bench]
    fn bench_build_heightfield_tile(b: &mut Bencher) {
        b.iter(|| {
            test_build_heightfield_tile();
        });
    }
}

// TODO: This is the hot path and should be optimized.
#[allow(clippy::too_many_arguments)]
fn process_triangle(
    a: Vec3A,
    b: Vec3A,
    c: Vec3A,
    vox_settings: &Nav,
    tile_max_bound: IVec3,
    tile_side: usize,
    voxel_cells: &mut [VoxelCell],
    area: Option<Area>,
) {
    let min_bound = a.min(b).min(c).div(vox_settings.cell_width).as_ivec3();
    let max_bound = a.max(b).max(c).div(vox_settings.cell_width).as_ivec3();

    // Check if triangle is completely outside the tile.
    if max_bound.x < 0
        || max_bound.z < 0
        || min_bound.x > tile_max_bound.x
        || min_bound.z > tile_max_bound.z
    {
        return;
    }

    let clamped_bound_min = min_bound.max(IVec3::ZERO);
    let clamped_bound_max = max_bound.min(tile_max_bound) + IVec3::ONE;
    let traversable = is_triangle_traversable(a, b, c, vox_settings);
    let vertices = [a, b, c];

    // For cache reasons we go.
    // --> X
    // Z
    // |
    // V
    // X is column. Z is row.
    // Which means we iterate Z first.
    for z in clamped_bound_min.z..clamped_bound_max.z {
        let row_clip_min = z as f32 * vox_settings.cell_width;
        let row_clip_max = row_clip_min + vox_settings.cell_width;

        // Clip polygon to the row.
        // TODO: This is awful & too complicated.
        let (row_min_clip_vert_count, row_min_clip_verts) =
            divide_polygon(&vertices, row_clip_min, 2, false);
        let (row_vert_count, row_verts) = divide_polygon(
            &row_min_clip_verts[..row_min_clip_vert_count],
            row_clip_max,
            2,
            true,
        );
        if row_vert_count < 3 {
            continue;
        }

        // Calculate the column footprint of the row.
        let mut column_min_vert_x = row_verts[0].x;
        let mut column_max_vert_x = row_verts[0].x;
        for vertex in row_verts.iter().take(row_vert_count).skip(1) {
            column_min_vert_x = column_min_vert_x.min(vertex.x);
            column_max_vert_x = column_max_vert_x.max(vertex.x);
        }
        let column_min = ((column_min_vert_x / vox_settings.cell_width) as i32).max(0);
        let column_max =
            ((column_max_vert_x / vox_settings.cell_width) as i32).min((tile_side - 1) as i32) + 1;

        for x in column_min..column_max {
            let column_clip_min = x as f32 * vox_settings.cell_width;
            let column_clip_max = column_clip_min + vox_settings.cell_width;

            // Clip polygon to column.
            let (column_min_clip_vert_count, column_min_clip_verts) =
                divide_polygon(&row_verts[..row_vert_count], column_clip_min, 0, false);
            let (column_vert_count, column_verts) = divide_polygon(
                &column_min_clip_verts[..column_min_clip_vert_count],
                column_clip_max,
                0,
                true,
            );
            if column_vert_count < 3 {
                continue;
            }

            let mut square_min_height = column_verts[0].y;
            let mut square_max_height = column_verts[0].y;
            for vertex in column_verts.iter().take(column_vert_count).skip(1) {
                square_min_height = square_min_height.min(vertex.y);
                square_max_height = square_max_height.max(vertex.y);
            }

            square_min_height = square_min_height.max(0.0);
            if square_max_height < 0.0 {
                continue;
            }

            let min_height = (square_min_height / vox_settings.cell_height) as u16;
            let max_height = (square_max_height / vox_settings.cell_height) as u16;

            let index = x as usize + z as usize * tile_side;
            let cell = &mut voxel_cells[index];

            let mut new_span = HeightSpan {
                min: min_height,
                max: max_height,
                traversable,
                area,
            };

            if cell.spans.is_empty() {
                cell.spans.push(new_span);
                continue;
            }
            // We need to go over all existing ones.
            let mut i = 0;
            while i < cell.spans.len() {
                let existing_span = &cell.spans[i];
                if existing_span.min > new_span.max {
                    // i is beyond the new span. We can insert!
                    break;
                } else if existing_span.max < new_span.min {
                    // i is before the new span. Continue until we hit one that isn't.
                    i += 1;
                    continue;
                } else {
                    match existing_span.max.cmp(&new_span.max) {
                        Ordering::Greater => {
                            new_span.traversable = existing_span.traversable;
                            new_span.area = existing_span.area;
                        }
                        Ordering::Equal => {
                            new_span.traversable |= existing_span.traversable;
                            // Higher area number has higher priority.
                            new_span.area = new_span.area.max(existing_span.area);
                        }
                        Ordering::Less => {}
                    }

                    // Extend new span to existing span's size.
                    if existing_span.min < new_span.min {
                        new_span.min = existing_span.min;
                    }
                    if existing_span.max > new_span.max {
                        new_span.max = existing_span.max;
                    }

                    cell.spans.remove(i);
                }
            }
            cell.spans.insert(i, new_span);
        }
    }
}

#[inline]
fn is_triangle_traversable(a: Vec3A, b: Vec3A, c: Vec3A, vox_settings: &Nav) -> bool {
    let ab = b - a;
    let ac = c - a;
    let normal = ab.cross(ac).normalize();
    let slope = normal.dot(Vec3A::Y).acos();
    slope < vox_settings.max_traversable_slope_degrees.to_radians()
}

/*
*   This function takes in a polygon (of max 7 vertices), an line on which to divide it, and an axis.
*   It then returns the left polygon's vertex count, the left polygon's vertices,
*   the right polygon's vertex count, and the right polygon's vertices.
*/
fn divide_polygon(
    vertices: &[Vec3A],
    clip_line: f32,
    axis: usize,
    keep_left: bool,
) -> (usize, [Vec3A; 7]) {
    let mut delta_from_line = [0.0; 7];
    // This loop determines which side of the line the vertex is on.
    for (i, vertex) in vertices.iter().enumerate() {
        delta_from_line[i] = clip_line - vertex[axis];
    }

    let mut polygon_left = [Vec3A::ZERO; 7];
    let mut polygon_right = [Vec3A::ZERO; 7];

    let mut verts_left = 0;
    let mut verts_right = 0;

    for i in 0..vertices.len() {
        let previous = (vertices.len() - 1 + i) % vertices.len(); // j is i-1 wrapped.

        let vertex_prev = vertices[previous];
        let vertex_i = vertices[i];

        let delta_prev = delta_from_line[previous];
        let delta_i = delta_from_line[i];

        let in_a = delta_prev >= 0.0;
        let in_b = delta_i >= 0.0;

        // Check if both vertices are on the same side of the line.
        if in_a != in_b {
            // We slide the vertex along to the edge.
            let slide = delta_prev / (delta_prev - delta_i);

            polygon_left[verts_left] = vertex_prev + (vertex_i - vertex_prev) * slide;
            polygon_right[verts_right] = polygon_left[verts_left];
            verts_left += 1;
            verts_right += 1;

            if delta_i > 0.0 {
                polygon_left[verts_left] = vertex_i;
                verts_left += 1;
            } else if delta_i < 0.0 {
                polygon_right[verts_right] = vertex_i;
                verts_right += 1;
            }
        } else {
            if delta_i >= 0.0 {
                polygon_left[verts_left] = vertex_i;
                verts_left += 1;

                if delta_i != 0.0 {
                    continue;
                }
            }
            polygon_right[verts_right] = vertex_i;
            verts_right += 1;
        }
    }

    if keep_left {
        (verts_left, polygon_left)
    } else {
        (verts_right, polygon_right)
    }
}

pub fn build_open_heightfield_tile(
    voxelized_tile: VoxelizedTile,
    vox_settings: &Nav,
) -> OpenTile {
    #[cfg(feature = "trace")]
    let _span = info_span!("raven::build_open_heightfield_tile").entered();

    let mut cells = vec![OpenCell::default(); voxelized_tile.cells.len()];
    let mut span_count = 0;

    // First we create open spaces.
    for (i, cell) in voxelized_tile
        .cells
        .iter()
        .enumerate()
        .filter(|(_, cell)| !cell.spans.is_empty())
    {
        let open_spans = &mut cells[i].spans;

        let mut iter = cell.spans.iter().peekable();
        while let Some(span) = iter.next() {
            let area = if span.traversable { span.area } else { None };

            if let Some(next_span) = iter.peek() {
                // Need to check if space is large enough.
                if next_span.min - span.max >= vox_settings.walkable_height {
                    open_spans.push(OpenSpan {
                        min: span.max,
                        max: Some(next_span.min),
                        area,
                        ..Default::default()
                    });
                }
            } else {
                // None above. This is an unbounded open space.
                open_spans.push(OpenSpan {
                    min: span.max,
                    max: None,
                    area,
                    ..Default::default()
                });
            }
        }
        span_count += open_spans.len();
    }

    // Create Open Tile.
    let mut open_tile = OpenTile {
        cells,
        distances: vec![u16::MAX; span_count].into_boxed_slice(),
        areas: vec![None; span_count].into_boxed_slice(),
        max_distance: 0,
        span_count,
        max_regions: 0,
    };

    // Assign tile_index & copy over areas.
    let mut tile_index = 0;
    for cell in open_tile.cells.iter_mut() {
        for span in cell.spans.iter_mut() {
            span.tile_index = tile_index;

            open_tile.areas[tile_index] = span.area;

            tile_index += 1;
        }
    }

    {
        #[cfg(feature = "trace")]
        let _span = info_span!("Link neighbours").entered();
        link_neighbours(&mut open_tile, vox_settings);
    }

    open_tile
}

fn link_neighbours(open_tile: &mut OpenTile, vox_settings: &Nav) {
    let mut neighbour_spans = Vec::with_capacity(3);

    let tile_side = vox_settings.get_tile_side_with_border();
    for i in 0..open_tile.cells.len() {
        if open_tile.cells[i].spans.is_empty() {
            continue;
        }

        let row = i / tile_side;
        let column = i % tile_side;

        let neighbour_index = [
            if column > 0 { Some(i - 1) } else { None },
            if row < (tile_side - 1) {
                Some(i + tile_side)
            } else {
                None
            },
            if column < (tile_side - 1) {
                Some(i + 1)
            } else {
                None
            },
            if row > 0 { Some(i - tile_side) } else { None },
        ];

        // For each direct neighbour.
        for (neighbour, neighbour_index) in neighbour_index
            .into_iter()
            .enumerate()
            .filter_map(|(i, index)| Some(i).zip(index))
        {
            neighbour_spans.clear();
            neighbour_spans.extend(
                open_tile.cells[neighbour_index]
                    .spans
                    .iter()
                    .map(|span| (span.min, span.max)),
            );

            for span in open_tile.cells[i].spans.iter_mut() {
                for (i, (min, max)) in neighbour_spans.iter().enumerate() {
                    if let Some((max, span_max)) = max.zip(span.max) {
                        let gap = span_max.min(max).abs_diff(span.min.max(*min));
                        if gap < vox_settings.walkable_height {
                            continue;
                        }
                    }

                    if min.abs_diff(span.min) < vox_settings.step_height {
                        span.neighbours[neighbour] = Some(i as u16);
                        break;
                    }
                }
            }
        }
    }
}

pub fn erode_walkable_area(open_tile: &mut OpenTile, vox_settings: &Nav) {
    #[cfg(feature = "trace")]
    let _span = info_span!("raven::erode_walkable_area").entered();

    let tile_side = vox_settings.get_tile_side_with_border();
    // Mark boundary cells.
    for (i, cell) in open_tile.cells.iter().enumerate() {
        for span in cell.spans.iter() {
            let area = open_tile.areas[span.tile_index];

            if area.is_none() {
                open_tile.distances[span.tile_index] = 0;
                continue;
            }

            let all_neighbours = span.neighbours.iter().enumerate().all(|(dir, neighbour)| {
                if let Some(neighbour) = neighbour {
                    let neighbour_index = get_neighbour_index(tile_side, i, dir);
                    let neighbour = &open_tile.cells[neighbour_index].spans[*neighbour as usize];

                    open_tile.areas[neighbour.tile_index].is_some() // Any neighbour not marked as unwalkable.
                } else {
                    false
                }
            });

            open_tile.distances[span.tile_index] = if all_neighbours { u16::MAX } else { 0 };
        }
    }

    filter_tile(open_tile, vox_settings);

    // Any cell within 2*walkable_radius is considered unwalkable. This ensures characters won't clip into walls.
    let threshold = vox_settings.walkable_radius * 2;
    for i in 0..open_tile.span_count {
        if open_tile.distances[i] < threshold {
            open_tile.areas[i] = None;
        }
    }
}

pub fn calculate_distance_field(open_tile: &mut OpenTile, vox_settings: &Nav) {
    #[cfg(feature = "trace")]
    let _span = info_span!("raven::calculate_distance_field").entered();

    let tile_side = vox_settings.get_tile_side_with_border();
    // Mark boundary cells.
    for (i, cell) in open_tile.cells.iter().enumerate() {
        for span in cell.spans.iter() {
            let area = open_tile.areas[span.tile_index];

            let all_neighbours = span.neighbours.iter().enumerate().all(|(dir, neighbour)| {
                if let Some(neighbour) = neighbour {
                    let neighbour_index = get_neighbour_index(tile_side, i, dir);
                    let neighbour = &open_tile.cells[neighbour_index].spans[*neighbour as usize];

                    open_tile.areas[neighbour.tile_index] == area // Only neighbours of same area.
                } else {
                    false
                }
            });

            open_tile.distances[span.tile_index] = if all_neighbours { u16::MAX } else { 0 };
        }
    }

    filter_tile(open_tile, vox_settings);

    open_tile.max_distance = *open_tile.distances.iter().max().unwrap_or(&0);

    // Box blur. If you're reading this, why?
    let threshold = 2;

    let mut blurred = vec![0; open_tile.distances.len()].into_boxed_slice();

    for (i, cell) in open_tile.cells.iter().enumerate() {
        for span in cell.spans.iter() {
            let distance = open_tile.distances[span.tile_index];
            if distance <= threshold {
                blurred[span.tile_index] = distance;
                continue;
            }

            let mut d = distance;
            for dir in 0..4 {
                let Some(index) = span.neighbours[dir] else {
                    d += distance * 2;
                    continue;
                };

                let other_cell_index = get_neighbour_index(tile_side, i, dir);
                let other_span = &open_tile.cells[other_cell_index].spans[index as usize];

                d += open_tile.distances[other_span.tile_index];

                let next_dir = (dir + 1) & 0x3;
                let Some(index) = other_span.neighbours[next_dir] else {
                    d += distance;
                    continue;
                };

                let other_cell_index = get_neighbour_index(tile_side, other_cell_index, next_dir);

                let other_span = &open_tile.cells[other_cell_index].spans[index as usize];

                d += open_tile.distances[other_span.tile_index];
            }

            // Apply distance change.
            blurred[span.tile_index] = (d + 5) / 9;
        }
    }

    open_tile.distances = blurred;
    // End Box Blur
}

fn filter_tile(open_tile: &mut OpenTile, vox_settings: &Nav) {
    let tile_side = vox_settings.get_tile_side_with_border();
    // Pass 1.
    for (i, cell) in open_tile.cells.iter().enumerate() {
        for span in cell.spans.iter() {
            let mut distance = open_tile.distances[span.tile_index];

            if let Some(span_index) = span.neighbours[0] {
                // (-1, 0)
                let other_cell_index = i - 1;
                let other_span = &open_tile.cells[other_cell_index].spans[span_index as usize];

                let other_distance = open_tile.distances[other_span.tile_index] + 2;
                if other_distance < distance {
                    distance = other_distance;
                }

                // (-1, -1)
                if let Some(span_index) = other_span.neighbours[3] {
                    let other_cell_index = other_cell_index - tile_side;
                    let other_span = &open_tile.cells[other_cell_index].spans[span_index as usize];

                    let other_distance = open_tile.distances[other_span.tile_index] + 3;
                    if other_distance < distance {
                        distance = other_distance;
                    }
                }
            }

            if let Some(span_index) = span.neighbours[3] {
                // (0, -1)
                let other_cell_index = i - tile_side;
                let other_span = &open_tile.cells[other_cell_index].spans[span_index as usize];

                let other_distance = open_tile.distances[other_span.tile_index] + 2;
                if other_distance < distance {
                    distance = other_distance;
                }

                // (1, -1)
                if let Some(span_index) = other_span.neighbours[2] {
                    let other_cell_index = other_cell_index + 1;
                    let other_span = &open_tile.cells[other_cell_index].spans[span_index as usize];

                    let other_distance = open_tile.distances[other_span.tile_index] + 3;
                    if other_distance < distance {
                        distance = other_distance;
                    }
                }
            }

            // Apply distance change.
            open_tile.distances[span.tile_index] = distance;
        }
    }

    // Pass 2
    for (i, cell) in open_tile.cells.iter().enumerate().rev() {
        for span in cell.spans.iter() {
            let mut distance = open_tile.distances[span.tile_index];

            if let Some(span_index) = span.neighbours[2] {
                // (1, 0)
                let other_cell_index = i + 1;
                let other_span = &open_tile.cells[other_cell_index].spans[span_index as usize];

                let other_distance = open_tile.distances[other_span.tile_index] + 2;
                if other_distance < distance {
                    distance = other_distance;
                }

                // (1, 1)
                if let Some(span_index) = other_span.neighbours[1] {
                    let other_cell_index = other_cell_index + tile_side;
                    let other_span = &open_tile.cells[other_cell_index].spans[span_index as usize];

                    let other_distance = open_tile.distances[other_span.tile_index] + 3;
                    if other_distance < distance {
                        distance = other_distance;
                    }
                }
            }

            if let Some(span_index) = span.neighbours[1] {
                // (0, 1)
                let other_cell_index = i + tile_side;
                let other_span = &open_tile.cells[other_cell_index].spans[span_index as usize];

                let other_distance = open_tile.distances[other_span.tile_index] + 2;
                if other_distance < distance {
                    distance = other_distance;
                }

                // (-1, 1)
                if let Some(span_index) = other_span.neighbours[0] {
                    let other_cell_index = other_cell_index - 1;
                    let other_span = &open_tile.cells[other_cell_index].spans[span_index as usize];

                    let other_distance = open_tile.distances[other_span.tile_index] + 3;
                    if other_distance < distance {
                        distance = other_distance;
                    }
                }
            }

            // Apply distance change.
            open_tile.distances[span.tile_index] = distance;
        }
    }
}
