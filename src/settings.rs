use bevy::prelude::*;
use std::num::{NonZeroU8, NonZeroU16};

#[derive(Resource, Clone)]
pub struct NavMeshSettings {
    /// The horizontal resolution of the voxelized tile.
    ///
    /// **Suggested value**: 1/2 of character radius.
    ///
    /// Smaller values will increase tile generation times with diminishing returns in nav-mesh detail.
    pub cell_width: f32,
    /// The vertical resolution of the voxelized tile.
    ///
    /// **Suggested value**: 1/2 of cell_width.
    ///
    /// Smaller values will increase tile generation times with diminishing returns in nav-mesh detail.
    pub cell_height: f32,

    /// Length of a tile's side in cells. Resulting size in world units is ``tile_width * cell_width``.
    ///
    /// **Suggested value**: ???
    ///
    /// Higher means more to update each time something within the tile changes, smaller means you will have more overhead from connecting the edges to other tiles & generating the tile itself.
    pub tile_width: NonZeroU16,

    /// Extents of the world as measured from the world origin (0.0, 0.0) on the XZ-plane.
    ///
    /// **Suggested value**: As small as possible whilst still keeping the entire world within it.
    ///
    /// This exists because figuring out which tile we are in around the world origin would not work without it.
    pub world_half_extents: f32,
    /// Bottom extents of the world on the Y-axis. The top extents is capped by ``world_bottom_bound + cell_height * u16::MAX``.
    ///
    /// **Suggested value**: Minium Y position of anything in the world that should be covered by the nav mesh.
    pub world_bottom_bound: f32,

    /// Maximum incline/slope traversable when navigating in radians.
    pub max_traversable_slope_radians: f32,
    /// Minimum open height for an area to be considered walkable in cell_height(s).
    ///
    /// **Suggested value**: The height of character * ``cell_height``, rounded up.
    pub walkable_height: u16,
    /// This will "pull-back" the nav-mesh from edges, meaning anywhere on the nav-mesh will be walkable for a character with a radius of ``walkable_radius * cell_width``.
    ///
    /// **Suggested value**: ``ceil(character_radius / cell_width)`` (2-3 if `cell_width` is 1/2 of ``character_radius``)  
    pub walkable_radius: u16,
    /// Maximum height difference that is still considered traversable in cell_height(s). This smooths out stair steps and small ledges.
    pub step_height: u16,

    /// Minimum size of a region in cells, anything smaller than this will be removed. This is used to filter out smaller disconnected island that may appear on surfaces like tables.
    pub min_region_area: u32,
    /// Maximum size of a region in cells we can merge other regions into.
    pub max_region_area_to_merge_into: u32,

    /// Maximum length of an edge before it's split.
    ///
    /// **Suggested value**: Start high and reduce if there are issues.
    pub max_edge_length: u16,
    /// Maximum difference allowed for simplified contour generation on the XZ-plane in cell_width(s).
    ///
    /// **Suggested value range**: `[1.1, 1.5]`
    pub max_contour_simplification_error: f32,

    /// Max tiles to generate in parallel at once. A value of ``None`` will result in no limit.
    ///
    /// Adjust this to control memory & CPU usage. More tiles generating at once will have a higher memory footprint.
    pub max_tile_generation_tasks: Option<NonZeroU16>,

    /// TODO: this breaks when any meshes are concave like heightfields. do not use this yet.
    /// When not None, height correct nav-mesh polygons where the surface height differs too much from the surface in cells. This is very useful for bumpy terrain.
    ///
    /// Helps on bumpy shapes like terrain but comes at a performance cost.
    /// **Experimental**: This may have issues at the edges of regions.
    pub experimental_detail_mesh_generation: Option<DetailMeshSettings>,
}

impl NavMeshSettings {
    /// Helper function for creating nav-mesh settings with reasonable defaults from the size of your navigation agent and bounds of your world.
    ///
    ///
    #[inline]
    pub fn from_agent_and_bounds(
        agent_radius: f32,
        agent_height: f32,
        world_half_extents: f32,
        world_bottom_bound: f32,
    ) -> Self {
        let cell_width = agent_radius / 2.0;
        let cell_height = agent_radius / 4.0;

        let walkable_height = (agent_height / cell_height) as u16;

        Self {
            cell_width,
            cell_height,
            tile_width: NonZeroU16::new(120).unwrap(),
            world_half_extents: world_half_extents.abs(),
            world_bottom_bound,
            max_traversable_slope_radians: 50.0_f32.to_radians(),
            walkable_height,
            walkable_radius: 2,
            step_height: 3,
            min_region_area: 100,
            max_region_area_to_merge_into: 500,
            max_edge_length: 80,
            max_contour_simplification_error: 1.1,
            max_tile_generation_tasks: NonZeroU16::new(8),
            experimental_detail_mesh_generation: None,
        }
    }
    /// Setter for [`NavMeshSettings::walkable_radius`]
    pub fn with_walkable_radius(mut self, walkable_radius: u16) -> Self {
        self.walkable_radius = walkable_radius;

        self
    }
    /// Setter for [`NavMeshSettings::tile_width`]
    pub fn with_tile_width(mut self, tile_width: NonZeroU16) -> Self {
        self.tile_width = tile_width;

        self
    }
    /// Setter for [`NavMeshSettings::max_traversable_slope_radians`]
    pub fn with_traversible_slope(mut self, traversible_slope: f32) -> Self {
        self.max_traversable_slope_radians = traversible_slope;

        self
    }
    /// Setter for [`NavMeshSettings::max_tile_generation_tasks`]
    pub fn with_max_tile_generation_tasks(
        mut self,
        max_tile_generation_tasks: Option<NonZeroU16>,
    ) -> Self {
        self.max_tile_generation_tasks = max_tile_generation_tasks;

        self
    }
    /// Setter for [`NavMeshSettings::step_height`]
    pub fn with_step_height(mut self, step_height: u16) -> Self {
        self.step_height = step_height;

        self
    }
    /// Setter for [`NavMeshSettings::min_region_area`] & [`NavMeshSettings::max_region_area_to_merge_into`]
    pub fn with_region_area(
        mut self,
        min_region_area: u32,
        max_region_area_to_merge_into: u32,
    ) -> Self {
        self.min_region_area = min_region_area;
        self.max_region_area_to_merge_into = max_region_area_to_merge_into;

        self
    }
    /// Setter for [`NavMeshSettings::max_contour_simplification_error`]
    pub fn with_max_contour_simplification_error(
        mut self,
        max_contour_simplification_error: f32,
    ) -> Self {
        self.max_contour_simplification_error = max_contour_simplification_error;

        self
    }
    /// Setter for [`NavMeshSettings::max_edge_length`]
    pub fn with_max_edge_length(mut self, max_edge_length: u16) -> Self {
        self.max_edge_length = max_edge_length;

        self
    }

    /// TODO: this breaks when any meshes are concave like heightfields. do not use this yet.
    /// Setter for [`NavMeshSettings::experimental_detail_mesh_generation`]
    ///
    /// **Experimental**: This may have issues at the edges of regions.
    pub fn with_experimental_detail_mesh_generation(
        mut self,
        detail_mesh_generation_settings: DetailMeshSettings,
    ) -> Self {
        self.experimental_detail_mesh_generation = Some(detail_mesh_generation_settings);

        self
    }

    /// Returns the length of a tile's side in world units.
    #[inline]
    pub fn get_tile_size(&self) -> f32 {
        self.cell_width * f32::from(self.tile_width.get())
    }
    #[inline]
    pub fn get_border_size(&self) -> f32 {
        f32::from(self.walkable_radius) * self.cell_width
    }

    /// Returns the tile coordinate that contains the supplied ``world_position``.
    #[inline]
    pub fn get_tile_containing_position(&self, world_position: Vec2) -> UVec2 {
        let offset_world = world_position + self.world_half_extents;

        (offset_world / self.get_tile_size()).as_uvec2()
    }

    /// Returns the minimum bound of a tile on the XZ-plane.
    #[inline]
    pub fn get_tile_origin(&self, tile: UVec2) -> Vec2 {
        tile.as_vec2() * self.get_tile_size() - self.world_half_extents
    }

    /// Returns the origin of a tile on the XZ-plane including the border area.
    #[inline]
    pub fn get_tile_origin_with_border(&self, tile: UVec2) -> Vec2 {
        self.get_tile_origin(tile) - self.get_border_size()
    }

    #[inline]
    pub fn get_tile_side_with_border(&self) -> usize {
        usize::from(self.tile_width.get()) + usize::from(self.walkable_radius) * 2
    }
    #[inline]
    pub fn get_border_side(&self) -> usize {
        // Not technically useful currently but in case.
        self.walkable_radius.into()
    }

    /// Returns the minimum & maximum bound of a tile on the XZ-plane.
    #[inline]
    pub fn get_tile_bounds(&self, tile: UVec2) -> (Vec2, Vec2) {
        let tile_size = self.get_tile_size();

        let min_bound = tile.as_vec2() * tile_size - self.world_half_extents;
        let max_bound = min_bound + tile_size;

        (min_bound, max_bound)
    }
}

#[derive(Clone)]
pub struct DetailMeshSettings {
    /// The maximum acceptible error in height between the nav-mesh polygons & the true world (in cells).
    pub max_height_error: NonZeroU16,
    /// Determines how often (in cells) to sample the height when generating the height-corrected nav-mesh.
    ///
    /// This greatly affects generation performance. Higher values reduce samples by half to the previous one.
    /// Ex. 1.0, 0.5, 0.25, 0.125.
    ///
    /// **Suggested value:** >=2. Start high & reduce as needed.  
    pub sample_step: NonZeroU8,
}
