pub mod extent;
pub mod graph;
pub mod map_types;
pub mod room;
pub mod sampling;

mod symmetric_map;

use ilattice3::Point;
use serde::{Deserialize, Serialize};

/// Implement this to allow the procedural generation algorithms to write into your voxel map.
pub trait VoxelEncoder {
    /// `data` is the voxel data to write into `point`.
    fn encode_voxel(&mut self, point: &Point, data: &Voxel);
}

#[derive(Debug, Deserialize, Serialize)]
pub struct SpawnArea {
    pub valid_spawn_points: Vec<Point>,
}

pub struct Voxel {
    pub distance: f32,
    pub voxel_type: u8,
}
