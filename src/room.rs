use crate::{sampling::sample_range, symmetric_map::SymmetricMap, SpawnArea, Voxel, VoxelEncoder};

use ilattice3::{
    normal::{Direction, DirectionIndex, Normal, PlaneSpanInfo, ALL_DIRECTIONS},
    Extent, Point,
};
use petgraph::{
    stable_graph::StableGraph,
    visit::{EdgeRef, IntoEdgeReferences},
    Undirected,
};
use rand::Rng;

pub const EMPTY_VOXEL: Voxel = Voxel {
    distance: std::f32::MAX,
    voxel_type: 0,
};

pub const FLOOR_VOXEL: Voxel = Voxel {
    distance: -1.0,
    voxel_type: 1,
};

pub fn fill_map_with_rooms(rooms: &[Extent], encoder: &mut impl VoxelEncoder) {
    let wall_thickness = 5;
    for r in rooms.iter() {
        let r_interior = r.radial_grow(-wall_thickness);
        for p in r {
            if !r_interior.contains_world(&p) {
                // TODO: check the plane of the wall to determine if it's a floor, ceiling, etc.
                encoder.encode_voxel(&p, &FLOOR_VOXEL);
            }
        }
    }
}

pub fn fill_map_with_doors(doors: &[Extent], encoder: &mut impl VoxelEncoder) {
    for d in doors.iter() {
        for p in d {
            encoder.encode_voxel(&p, &EMPTY_VOXEL);
        }
    }
}

/// Returns the extent where a doorway could be sliced between two rooms.
pub fn get_door_able_extent_for_rooms(r1: &Extent, r2: &Extent) -> Option<(Extent, Direction)> {
    let pen = Extent::penetrations(r1, r2);

    let zeroes: Vec<_> = pen
        .iter()
        .filter_map(|(d, p)| if *p == 0 { Some(d) } else { None })
        .collect();

    // We can only make a door if there is exactly one direction where the penetration depth is
    // zero. If there we two, then the rooms would have an intersecting edge. If there were
    // three, an intersecting corner.
    if zeroes.len() != 1 {
        return None;
    }

    let dir = zeroes[0];
    let neg_dir = dir.negate();

    // Make sure the door doesn't eat up the boundaries of the wall.
    let mut grow_by = DirectionIndex::zeroes();
    for d in ALL_DIRECTIONS.iter() {
        if *d != dir && *d != neg_dir {
            *grow_by.get_mut(*d) = -1;
        }
    }

    // Grow the rooms into each other along the door normal so we can just get their
    // intersection as the door extent.
    let mut r1_grow_by = grow_by;
    *r1_grow_by.get_mut(neg_dir) = 1;
    let mut r2_grow_by = grow_by;
    *r2_grow_by.get_mut(dir) = 1;

    let grown_r1 = r1.directional_grow(&r1_grow_by);
    let grown_r2 = r2.directional_grow(&r2_grow_by);

    Some((grown_r1.intersection(&grown_r2), dir))
}

pub fn try_generate_door_big_enough_between_rooms(
    min_door_dim: u32,
    max_door_dim: u32,
    r1: &Extent,
    r2: &Extent,
    rng: &mut impl Rng,
) -> Option<Extent> {
    let (extent, dir) = if let Some((extent, dir)) = get_door_able_extent_for_rooms(r1, r2) {
        (extent, dir)
    } else {
        return None;
    };

    if extent.is_empty() {
        return None;
    }

    // Choose a random 2 x N x M door inside the given extent where there is space for a door.

    let n = Normal::Axis(dir.positive());
    let PlaneSpanInfo { u, v } = n.get_plane_span_info();

    let sup = extent.get_local_supremum();
    let u_sup = sup.dot(&u);
    let v_sup = sup.dot(&v);

    if (u_sup < min_door_dim as i32) || (v_sup < min_door_dim as i32) {
        return None;
    }

    let min = extent.get_minimum();
    let u_min = min.dot(&u);
    let v_min = min.dot(&v);

    // TODO: if the door is in a wall (not a ceiling or floor), then make the bottom of the
    // door touch the floor
    let (door_u_min, door_u_max) = sample_range(rng, u_min, u_min + u_sup - 1);
    let (door_v_min, door_v_max) = sample_range(rng, v_min, v_min + v_sup - 1);
    let door_u_sup = (1 + door_u_max - door_u_min)
        .min(max_door_dim as i32)
        .max(min_door_dim as i32);
    let door_v_sup = (1 + door_v_max - door_v_min)
        .min(max_door_dim as i32)
        .max(min_door_dim as i32);

    let n = Point::from(n);

    let door_min = n * min.dot(&n) + u * door_u_min + v * door_v_min;
    let door_sup = n * 2 + u * door_u_sup + v * door_v_sup;
    let door = Extent::from_min_and_local_supremum(door_min, door_sup);

    // It's possible that the random door escapes the valid extent due to size clamping.
    if door.is_subset(&extent) {
        Some(door)
    } else {
        None
    }
}

pub fn generate_door_graph(
    rooms: &[Extent],
    min_door_dim: u32,
    max_door_dim: u32,
    rng: &mut impl Rng,
    doors: &mut SymmetricMap<Extent>,
) -> StableGraph<usize, (), Undirected> {
    let mut graph = StableGraph::default();
    for i in 0..rooms.len() {
        graph.add_node(i);
    }
    let all_node_indices: Vec<_> = graph.node_indices().collect();
    for i in all_node_indices.iter() {
        for j in all_node_indices.iter() {
            let i_idx = graph[*i];
            let j_idx = graph[*j];

            if j_idx <= i_idx {
                // Don't visit the same undirected edge twice.
                continue;
            }

            // TODO: maybe retry?
            if let Some(door) = try_generate_door_big_enough_between_rooms(
                min_door_dim,
                max_door_dim,
                &rooms[i_idx],
                &rooms[j_idx],
                rng,
            ) {
                // It seems like too much overhead to put the door extents into the graph edges,
                // since we copy the graph elements a lot.
                doors.insert(i_idx, j_idx, door);
                graph.add_edge(*i, *j, ());
            }
        }
    }

    graph
}

pub fn collect_rooms_from_room_graph(
    room_candidates: &[Extent],
    room_graph: &StableGraph<usize, (), Undirected>,
) -> Vec<Extent> {
    room_graph
        .node_indices()
        .map(|i| room_candidates[room_graph[i]])
        .collect()
}

pub fn collect_doors_from_room_graph(
    doors: &SymmetricMap<Extent>,
    room_graph: &StableGraph<usize, (), Undirected>,
) -> Vec<Extent> {
    room_graph
        .edge_references()
        .map(|e| *doors.get(room_graph[e.source()], room_graph[e.target()]))
        .collect()
}

/// The 1xNxM extent inside the room walls just above the floor.
/// BUG: doesn't account for doors in the floor
pub fn spawn_in_room(room: &Extent) -> SpawnArea {
    let internal_boundary = room.directional_grow(&DirectionIndex::new([-1; 6]));
    let mut sup = *internal_boundary.get_local_supremum();
    sup.y = 1;
    let mut hero_spawn_area = internal_boundary;
    hero_spawn_area.set_local_supremum(sup);

    SpawnArea {
        valid_spawn_points: hero_spawn_area.into_iter().collect(),
    }
}

// ████████╗███████╗███████╗████████╗███████╗
// ╚══██╔══╝██╔════╝██╔════╝╚══██╔══╝██╔════╝
//    ██║   █████╗  ███████╗   ██║   ███████╗
//    ██║   ██╔══╝  ╚════██║   ██║   ╚════██║
//    ██║   ███████╗███████║   ██║   ███████║
//    ╚═╝   ╚══════╝╚══════╝   ╚═╝   ╚══════╝

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_get_door_able_extent_for_rooms_successful() {
        let r1 = Extent::from_min_and_local_supremum([0, 0, 0].into(), [4, 4, 4].into());
        let r2 = Extent::from_min_and_local_supremum([0, 0, -4].into(), [4, 4, 4].into());

        assert_eq!(
            get_door_able_extent_for_rooms(&r1, &r2),
            Some((
                Extent::from_min_and_local_supremum([1, 1, -1].into(), [2, 2, 2].into(),),
                Direction::PosZ,
            )),
        );
    }

    #[test]
    fn test_get_door_able_extent_for_rooms_unsuccessful() {
        let r1 = Extent::from_min_and_local_supremum([0, 0, 0].into(), [4, 4, 4].into());
        let r2 = Extent::from_min_and_local_supremum([0, -4, -4].into(), [4, 4, 4].into());

        assert_eq!(get_door_able_extent_for_rooms(&r1, &r2), None);
    }
}
