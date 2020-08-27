use crate::{
    extent::resolve_extent_overlaps,
    graph::{largest_connected_subgraph, longest_path_in_tree, prune_outer_nodes_to_reach_size},
    room::{
        collect_doors_from_room_graph, collect_rooms_from_room_graph, fill_map_with_doors,
        fill_map_with_rooms, generate_door_graph, spawn_in_room,
    },
    sampling::{sample_extents, LatticeNormalDistSpec, LatticeUniformDistSpec},
    symmetric_map::SymmetricMap,
    SpawnArea, VoxelEncoder,
};

use fnv::FnvHashSet;
use ilattice3::Extent;
use petgraph::{
    algo::min_spanning_tree,
    data::FromElements,
    dot::{Config, Dot},
    stable_graph::StableGraph,
    visit::IntoNodeReferences,
    Undirected,
};
use rand::prelude::*;
use serde::{Deserialize, Serialize};
use std::iter::FromIterator;

pub const MAX_GENERATE_TRIES: usize = 200;

#[derive(Debug, Deserialize, Serialize)]
pub struct DungeonMeta {
    pub spawn_area: SpawnArea,
}

#[derive(Clone, Default, Deserialize, Serialize)]
pub struct RoomGraphSpec {
    pub num_rooms: usize,
    pub entrance_to_objective_path_length: usize,
}

#[derive(Clone, Default, Deserialize, Serialize)]
pub struct RoomDistributionSpec {
    pub location: LatticeUniformDistSpec,
    pub size: LatticeNormalDistSpec,
}

#[derive(Clone, Default, Deserialize, Serialize)]
pub struct DungeonMapSpec {
    pub seed: [u32; 4],
    pub room_graph: RoomGraphSpec,
    pub room_dist: RoomDistributionSpec,
    pub min_room_dim: u32,
    pub max_room_dim: u32,
    pub min_door_dim: u32,
    pub max_door_dim: u32,
}

impl DungeonMapSpec {
    fn valid_room_size(&self, room: &Extent) -> bool {
        let dims = room.get_local_supremum();

        *dims >= [self.min_room_dim as i32; 3].into()
            && *dims <= [self.max_room_dim as i32; 3].into()
    }

    fn generate_room_candidates(&self, rng: &mut impl Rng) -> Vec<Extent> {
        sample_extents(
            10 * self.room_graph.num_rooms,
            |r: &Extent| self.valid_room_size(r),
            self.room_dist.location.make(),
            self.room_dist.size.make(),
            rng,
        )
    }

    /// Returns true iff we were able to remove exactly enough rooms to hit the desired room count.
    fn prune_rooms_to_desired_size(
        &self,
        main_path: &[usize],
        room_graph: &mut StableGraph<usize, (), Undirected>,
    ) -> bool {
        // Preserve the rooms on the main path.
        let mut keep_nodes = FnvHashSet::default();
        let keep_rooms = FnvHashSet::<usize>::from_iter(main_path.iter().cloned());
        for (n, room_idx) in room_graph.node_references() {
            if keep_rooms.contains(room_idx) {
                keep_nodes.insert(n);
            }
        }
        let accept_fn = |graph_after_node_removal: &StableGraph<usize, (), Undirected>| {
            // Make sure all of the main path rooms remain in the graph.
            for n in keep_nodes.iter() {
                if !graph_after_node_removal.contains_node(*n) {
                    return false;
                }
            }

            true
        };

        prune_outer_nodes_to_reach_size(room_graph, accept_fn, self.room_graph.num_rooms);
        log::debug!(
            "{} rooms after pruning outer nodes",
            room_graph.node_count()
        );
        log::debug!(
            "After pruning = {:?}",
            Dot::with_config(&room_graph.clone(), &[Config::EdgeNoLabel])
        );

        room_graph.node_count() >= self.room_graph.num_rooms
    }

    /// On success, returns `Some` and writes the generated voxels into `encoder`. Leaves the
    /// encoder untouched on failure.
    pub fn try_generate(
        &self,
        rng: &mut impl Rng,
        encoder: &mut impl VoxelEncoder,
    ) -> Option<DungeonMeta> {
        log::debug!("Generating dungeon map");

        let mut room_candidates = self.generate_room_candidates(rng);
        log::debug!("Generated {} room candidates", room_candidates.len());

        resolve_extent_overlaps(&mut room_candidates);
        log::debug!("Done resolving room overlaps");

        let mut doors = SymmetricMap::new();
        let mut room_graph = generate_door_graph(
            &room_candidates,
            self.min_door_dim,
            self.max_door_dim,
            rng,
            &mut doors,
        );

        // Prune disconnected rooms.
        if let Some(subgraph) = largest_connected_subgraph(&room_graph) {
            if subgraph.node_count() < self.room_graph.num_rooms {
                return None;
            } else {
                room_graph = subgraph;
            }
        }
        log::debug!("{} connected rooms", room_graph.node_count());

        let mst = StableGraph::from_elements(min_spanning_tree(&room_graph));
        log::debug!(
            "MST before pruning = {:?}",
            Dot::with_config(&mst, &[Config::EdgeNoLabel])
        );

        let main_path = choose_main_path(self.room_graph.entrance_to_objective_path_length, &mst)?;
        log::debug!("Main path = {:?}", main_path);

        // Make sure we keep at least the main path nodes.
        self.prune_rooms_to_desired_size(&main_path, &mut room_graph);

        let chosen_rooms = collect_rooms_from_room_graph(&room_candidates, &room_graph);
        let chosen_doors = collect_doors_from_room_graph(&doors, &room_graph);

        fill_map_with_rooms(&chosen_rooms, encoder);
        fill_map_with_doors(&chosen_doors, encoder);

        let spawn_area = spawn_in_room(&room_candidates[*main_path.last().unwrap()]);
        log::debug!("Spawn area = {:?}", spawn_area);

        Some(DungeonMeta { spawn_area })
    }

    pub fn generate(&self, rng: &mut impl Rng, encoder: &mut impl VoxelEncoder) -> DungeonMeta {
        for _ in 0..MAX_GENERATE_TRIES {
            if let Some(meta) = self.try_generate(rng, encoder) {
                return meta;
            }
        }

        panic!(
            "Failed to generate dungeon after {} tries",
            MAX_GENERATE_TRIES
        );
    }
}

/// Returns vec of room indices.
fn choose_main_path(
    desired_len: usize,
    mst: &StableGraph<usize, (), Undirected>,
) -> Option<Vec<usize>> {
    let path = longest_path_in_tree(mst);

    if path.len() >= desired_len {
        Some(path[0..desired_len - 1].iter().map(|n| mst[*n]).collect())
    } else {
        None
    }
}
