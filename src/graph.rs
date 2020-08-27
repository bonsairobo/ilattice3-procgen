use petgraph::{
    algo::tarjan_scc,
    graph::NodeIndex,
    stable_graph::StableGraph,
    visit::{depth_first_search, Control, DfsEvent, EdgeRef},
    EdgeType, Undirected,
};
use std::collections::HashSet;
use std::iter::FromIterator;

pub fn induced_subgraph<N: Clone, E: Clone, Ty: EdgeType, S: std::hash::BuildHasher>(
    graph: &StableGraph<N, E, Ty>,
    nodes: &HashSet<NodeIndex, S>,
) -> StableGraph<N, E, Ty> {
    graph.filter_map(
        |i, n| {
            if nodes.contains(&i) {
                Some(n.clone())
            } else {
                None
            }
        },
        |_, e| Some(e.clone()),
    )
}

/// Returns Some iff the subgraph is a proper subgraph.
pub fn largest_connected_subgraph<N, E, Ty>(
    graph: &StableGraph<N, E, Ty>,
) -> Option<StableGraph<N, E, Ty>>
where
    N: Clone,
    E: Clone,
    Ty: EdgeType,
{
    let components = tarjan_scc(&graph);
    if components.len() > 1 {
        let largest_component = components
            .iter()
            .max_by_key(|c| c.len())
            .expect("Must have at least one component");
        let largest_component_set: HashSet<NodeIndex> =
            HashSet::from_iter(largest_component.iter().cloned());

        Some(induced_subgraph(&graph, &largest_component_set))
    } else {
        None
    }
}

/// Removes nodes with the fewest edges until the desired number of nodes is reached.
/// `accept_fn` allows external constraints to be enforced, preventing certain nodes from being
/// removed.
pub fn prune_outer_nodes_to_reach_size<N: Clone, E: Clone>(
    graph: &mut StableGraph<N, E, Undirected>,
    accept_fn: impl Fn(&StableGraph<N, E, Undirected>) -> bool,
    desired_size: usize,
) {
    let mut max_edges_per_removed_node = 1;
    loop {
        let mut removed_nodes = false;
        let all_node_indices: Vec<_> = graph.node_indices().collect();
        for n in all_node_indices.iter() {
            if graph.node_count() <= desired_size {
                return;
            }

            let edges: Vec<_> = graph
                .edges(*n)
                .map(|e| (e.source(), e.target(), e.weight().clone()))
                .collect();
            if edges.len() == max_edges_per_removed_node {
                let node_weight = graph.remove_node(*n).expect("Node must exist");

                // Removed nodes can disconnect parts of the graph, so make sure we don't violate
                // some other constraint.
                let subgraph = largest_connected_subgraph(graph);
                let check_graph = if let Some(subgraph_ref) = subgraph.as_ref() {
                    subgraph_ref
                } else {
                    &*graph
                };
                if accept_fn(check_graph) {
                    removed_nodes = true;
                    if let Some(g) = subgraph {
                        *graph = g;
                    }
                } else {
                    // Revert the node removal.
                    graph.add_node(node_weight);
                    for (src, tgt, weight) in edges.into_iter() {
                        graph.add_edge(src, tgt, weight);
                    }
                }
            }
        }

        if !removed_nodes {
            max_edges_per_removed_node += 1;
        }
    }
}

/// Assumes `graph` is a tree.
pub fn longest_path_to_point_in_tree<N, E>(
    graph: &StableGraph<N, E, Undirected>,
    target: NodeIndex,
) -> Vec<NodeIndex> {
    let mut predecessors = vec![NodeIndex::end(); graph.node_count()];
    let mut successors = vec![NodeIndex::end(); graph.node_count()];
    depth_first_search(graph, Some(target), |event| {
        if let DfsEvent::TreeEdge(u, v) = event {
            predecessors[v.index()] = u;
            successors[u.index()] = v;
        }

        Control::<()>::Continue
    });

    let no_succesors: Vec<_> = graph
        .node_indices()
        .filter(|i| successors[i.index()] == NodeIndex::end())
        .collect();

    let mut max_path = Vec::new();
    for i in no_succesors.iter() {
        let mut next = *i;
        let mut path = vec![*i];
        loop {
            let pred = predecessors[next.index()];
            path.push(pred);
            if pred == target {
                if path.len() > max_path.len() {
                    max_path = path;
                }
                break;
            }
            next = pred;
        }
    }

    max_path
}

/// Assumes `graph` is a tree.
pub fn longest_path_in_tree<N, E>(graph: &StableGraph<N, E, Undirected>) -> Vec<NodeIndex> {
    // Do two DFS searches for the longest path from a node in order to find the longest path.
    let start = graph
        .node_indices()
        .next()
        .expect("Must have at least one node");
    let path = longest_path_to_point_in_tree(graph, start);

    longest_path_to_point_in_tree(graph, *path.first().expect("Must have at least one node"))
}
