// This file is borrowed and directly integrated from pathfinding = "4.11",
// https://crates.io/crates/pathfinding.
// It was available under Apache-2.0 OR MIT licenses.

// Changes were made by Bourumir Wyngs group starting from 5 November 2024 to
// integrate into rs-opw-kinematics projects:
//
//  - Removed no longer relevant examples from comments as this is not a specialized code.
//
//! Compute a shortest path using the [IDA* search
//! algorithm](https://en.wikipedia.org/wiki/Iterative_deepening_A*).

/// Compute a shortest path using the [IDA* search
/// algorithm](https://en.wikipedia.org/wiki/Iterative_deepening_A*).
///
/// The shortest path starting from `start` up to a node for which `success` returns `true` is
/// computed and returned along with its total cost, in a `Some`. If no path can be found, `None`
/// is returned instead.
///
/// - `start` is the starting node.
/// - `successors` returns a list of successors for a given node, along with the cost for moving
///   from the node to the successor. This cost must be non-negative.
/// - `heuristic` returns an approximation of the cost from a given node to the goal. The
///   approximation must not be greater than the real cost, or a wrong shortest path may be returned.
/// - `success` checks whether the goal has been reached. It is not a node as some problems require
///   a dynamic solution instead of a fixed node.
///
/// A node will never be included twice in the path as determined by the `Eq` relationship.
///
/// The returned path comprises both the start and end node.
///
pub fn idastar_algorithm<N, FN, IN, FH, FS>(
    start: &N,
    mut successors: FN,
    mut heuristic: FH,
    mut success: FS,
) -> Option<(Vec<N>, f64)>
where
    N: Eq + Clone,
    FN: FnMut(&N) -> IN,
    IN: IntoIterator<Item = (N, f64)>,
    FH: FnMut(&N) -> f64,
    FS: FnMut(&N) -> bool,
{
    let mut bound = heuristic(start);
    let mut path = vec![start.clone()];
    loop {
        match search(
            &mut path,
            0.0, // Use 0.0 for f64 instead of Zero::zero()
            bound,
            &mut successors,
            &mut heuristic,
            &mut success,
        ) {
            Path::Found(path, cost) => return Some((path, cost)),
            Path::Minimum(min) => {
                if bound == min {
                    return None;
                }
                bound = min;
            }
            Path::Impossible => return None,
        }
    }
}

enum Path<N> {
    Found(Vec<N>, f64),
    Minimum(f64),
    Impossible,
}

fn search<N, FN, IN, FH, FS>(
    path: &mut Vec<N>,
    cost: f64,
    bound: f64,
    successors: &mut FN,
    heuristic: &mut FH,
    success: &mut FS,
) -> Path<N>
where
    N: Eq + Clone,
    FN: FnMut(&N) -> IN,
    IN: IntoIterator<Item = (N, f64)>,
    FH: FnMut(&N) -> f64,
    FS: FnMut(&N) -> bool,
{
    let neighbs = {
        let start = &path[path.len() - 1];
        let f = cost + heuristic(start);
        if f > bound {
            return Path::Minimum(f);
        }
        if success(start) {
            return Path::Found(path.clone(), f);
        }
        let mut neighbs = successors(start)
            .into_iter()
            .filter_map(|(n, c)| {
                (!path.contains(&n)).then(|| {
                    let h = heuristic(&n);
                    (n, c, c + h)
                })
            })
            .collect::<Vec<_>>();
        neighbs.sort_unstable_by(|(_, _, c1), (_, _, c2)| c1.partial_cmp(c2).unwrap());
        neighbs
    };
    let mut min = None;
    for (node, extra, _) in neighbs {
        path.push(node);
        match search(path, cost + extra, bound, successors, heuristic, success) {
            found_path @ Path::Found(_, _) => return found_path,
            Path::Minimum(m) if min.map_or(true, |n| n > m) => min = Some(m),
            _ => (),
        }
        path.pop();
    }
    min.map_or(Path::Impossible, Path::Minimum)
}
