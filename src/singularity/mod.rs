//! Recovery of arm singularities and wrist poles.
//!
//! J1 can turn freely when the wrist center lies on the base rotation axis.
//! J2 can turn freely when equal effective arm lengths fold back onto the shoulder.
//! Both are free when that fully folded arm meets the shoulder on the base axis.
//! At a wrist pole, model J5 is zero or +/-pi and only the sum or difference
//! of J4 and J6 is fixed by the target orientation.
//! The solvers use wrist orientation and joint limits to select feasible solutions.

mod continuum;
pub(crate) mod j1free;
pub(crate) mod j1j2free;
pub(crate) mod j2free;
pub(crate) mod wrist_pole;
