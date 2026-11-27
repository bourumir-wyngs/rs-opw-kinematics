//! Branch-indexed results for six-axis OPW inverse kinematics.
//!
//! Each slot identifies a choice in the analytical equations. Its index stays
//! fixed across poses, joint limits and continuation references. At a singularity
//! several slots can refer to one family of configurations. A representative
//! of that family can depend on the reference configuration.
//!
//! Use `results[branch.index()]` to look up a branch in the eight-element
//! [`IKResults`] array. For example, this function reads
//! the joints for one branch if its pose was verified and it passes all checked limits.
//!
//! ```
//! use rs_opw_kinematics::ik_result::{Branch, Elbow, GeometryStatus, IKResults, Shoulder, Wrist};
//! use rs_opw_kinematics::kinematic_traits::Joints;
//!
//! fn joints_for(results: &IKResults, branch: Branch) -> Option<&Joints> {
//!     let slot = &results[branch.index()];
//!     (slot.meta.geometry == GeometryStatus::Verified && slot.meta.limits_violated == 0)
//!         .then_some(&slot.joints)
//! }
//!
//! let branch = Branch::new(Shoulder::Second, Elbow::Positive, Wrist::Flipped);
//! assert_eq!(branch.index(), 6);
//! assert_eq!(branch.wrist(), Wrist::Flipped);
//! ```

use crate::kinematic_traits::Joints;

/// Radial root of the OPW shoulder equations in model coordinates.
#[repr(u8)]
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum Shoulder {
    /// Positive radial root.
    First = 0,
    /// Negative radial root.
    Second = 1,
}

/// Sign of the arccosine term in the OPW equation for J3.
#[repr(u8)]
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum Elbow {
    /// Add the arccosine term.
    Positive = 0,
    /// Subtract the arccosine term.
    Negative = 1,
}

/// Wrist solution in OPW model coordinates.
#[repr(u8)]
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum Wrist {
    /// Principal J5 bend in `[0, π]`.
    Principal = 0,
    /// Negated principal J5 bend, with the corresponding half-turn shifts in J4 and J6.
    Flipped = 1,
}

/// One of the eight analytical branches, identified by its shoulder, elbow and
/// wrist choices before joint offsets, axis sign corrections and whole-turn adjustments.
#[repr(transparent)]
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct Branch(u8);

impl Branch {
    /// Constructs the branch for the given shoulder root, elbow root and wrist solution.
    pub const fn new(shoulder: Shoulder, elbow: Elbow, wrist: Wrist) -> Self {
        Self(((wrist as u8) << 2) | ((shoulder as u8) << 1) | elbow as u8)
    }

    /// Returns this branch's shoulder root.
    pub const fn shoulder(self) -> Shoulder {
        if self.0 & 0b010 == 0 {
            Shoulder::First
        } else {
            Shoulder::Second
        }
    }

    /// Returns this branch's elbow root.
    pub const fn elbow(self) -> Elbow {
        if self.0 & 0b001 == 0 {
            Elbow::Positive
        } else {
            Elbow::Negative
        }
    }

    /// Returns this branch's wrist solution, including at a pole where the two wrist branches meet.
    pub const fn wrist(self) -> Wrist {
        if self.0 & 0b100 == 0 {
            Wrist::Principal
        } else {
            Wrist::Flipped
        }
    }

    /// Returns the zero-based array index (`0..8`) for this branch.
    /// Use `results[branch.index()]` to access its slot in an [`IKResults`] array.
    ///
    /// The index is `4 * wrist + 2 * shoulder + elbow` using the components'
    /// zero-or-one discriminants. This is the order used by
    /// [CEAD's reach kernel](https://github.com/CEAD-group/py-opw-kinematics/blob/7e16424d9a8c359c098faa74403af6325ad61c2c/src/reach.rs#L50-L56).
    pub const fn index(self) -> usize {
        self.0 as usize
    }
}

/// Outcome of solving the pose equations for one branch, independent of whether
/// the representative meets the configured joint limits.
#[repr(u8)]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum GeometryStatus {
    /// Forward kinematics of the stored joints reproduces the requested pose
    /// within the solver's position and orientation tolerances.
    Verified,
    /// The analytical equations have no real root for this branch at this pose.
    NoRealRoot,
    /// The solver obtained finite joint values, but their forward pose failed verification.
    FailedPoseCheck,
    /// A singular family was identified, but the search found no representative
    /// under the supplied conditions. Other members of the family may exist.
    NoRepresentativeFound,
}

/// Branch identity, singularities and joint-limit diagnostics for one slot.
#[repr(C)]
#[derive(Debug, Clone, Copy)]
pub struct IKMetadata {
    /// Analytical branch assigned before filtering or ranking candidates.
    pub branch: Branch,
    /// Geometric outcome for this slot. A verified pose can still violate joint limits.
    pub geometry: GeometryStatus,
    /// Singularities of the branch in OPW model coordinates. Conditions can coincide.
    ///
    /// | Bit | Condition |
    /// | --- | --- |
    /// | 0 | Free base rotation (J1) |
    /// | 1 | Free shoulder rotation (J2) |
    /// | 2 | Fully extended elbow |
    /// | 3 | Fully folded elbow |
    /// | 4 | Shoulder-cylinder boundary |
    /// | 5 | Wrist pole at J5 = 0 |
    /// | 6 | Wrist pole at J5 = ±π |
    /// | 7 | Reserved |
    pub singularities: u8,
    /// Recovery used to obtain the stored joint values. Bit 0 means J1 was
    /// selected by a free-arm search, bit 1 means J2 was selected by such a search,
    /// and bit 2 means J4/J6 were recovered from the coupled wrist phase.
    /// Zero denotes no recovery. An attempted but unused recovery sets no bit.
    /// Bits 3 through 7 are reserved.
    pub recovery: u8,
    /// Bit `i` identifies slot `i` as belonging to the same geometric family,
    /// including this slot. A regular solution has only its own bit set. For
    /// example, when only slots 2 and 6 meet at a wrist pole, they both carry
    /// `0x44`, even if they hold different representatives. Zero means no family
    /// was identified.
    pub family_slots: u8,
    /// Effective joint limits evaluated for the stored joints. Bits 0 through 5
    /// correspond to J1 through J6; the upper two bits are reserved.
    pub limits_checked: u8,
    /// Subset of [`limits_checked`](Self::limits_checked) for which the stored
    /// angle was outside the permitted range. An inclusive boundary passes.
    pub limits_violated: u8,
    /// Index (0 = J1) of the joint with the smallest signed margin. Ties select
    /// the lowest index. `u8::MAX` means there is no finite margin.
    pub limiting_joint: u8,
}

/// One fixed slot in a complete six-axis inverse-kinematics result.
#[repr(C)]
#[derive(Debug, Clone, Copy)]
pub struct IKResult {
    /// Six angles in radians, in the robot's joint coordinates. Verified and
    /// failed-pose-check candidates retain their finite angles, including angles
    /// that violate limits. With no real root or no representative, all six are NaN.
    pub joints: Joints,
    /// Minimum signed angular distance, in radians, to an evaluated joint-limit
    /// boundary. Each joint contributes positive clearance inside its allowed arc,
    /// zero on the boundary, or negative distance to the arc outside it. Thus a
    /// negative minimum describes the worst violation. Periodic limits use angular
    /// distance around the circle. An evaluated configuration with no effective
    /// limits has margin +∞; an unevaluated margin is NaN.
    pub limit_margin_rad: f64,
    /// Identity and diagnostics for the stored representative.
    pub meta: IKMetadata,
}

/// All eight branch slots in [`Branch`] order, including missing roots and
/// failed candidates. Store each entry at `entry.meta.branch.index()`.
pub type IKResults = [IKResult; 8];

const _: () = assert!(size_of::<Branch>() == 1);
const _: () = assert!(size_of::<IKMetadata>() == 8);
const _: () = assert!(size_of::<IKResult>() == 64);

#[cfg(test)]
mod tests {
    use super::{Branch, Elbow, Shoulder, Wrist};

    #[test]
    fn branch_components_match_all_eight_slots() {
        use Elbow::{Negative, Positive};
        use Shoulder::{First, Second};
        use Wrist::{Flipped, Principal};

        let slots = [
            (First, Positive, Principal),
            (First, Negative, Principal),
            (Second, Positive, Principal),
            (Second, Negative, Principal),
            (First, Positive, Flipped),
            (First, Negative, Flipped),
            (Second, Positive, Flipped),
            (Second, Negative, Flipped),
        ];
        for (index, (shoulder, elbow, wrist)) in slots.into_iter().enumerate() {
            let branch = Branch::new(shoulder, elbow, wrist);
            assert_eq!(branch.index(), index);
            assert_eq!(
                (branch.shoulder(), branch.elbow(), branch.wrist()),
                (shoulder, elbow, wrist)
            );
        }
    }
}
