//! Branch-indexed results for six-axis OPW inverse kinematics.
//!
//! Each slot identifies a choice in the analytical equations. Its index stays
//! fixed across poses, joint limits and continuation references. At a singularity
//! several slots can refer to one family of configurations. A slot holds one
//! representative of that family, which can depend on the reference configuration.

#![allow(dead_code, reason = "The solver does not produce these records yet.")]

use crate::kinematic_traits::Joints;

/// The eight analytical branch slots, in the same order as CEAD's reach kernel.
///
/// The index is `4 * wrist_flip + 2 * shoulder + elbow`. In OPW model
/// coordinates, the first/second shoulder uses the positive/negative radial
/// root, and the positive/negative elbow uses the corresponding sign of the
/// arccosine term in J3. The principal wrist has a J5 bend in `[0, π]`; the
/// flipped wrist takes its negative. These choices precede joint offsets, axis
/// sign corrections and whole-turn adjustments.
#[repr(u8)]
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub(crate) enum BranchId {
    FirstPositivePrincipal = 0,
    FirstNegativePrincipal = 1,
    SecondPositivePrincipal = 2,
    SecondNegativePrincipal = 3,
    FirstPositiveFlipped = 4,
    FirstNegativeFlipped = 5,
    SecondPositiveFlipped = 6,
    SecondNegativeFlipped = 7,
}

/// Outcome of solving the pose equations for one branch, independent of whether
/// the representative meets the configured joint limits.
#[repr(u8)]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) enum GeometryStatus {
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
pub(crate) struct IKMetadata {
    /// Analytical branch assigned before filtering or ranking candidates.
    pub(crate) branch: BranchId,
    /// Geometric outcome for this slot. A verified pose can still violate joint limits.
    pub(crate) geometry: GeometryStatus,
    /// Singularities of the branch in model coordinates. Bits 0 through 6 mean,
    /// respectively, free J1, free J2, extended elbow, folded elbow, shoulder-cylinder
    /// boundary, J5 at zero, and J5 at ±π. Conditions can coincide; bit 7 is reserved.
    pub(crate) singularities: u8,
    /// Recovery used to obtain the stored joint values. Bit 0 means J1 was
    /// selected by a free-arm search, bit 1 means J2 was selected by such a search,
    /// and bit 2 means J4/J6 were recovered from the coupled wrist phase.
    /// Zero denotes no recovery. An attempted but unused recovery sets no bit.
    pub(crate) recovery: u8,
    /// Bit `i` identifies slot `i` as belonging to the same geometric family,
    /// including this slot. A regular solution has only its own bit set. For
    /// example, slots 2 and 6 at a wrist pole both carry `0x44`, even if they
    /// hold different representatives. Zero means no family was identified.
    pub(crate) family_slots: u8,
    /// Effective joint limits evaluated for the stored joints. Bits 0 through 5
    /// correspond to J1 through J6; the upper two bits are reserved.
    pub(crate) limits_checked: u8,
    /// Subset of `limits_checked` for which the stored angle was outside the
    /// permitted range. An angle exactly on an inclusive boundary passes.
    pub(crate) limits_violated: u8,
    /// Index (0 = J1) of the joint with the smallest signed margin. Ties select
    /// the lowest index. `u8::MAX` means there is no finite margin.
    pub(crate) limiting_joint: u8,
}

/// One fixed slot in a complete six-axis inverse-kinematics result.
#[repr(C)]
#[derive(Debug, Clone, Copy)]
pub(crate) struct IKResult {
    /// Six angles in radians, in the robot's joint coordinates. Verified and
    /// failed-pose-check candidates retain their finite angles, including angles
    /// that violate limits. With no real root or no representative, all six are NaN.
    pub(crate) joints: Joints,
    /// Minimum signed angular distance, in radians, to an evaluated joint-limit
    /// boundary. Each joint contributes positive clearance inside its allowed arc,
    /// zero on the boundary, or negative distance to the arc outside it. Thus a
    /// negative minimum describes the worst violation. Periodic limits use angular
    /// distance around the circle. An evaluated configuration with no effective
    /// limits has margin +∞; an unevaluated margin is NaN.
    pub(crate) limit_margin_rad: f64,
    /// Identity and diagnostics for the stored representative.
    pub(crate) meta: IKMetadata,
}

/// All eight branch slots in [`BranchId`] order, including missing roots and
/// failed candidates. Entry `i` always has branch discriminant `i`.
pub(crate) type IKResults = [IKResult; 8];

const _: () = assert!(size_of::<IKMetadata>() == 8);
const _: () = assert!(size_of::<IKResult>() == 64);
