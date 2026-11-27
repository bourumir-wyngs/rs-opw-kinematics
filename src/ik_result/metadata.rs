use std::fmt;

use super::{Branch, GeometryStatus};

/// A singularity in OPW model coordinates. Several can occur at the same pose.
#[repr(u8)]
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum Singularity {
    /// The wrist center lies on the base axis and J1 is free.
    FreeJ1,
    /// The folded arm places the wrist center on the shoulder axis and J2 is free.
    FreeJ2,
    /// The effective arm links are parallel at the outer reach boundary.
    ExtendedElbow,
    /// The effective arm links are antiparallel at the inner reach boundary.
    FoldedElbow,
    /// The two shoulder branches meet at the shoulder cylinder.
    ShoulderCylinderBoundary,
    /// Wrist pole at J5 = 0, where the target fixes J4 + J6.
    WristZero,
    /// Wrist pole at J5 = ±π, where the target fixes J4 - J6.
    WristPi,
}

/// A recovery method used to obtain the joint values stored in a branch slot.
#[repr(u8)]
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum Recovery {
    /// A free-arm search selected the base angle J1.
    FreeJ1Search,
    /// A free-arm search selected the shoulder angle J2.
    FreeJ2Search,
    /// The coupled wrist phase determined the selected J4/J6 pair.
    WristPhase,
}

/// One of the six rotary joints, numbered from the base to the flange.
#[repr(u8)]
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum Joint {
    /// First joint.
    J1,
    /// Second joint.
    J2,
    /// Third joint.
    J3,
    /// Fourth joint.
    J4,
    /// Fifth joint.
    J5,
    /// Sixth joint.
    J6,
}

impl Joint {
    /// Returns the zero-based index into a [`Joints`](crate::kinematic_traits::Joints) array.
    pub const fn index(self) -> usize {
        self as usize
    }
}

// Each set has at most eight members. Only typed members can enter its byte.
macro_rules! define_set {
    ($(#[$doc:meta])* $set:ident($item:ident), |$member:ident| $index:expr, $members:expr) => {
        $(#[$doc])*
        #[repr(transparent)]
        #[derive(Default, Clone, Copy, PartialEq, Eq, Hash)]
        pub struct $set(u8);

        impl $set {
            const fn bit($member: $item) -> u8 {
                1 << $index
            }

            /// An empty set.
            pub const fn empty() -> Self {
                Self(0)
            }

            /// Returns whether the set has no members.
            pub const fn is_empty(self) -> bool {
                self.0 == 0
            }

            /// Returns whether the given member belongs to this set.
            pub const fn contains(self, member: $item) -> bool {
                self.0 & Self::bit(member) != 0
            }

            /// Adds a member. Adding the same member twice leaves the set unchanged.
            pub fn insert(&mut self, member: $item) {
                self.0 |= Self::bit(member);
            }

            /// Removes a member if present.
            pub fn remove(&mut self, member: $item) {
                self.0 &= !Self::bit(member);
            }

            /// Yields the members present in ascending index order.
            pub fn iter(self) -> impl Iterator<Item = $item> {
                ($members).filter(move |&member| self.contains(member))
            }
        }

        impl FromIterator<$item> for $set {
            fn from_iter<I: IntoIterator<Item = $item>>(members: I) -> Self {
                let mut set = Self::empty();
                for member in members {
                    set.insert(member);
                }
                set
            }
        }

        impl fmt::Debug for $set {
            fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
                formatter.debug_set().entries(self.iter()).finish()
            }
        }
    };
}

define_set!(
    /// Singularities present in a branch, in [`Singularity`] declaration order.
    SingularitySet(Singularity),
    |s| s as u8,
    [
        Singularity::FreeJ1,
        Singularity::FreeJ2,
        Singularity::ExtendedElbow,
        Singularity::FoldedElbow,
        Singularity::ShoulderCylinderBoundary,
        Singularity::WristZero,
        Singularity::WristPi,
    ]
    .into_iter()
);

define_set!(
    /// Recovery methods used by a representative, in [`Recovery`] declaration order.
    RecoverySet(Recovery),
    |r| r as u8,
    [
        Recovery::FreeJ1Search,
        Recovery::FreeJ2Search,
        Recovery::WristPhase
    ]
    .into_iter()
);

define_set!(
    /// Analytical branches in the order of [`Branch::index`].
    BranchSet(Branch),
    |branch| branch.index(),
    (0..8).map(Branch)
);

define_set!(
    /// Rotary joints in J1 through J6 order.
    JointSet(Joint),
    |joint| joint.index(),
    [
        Joint::J1,
        Joint::J2,
        Joint::J3,
        Joint::J4,
        Joint::J5,
        Joint::J6
    ]
    .into_iter()
);

/// The outcome of checking one joint's configured angular range.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum JointLimitStatus {
    /// No effective limit was checked for this joint.
    Unchecked,
    /// The angle lies in the allowed range, including its boundaries.
    WithinLimits,
    /// The angle lies outside the allowed range.
    Violated,
}

/// Per-joint limit outcomes. All six joints start unchecked with [`Default::default`].
#[repr(C)]
#[derive(Debug, Default, Clone, Copy, PartialEq, Eq)]
pub struct JointLimitChecks {
    checked: JointSet,
    violated: JointSet,
}

impl JointLimitChecks {
    /// Returns the outcome for one joint.
    pub const fn status(self, joint: Joint) -> JointLimitStatus {
        if self.violated.contains(joint) {
            JointLimitStatus::Violated
        } else if self.checked.contains(joint) {
            JointLimitStatus::WithinLimits
        } else {
            JointLimitStatus::Unchecked
        }
    }

    /// Records the outcome for one joint, replacing any previous outcome.
    pub fn set(&mut self, joint: Joint, status: JointLimitStatus) {
        match status {
            JointLimitStatus::Unchecked => {
                self.checked.remove(joint);
                self.violated.remove(joint);
            }
            JointLimitStatus::WithinLimits => {
                self.checked.insert(joint);
                self.violated.remove(joint);
            }
            JointLimitStatus::Violated => {
                self.checked.insert(joint);
                self.violated.insert(joint);
            }
        }
    }

    /// All joints for which a limit was checked, including violations.
    pub const fn checked(self) -> JointSet {
        self.checked
    }

    /// The checked joints whose angles were outside their allowed ranges.
    pub const fn violated(self) -> JointSet {
        self.violated
    }
}

/// Branch identity, singularities, recovery and joint-limit diagnostics for one slot.
///
/// # Examples
///
/// Given an [`IKResults`](super::IKResults) array, read a branch's diagnostics
/// with `let meta = &results[branch.index()].meta;`.
///
/// In this example, the two wrist branches meet at J5 = 0. Wrist-phase recovery
/// provided a representative whose forward pose verifies. J4 is within its limit,
/// while J3 is outside its allowed range.
///
/// ```
/// use rs_opw_kinematics::ik_result::{
///     Branch, Elbow, GeometryStatus, IKMetadata, Joint, JointLimitChecks, JointLimitStatus,
///     Recovery, Shoulder, Singularity, Wrist,
/// };
///
/// let principal = Branch::new(Shoulder::Second, Elbow::Positive, Wrist::Principal);
/// let flipped = Branch::new(Shoulder::Second, Elbow::Positive, Wrist::Flipped);
/// let mut limits = JointLimitChecks::default();
/// limits.set(Joint::J3, JointLimitStatus::Violated);
/// limits.set(Joint::J4, JointLimitStatus::WithinLimits);
///
/// let meta = IKMetadata {
///     branch: principal,
///     geometry: GeometryStatus::Verified,
///     singularities: [Singularity::WristZero].into_iter().collect(),
///     recovery: [Recovery::WristPhase].into_iter().collect(),
///     family: [principal, flipped].into_iter().collect(),
///     limits,
///     limiting_joint: Some(Joint::J3),
/// };
///
/// // Identify the singularity and how the joint angles were recovered.
/// assert!(meta.singularities.contains(Singularity::WristZero));
/// assert!(meta.recovery.contains(Recovery::WristPhase));
///
/// // Iterate the branches in the same family, in IKResults array order.
/// let slots: Vec<_> = meta.family.iter().map(Branch::index).collect();
/// assert_eq!(slots, [2, 6]);
///
/// // Inspect one joint, list all violations, or read the joint with the smallest margin.
/// assert_eq!(meta.limits.status(Joint::J4), JointLimitStatus::WithinLimits);
/// assert_eq!(meta.limits.status(Joint::J6), JointLimitStatus::Unchecked);
/// assert_eq!(meta.limits.status(Joint::J3), JointLimitStatus::Violated);
/// assert_eq!(meta.limits.violated().iter().collect::<Vec<_>>(), [Joint::J3]);
/// assert_eq!(meta.limiting_joint, Some(Joint::J3)); // None when no margin is finite.
/// ```
#[repr(C)]
#[derive(Debug, Clone, Copy)]
pub struct IKMetadata {
    /// Analytical branch assigned before filtering or ranking candidates.
    pub branch: Branch,
    /// Geometric outcome for this slot. A verified pose can still violate joint limits.
    pub geometry: GeometryStatus,
    /// Singularities identified for the branch in OPW model coordinates.
    pub singularities: SingularitySet,
    /// Recovery methods that contributed to the stored joint values.
    pub recovery: RecoverySet,
    /// Branches in the same geometric family, including this branch. An ordinary
    /// solution has a singleton family. Branches that meet at a wrist pole can
    /// have different representatives. An empty set means no family was identified.
    pub family: BranchSet,
    /// Limit outcomes for the stored joint angles.
    pub limits: JointLimitChecks,
    /// Joint with the smallest signed limit margin, or `None` if no margin is
    /// finite. Ties select the joint with the lowest index.
    pub limiting_joint: Option<Joint>,
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::ik_result::{Elbow, Shoulder, Wrist};

    #[test]
    fn family_and_singularities_retain_distinct_members_in_order() {
        let principal = Branch::new(Shoulder::Second, Elbow::Positive, Wrist::Principal);
        let flipped = Branch::new(Shoulder::Second, Elbow::Positive, Wrist::Flipped);
        let family: BranchSet = [flipped, principal, flipped].into_iter().collect();
        assert_eq!(family.iter().map(Branch::index).collect::<Vec<_>>(), [2, 6]);

        let mut singularities = SingularitySet::empty();
        singularities.insert(Singularity::WristZero);
        singularities.insert(Singularity::FreeJ1);
        assert_eq!(
            singularities.iter().collect::<Vec<_>>(),
            [Singularity::FreeJ1, Singularity::WristZero]
        );
        assert!(!singularities.contains(Singularity::WristPi));

        let mut recovery: RecoverySet = [Recovery::FreeJ1Search, Recovery::WristPhase]
            .into_iter()
            .collect();
        recovery.remove(Recovery::FreeJ1Search);
        assert_eq!(recovery.iter().collect::<Vec<_>>(), [Recovery::WristPhase]);
    }

    #[test]
    fn joint_limit_outcomes_are_independent_and_replaceable() {
        use JointLimitStatus::{Unchecked, Violated, WithinLimits};

        let mut limits = JointLimitChecks::default();
        assert_eq!(limits.status(Joint::J4), Unchecked);
        limits.set(Joint::J6, WithinLimits);
        limits.set(Joint::J4, Violated);
        assert_eq!(limits.status(Joint::J4), Violated);
        assert_eq!(
            limits.checked().iter().collect::<Vec<_>>(),
            [Joint::J4, Joint::J6]
        );
        assert_eq!(limits.violated().iter().collect::<Vec<_>>(), [Joint::J4]);

        limits.set(Joint::J4, WithinLimits);
        assert_eq!(limits.status(Joint::J4), WithinLimits);
        assert!(limits.violated().is_empty());
        limits.set(Joint::J4, Violated);
        limits.set(Joint::J4, Unchecked);
        assert_eq!(limits.status(Joint::J4), Unchecked);
        assert!(limits.violated().is_empty());
        assert_eq!(limits.checked().iter().collect::<Vec<_>>(), [Joint::J6]);
    }
}
