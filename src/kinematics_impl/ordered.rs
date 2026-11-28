//! Preserve analytical branch identity through recovery and forward verification.

use super::*;
use crate::ik_result::{Branch, GeometryStatus, IKResult, Recovery, SingularitySet};

/// One wrist root together with the method that produced it. A coupled-phase
/// candidate can also be tried at a small but resolvable nonzero bend.
pub(super) struct WristChoice {
    pub(super) angles: WristBranch,
    pub(super) wrist: Wrist,
    pub(super) recovered: bool,
    pub(super) resolved_pole: bool,
}

impl WristChoice {
    fn pole(&self, confirmed_by_pose: bool) -> Option<Singularity> {
        (self.resolved_pole || confirmed_by_pose).then_some(if self.angles.q5 == 0.0 {
            Singularity::WristZero
        } else {
            Singularity::WristPi
        })
    }
}

impl ArmBranch {
    fn matches(self, exact: ArmBranch) -> bool {
        // At a double root the arm-angle error grows as sqrt(roundoff).
        let tolerance = ARM_ROUNDOFF.sqrt();
        [
            (self.q1, exact.q1),
            (self.q2, exact.q2),
            (self.q3, exact.q3),
        ]
        .into_iter()
        .all(|(actual, target)| wrapped_angle(actual - target).abs() <= tolerance)
    }
}

impl ArmRecovery {
    fn singularities(&self, arm: usize) -> SingularitySet {
        let mut singularities = SingularitySet::empty();
        if self.free_j1 {
            singularities.insert(Singularity::FreeJ1);
        }
        if self.free_j2[arm / 2] {
            singularities.insert(Singularity::FreeJ2);
        }
        if self.shoulder_boundary {
            singularities.insert(Singularity::ShoulderCylinderBoundary);
        }
        if let Some(boundary) = self.elbow_boundary[arm / 2] {
            singularities.insert(boundary);
        }
        singularities
    }
}

impl OPWKinematics {
    pub(super) fn ordered_ik(&self, pose: &Pose, reference: Option<&Joints>) -> IKResults {
        if self.parameters.dof != 6 {
            return IKResult::empty_slots(GeometryStatus::Unsupported);
        }
        let reference = reference.unwrap_or_else(|| self.constraint_centers());
        if !pose.is_valid() || !reference.iter().all(|angle| angle.is_finite()) {
            return IKResult::empty_slots(GeometryStatus::InvalidInput);
        }

        let mut results = IKResult::empty_slots(GeometryStatus::NoRealRoot);
        let recovery = self.arm_branches(pose);
        for (index, arm) in recovery.branches.iter().enumerate() {
            // At a fully folded equal-length arm the ordinary q2 equation
            // divides by zero. The free-J2 search still has a real arm family.
            if !(arm.q1.is_finite() && arm.q2.is_finite() && arm.q3.is_finite())
                && !recovery.free_j2[index / 2]
            {
                continue;
            }
            for slot in [index, index + 4] {
                results[slot].meta.geometry = GeometryStatus::NoRepresentativeFound;
                results[slot].meta.singularities = recovery.singularities(index);
            }
        }

        let matrix = RotationMatrix::from_quat(pose.rotation);
        let near = J4J6Near::from_joints(reference, &self.parameters);
        let mut wrist_families = [false; 4];
        let mut visit = |indices: Range<usize>, mut arm: ArmBranch, searched: bool| {
            let mut choices = Self::wrist_choices(&matrix, arm, &near).peekable();
            let mut confirmed_pole = false;
            // The overlap yields its coupled-phase candidate first. Test the
            // pole hypothesis against the position equations independently of
            // the chosen wrist phase or reference.
            if let Some(first) = choices.peek()
                && first.recovered
                && !first.resolved_pole
                && let Some(refined) = self.arm_at_wrist_pole(pose, arm, first.angles.q5)
            {
                arm = refined;
                confirmed_pole = true;
                choices = Self::wrist_choices(&matrix, arm, &near).peekable();
            }
            for choice in choices {
                if confirmed_pole && !choice.recovered {
                    continue;
                }
                let start = indices.start + 4 * choice.wrist as usize;
                let pole = choice.pole(confirmed_pole);
                let candidate = self.ordered_candidate(
                    pose,
                    reference,
                    arm,
                    &choice,
                    searched,
                    results[start].meta.branch,
                );
                for index in indices.clone() {
                    let slot = &mut results[index + 4 * choice.wrist as usize];
                    if slot.meta.geometry == GeometryStatus::NoRealRoot {
                        continue;
                    }
                    // The family can visit a pole at a different free-arm
                    // angle from the representative ultimately stored here.
                    if let Some(pole) = pole {
                        wrist_families[index] = true;
                        if !slot.joints[0].is_finite() {
                            slot.meta.singularities.insert(pole);
                        }
                    }
                    let Some(mut candidate) = candidate else {
                        continue;
                    };
                    candidate.meta.branch = slot.meta.branch;
                    candidate.meta.singularities = recovery.singularities(index);
                    if let Some(pole) = pole {
                        candidate.meta.singularities.insert(pole);
                    }
                    if searched && recovery.free_j1 {
                        candidate.meta.recovery.insert(Recovery::FreeJ1Search);
                    }
                    if searched && recovery.free_j2[index / 2] {
                        candidate.meta.recovery.insert(Recovery::FreeJ2Search);
                    }
                    if prefer(&candidate, slot, reference) {
                        *slot = candidate;
                    }
                }
            }
        };

        self.visit_singular_arms(pose, &recovery, reference, None, |indices, arm| {
            visit(indices, arm, true)
        });
        for (index, arm) in recovery.branches.into_iter().enumerate() {
            if arm.q1.is_finite() && arm.q2.is_finite() && arm.q3.is_finite() {
                visit(index..index + 1, arm, false);
            }
        }
        assign_families(&mut results, wrist_families);
        results
    }
    /// Validate once per recovered arm/wrist pair, even when several
    /// coincident arm branches share the same representative.
    fn ordered_candidate(
        &self,
        pose: &Pose,
        reference: &Joints,
        arm: ArmBranch,
        choice: &WristChoice,
        searched: bool,
        branch: Branch,
    ) -> Option<IKResult> {
        let mut joints = match self.wrist_to_joints(arm, choice.angles, reference, None) {
            Some(joints) => joints,
            None if !choice.recovered => {
                // A physical turn may be unrepresentable at extreme absolute
                // limits. Retain the finite root and diagnose its limits.
                self.user_joints(arm, choice.angles)
            }
            None => return None,
        };
        for (i, joint) in joints.iter_mut().enumerate() {
            if !self.has_absolute_limit(i) && (*joint - reference[i]).abs() > PI {
                normalize_near(joint, reference[i]);
            }
        }
        if !joints.iter().all(|angle| angle.is_finite()) {
            return None;
        }
        if searched && let Some(limits) = self.constraints.as_ref() {
            self.snap_singular_limits(&mut joints, limits, None);
        }
        let mut candidate = IKResult::empty(branch, GeometryStatus::FailedPoseCheck);
        candidate.joints = joints;
        if self.try_forward(&joints).is_ok_and(|actual| {
            (actual.translation - pose.translation).length() <= self.distance_tolerance
                && actual.angular_distance(*pose) <= ANGULAR_TOLERANCE
        }) {
            candidate.meta.geometry = GeometryStatus::Verified;
        }
        candidate.check_limits(self.constraints.as_ref(), &joints);
        if choice.recovered {
            candidate.meta.recovery.insert(Recovery::WristPhase);
        }
        Some(candidate)
    }

    /// At a pole, the tool axis fixes the arm's J1 and J2+J3 directions.
    /// Solve the arm from these directions and check the shoulder offset and
    /// upper-arm length against the measured wrist center at roundoff scale.
    /// This avoids the ill-conditioned acos at a nearly extended/folded elbow.
    fn arm_at_wrist_pole(&self, pose: &Pose, arm: ArmBranch, pole_q5: f64) -> Option<ArmBranch> {
        let p = &self.parameters;
        let tool_axis = pose.rotation * DVec3::Z;
        let wrist = pose.translation - p.c4 * tool_axis;
        let axis = if pole_q5 == 0.0 {
            tool_axis
        } else {
            -tool_axis
        };

        // A vertical axis leaves J1 undetermined. Keep the analytical J1
        // whenever its radial plane already contains the axis to roundoff.
        let (s1, c1) = arm.q1.sin_cos();
        let q1 = if (axis.y * c1 - axis.x * s1).abs() <= WRIST_ROUNDOFF_THR {
            arm.q1
        } else {
            let azimuth = axis.y.atan2(axis.x);
            if (azimuth - arm.q1).cos() >= 0.0 {
                azimuth
            } else {
                azimuth + PI
            }
        };
        let (s1, c1) = q1.sin_cos();
        let q23 = (axis.x * c1 + axis.y * s1).atan2(axis.z);
        let (s23, c23) = q23.sin_cos();
        // Subtract the model-space forearm from the wrist center. What
        // remains must be c2 * (sin(q2), cos(q2)) in the radial plane.
        let x = wrist.x * c1 + wrist.y * s1 - p.a1 - (p.c3 * s23 + p.a2 * c23);
        let z = wrist.z - p.c1 - (p.c3 * c23 - p.a2 * s23);
        let tolerance = ARM_ROUNDOFF * self.distance_tolerance / RELATIVE_DISTANCE_TOLERANCE;
        if (wrist.y * c1 - wrist.x * s1 - p.b).abs() > tolerance
            || (x.hypot(z) - p.c2.abs()).abs() > tolerance
        {
            return None;
        }
        let q2 = (x * p.c2.signum()).atan2(z * p.c2.signum());
        let refined = ArmBranch {
            q1,
            q2,
            q3: q23 - q2,
        };
        arm.matches(refined).then_some(refined)
    }
}

/// Choose one representative within a slot. In the wrist-overlap band, both
/// phase recovery and the regular root can verify. Preserve a resolvable bend
/// whenever its regular representative also satisfies the configured limits.
fn prefer(candidate: &IKResult, current: &IKResult, reference: &Joints) -> bool {
    if !current.joints[0].is_finite() {
        return true;
    }
    let rank = |slot: &IKResult| {
        (
            slot.meta.geometry == GeometryStatus::Verified,
            slot.meta.limits.violated().is_empty(),
            !slot.meta.recovery.contains(Recovery::WristPhase)
                || slot.meta.singularities.contains(Singularity::WristZero)
                || slot.meta.singularities.contains(Singularity::WristPi),
        )
    };
    match rank(candidate).cmp(&rank(current)) {
        std::cmp::Ordering::Equal => {
            calculate_distance(&candidate.joints, reference)
                < calculate_distance(&current.joints, reference)
        }
        ordering => ordering.is_gt(),
    }
}

/// At a shoulder, elbow or wrist coalescence, the corresponding pair of
/// labels belongs to one family. Component closure also handles simultaneous
/// singularities, including different representatives of a free arm.
fn assign_families(results: &mut IKResults, wrist_families: [bool; 4]) {
    let mut group = Branch::ALL.map(Branch::index);
    for slot in results.iter() {
        if slot.meta.geometry == GeometryStatus::NoRealRoot {
            continue;
        }
        let index = slot.meta.branch.index();
        let conditions = slot.meta.singularities;
        // The three independent choices occupy the shoulder, elbow and wrist
        // bits in Branch::index(). Toggling a bit selects its paired branch.
        for (bit, joined) in [
            (
                2,
                conditions.contains(Singularity::FreeJ1)
                    || conditions.contains(Singularity::ShoulderCylinderBoundary),
            ),
            (
                1,
                conditions.contains(Singularity::FoldedElbow)
                    || conditions.contains(Singularity::ExtendedElbow)
                    || conditions.contains(Singularity::FreeJ2),
            ),
            (4, wrist_families[index % 4]),
        ] {
            let other = index ^ bit;
            if joined && results[other].meta.geometry != GeometryStatus::NoRealRoot {
                let (keep, merge) = (group[index], group[other]);
                for label in &mut group {
                    if *label == merge {
                        *label = keep;
                    }
                }
            }
        }
    }
    for slot in results.iter_mut() {
        if slot.meta.geometry != GeometryStatus::NoRealRoot {
            slot.meta.family = Branch::ALL
                .into_iter()
                .filter(|other| group[other.index()] == group[slot.meta.branch.index()])
                .collect();
        }
    }
}
