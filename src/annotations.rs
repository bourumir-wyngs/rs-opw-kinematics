use crate::kinematic_traits::{Joints, Pose};
use bitflags::bitflags;
use nalgebra::{Isometry3, Translation3};
use parry3d::math::Rotation;
use parry3d::na;
use std::fmt;

bitflags! {
    /// Flags that can be set on AnnotatedJoints in the output
    #[derive(Clone, Copy)]
    pub struct PathFlags: u32 {
        const NONE = 0b0000_0000;

        /// Position is part of the movement from the initil ("home") pose to the landing
        /// pose. It may include very arbitrary joint movements, so Cartesian stroke
        /// may not work on this part of the trajectory.
        const ONBOARDING =          0b0000_0010;

        /// Position directly matches one of the stroke poses given in the input.
        const TRACE =               0b0000_0100;

        /// Position is a linear interpolation between two poses of the trace. These poses
        /// are not needed for the robots that have built-in support for Cartesian stroke,
        /// but may be important for more developed models that only rotate between
        /// the given joint positions without guarantees that TCP movement is linear.
        const LIN_INTERP =          0b0000_1000;

        /// Position corresponds the starting pose ("land") that is normally little above
        /// the start of the required trace
        const LAND =                0b0001_0000;

        /// The tool is movind down from the landing position to the first stroke position.
        /// Activate the tool mechanism if any (start the drill, open sprayer, etc).
        const LANDING =             0b0010_0000;

        /// Position corresponds the ennding pose ("park") that is normally little above
        /// the end of the required trace. This is the last pose to include into the
        /// output.
        const PARK =                0b0100_0000;

        /// The tool is moving up from the last trace point to the parking position
        /// (shut down the effector mechanism if any)
        const PARKING =             0b1000_0000;

        /// Used with raster projector, indicates the movement considered "forwards"
        const FORWARDS =             0b1000_0001;

        /// Used with raster projector, indicates the movement considered "backwards"
        const BACKWARDS =             0b1000_0010;

        ///  Used with raster projector, indicates change of directions (first point in new direction)
        const U_TURN =             0b1000_0011;

        /// Combined flag representing the "original" position, so the one that was
        /// given in the input.
        const ORIGINAL = Self::TRACE.bits() | Self::LAND.bits() | Self::PARK.bits();

        /// Special flag used in debugging to mark out anything of interest
        const DEBUG = 0b1000_0000_0000_0000;

        // The movement INTO this pose is Cartesian stroke
        const CARTESIAN = Self::LIN_INTERP.bits() | Self::LAND.bits() | Self::PARK.bits();
    }
}

#[derive(Clone, Copy)]
pub struct AnnotatedPose {
    pub pose: Pose,
    pub flags: PathFlags,
}

/// Annotated joints specifying if it is joint-joint or Cartesian move (to this joint, not from)
#[derive(Clone, Copy)]
pub struct AnnotatedJoints {
    pub joints: Joints,
    pub flags: PathFlags,
}

#[derive(Clone, Copy, Debug)]
pub struct AnnotatedPathStep {
    pub point: (f32, f32),
    pub flags: PathFlags,
}

impl AnnotatedPose {
    pub(crate) fn interpolate(&self, other: &AnnotatedPose, p: f64) -> AnnotatedPose {
        assert!((0.0..=1.0).contains(&p));

        // Interpolate translation (linearly)
        let self_translation = &self.pose.translation.vector;
        let other_translation = &other.pose.translation.vector;

        let translation = self_translation.lerp(&other_translation, p);
        let rotation = self.pose.rotation.slerp(&other.pose.rotation, p);

        AnnotatedPose {
            pose: Pose::from_parts(Translation3::from(translation), rotation),
            flags: PathFlags::LIN_INTERP,
        }
    }

    // Create a new instance of annotated pose that will always be 64 bit pose.
    pub fn from_parts<T: na::RealField + parry3d::simba::scalar::SubsetOf<f64>>(
        translation: Translation3<T>,
        rotation: Rotation<T>,
    ) -> AnnotatedPose {
        AnnotatedPose {
            pose: Isometry3::from_parts(translation.cast::<f64>(), rotation.cast::<f64>()),
            flags: PathFlags::NONE,
        }
    }
}

fn flag_representation(flags: &PathFlags) -> String {
    const FLAG_MAP: &[(PathFlags, &str)] = &[
        // Interpolation flags
        (PathFlags::LIN_INTERP, "LIN_INTERP"),
        // Cartesian planning flags
        (PathFlags::LAND, "LAND"),
        (PathFlags::LANDING, "LANDING"),
        (PathFlags::PARK, "PARK"),
        (PathFlags::PARKING, "PARKING"),
        (PathFlags::TRACE, "TRACE"),
        (PathFlags::CARTESIAN, "CARTESIAN"),
        // Joints-to-joints movement (RRT) flag
        (PathFlags::ONBOARDING, "ONBOARDING"),
        // Projector flags
        (PathFlags::DEBUG, "DEBUG"),
    ];

    FLAG_MAP
        .iter()
        .filter(|(flag, _)| flags.contains(*flag))
        .map(|(_, name)| *name)
        .collect::<Vec<_>>()
        .join(" | ")
}
impl fmt::Debug for PathFlags {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let flag_string = flag_representation(self);
        write!(f, "{}", flag_string)
    }
}

impl fmt::Debug for AnnotatedPose {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        let translation = self.pose.translation.vector;
        let rotation = self.pose.rotation;
        write!(
            formatter,
            "{}: [{:.3}, {:.3}, {:.3}], quat {{ w: {:.3}, i: {:.3}, j: {:.3}, k: {:.3} }}",
            flag_representation(&self.flags),
            translation.x,
            translation.y,
            translation.z,
            rotation.w,
            rotation.i,
            rotation.j,
            rotation.k
        )
    }
}

impl fmt::Debug for AnnotatedJoints {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(
            formatter,
            "{}: {:.2}, {:.2}, {:.2}, {:.2}, {:.2}, {:.2} ",
            flag_representation(&self.flags),
            self.joints[0].to_degrees(),
            self.joints[1].to_degrees(),
            self.joints[2].to_degrees(),
            self.joints[3].to_degrees(),
            self.joints[4].to_degrees(),
            self.joints[5].to_degrees(),
        )
    }
}
