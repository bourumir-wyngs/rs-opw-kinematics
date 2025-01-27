use crate::kinematic_traits::{Joints, Pose};
use bitflags::bitflags;
use nalgebra::{Isometry3, Translation3, Unit, UnitQuaternion, Vector3};
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
        const ONBOARDING =          1 << 1;

        /// Position directly matches one of the stroke poses given in the input.
        const STROKE =               1 << 2;

        /// Position is a linear interpolation between two poses of the trace. These poses
        /// are not needed for the robots that have built-in support for Cartesian stroke,
        /// but may be important for more developed models that only rotate between
        /// the given joint positions without guarantees that TCP movement is linear.
        const LIN_INTERP =          1 << 3;

        /// Position corresponds the starting pose ("land") that is normally little above
        /// the start of the required trace
        const LAND =                1 << 4;

        /// The tool is movind down from the landing position to the first stroke position.
        /// Activate the tool mechanism if any (start the drill, open sprayer, etc).
        const LANDING =             1 << 5;

        /// Position corresponds the ennding pose ("park") that is normally little above
        /// the end of the required trace. This is the last pose to include into the
        /// output.
        const PARK =                1 << 6;

        /// The tool is moving up from the last trace point to the parking position
        /// (shut down the effector mechanism if any)
        const PARKING =             1 << 7;

        /// Used with raster projector, indicates the movement considered "forwards"
        const FORWARDS =             1 << 8;

        /// Used with raster projector, indicates the movement considered "backwards"
        const BACKWARDS =            1 << 9;

        /// Closing with RRT gap that is not possible to transition with linear stroke
        const GAP_CLOSING =              1 << 10;

        /// Combined flag representing the "original" position, so the one that was
        /// given in the input.
        const ORIGINAL = Self::STROKE.bits() | Self::LAND.bits() | Self::PARK.bits();

        /// Special flag used in debugging to mark out anything of interest. Largest can be stored
        /// in u32
        const DEBUG = 1 << 31;

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
    pub x: f32,
    pub y: f32,
    pub flags: PathFlags,
}

impl AnnotatedPose {
    /// Move the pose "away" from the target along local quaternion z axis.
    /// Positive value is meant to move away, negative - closer to the target.
    pub fn elevate(&self, dz: f64) -> AnnotatedPose {
        // Extract the rotation component as a UnitQuaternion
        let rotation = &self.pose.rotation;

        // Determine the local Z-axis direction (quaternion's orientation)
        let local_z_axis = rotation.transform_vector(&Vector3::z());

        // Compute the new translation by adding dz along the local Z-axis
        let translation = self.pose.translation.vector + dz * local_z_axis;

        // Return a new Isometry3 with the updated translation and the same rotation
        AnnotatedPose {
            pose: Isometry3::from_parts(translation.into(), rotation.clone()),
            flags: self.flags,
        }
    }

    /// Twist around the local coordinate system (not the global xyz) applying roll, pitch and yaw    
    pub fn twist(&self, roll: f64, pitch: f64, yaw: f64) -> AnnotatedPose {
        use once_cell::sync::Lazy;
        static X_AXIS: Lazy<Unit<Vector3<f64>>> = Lazy::new(|| Unit::new_normalize(*Vector3::x_axis()));
        static Y_AXIS: Lazy<Unit<Vector3<f64>>> = Lazy::new(|| Unit::new_normalize(*Vector3::y_axis()));
        static Z_AXIS: Lazy<Unit<Vector3<f64>>> = Lazy::new(|| Unit::new_normalize(*Vector3::z_axis()));

        // Combine the pilot's inputs into a single local rotation

        let local_rotation = UnitQuaternion::from_axis_angle(&X_AXIS, roll) *
            UnitQuaternion::from_axis_angle(&Y_AXIS, pitch) *
            UnitQuaternion::from_axis_angle(&Z_AXIS, yaw);

        let global_rotation = self.pose.rotation * local_rotation;

        // Return the updated pose
        return Self::from_parts(self.pose.translation, global_rotation)
    }

    pub fn interpolate(&self, other: &AnnotatedPose, p: f64) -> AnnotatedPose {
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
        (PathFlags::LIN_INTERP, "LI"),
        // Cartesian planning flags
        (PathFlags::LAND, "LAND"),
        (PathFlags::LANDING, "LANDING"),
        (PathFlags::PARK, "PARK"),
        (PathFlags::PARKING, "PARKING"),
        (PathFlags::STROKE, "STROKE"),
        (PathFlags::CARTESIAN, "CARTESIAN"),
        // Joints-to-joints movement (RRT) flag
        (PathFlags::ONBOARDING, "ONBOARDING"),
        // Mesh generator flags
        (PathFlags::FORWARDS, "FORWARDS"),
        (PathFlags::BACKWARDS, "BACKWARDS"),
        (PathFlags::GAP_CLOSING, "GAP_CLOSING"),
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
