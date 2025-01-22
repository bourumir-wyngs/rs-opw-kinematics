/*
```
LIN_INTERP | TRACE: 114.23, 73.80, -137.84, 175.93, 3.65, 93.93
TRACE: 112.64, 73.84, -138.09, 170.91, 2.35, 98.92
LIN_INTERP | TRACE: 111.27, 73.87, -138.30, 157.80, 1.23, 111.97
TRACE: 109.91, 73.90, -138.51, 84.74, 0.57, 184.99
TRACE: 108.29, 73.92, -138.57, 12.82, 1.22, 257.05
TRACE: 106.99, 73.94, -138.32, 2.97, 1.21, 267.00
TRACE: 105.14, 73.94, -137.90, 43.43, 0.83, 226.28
TRACE: 104.83, 73.93, -137.17, 164.89, 1.14, 104.97
TRACE: 104.72, 73.89, -136.29, 176.49, 3.35, 93.41
```


```
TRACE: 179.67, 74.15, -136.09, -5.01, -2.83, -85.11 
TRACE: 179.67, 74.20, -136.84, -12.68, -1.12, -77.44 
TRACE: 179.67, 74.23, -137.50, -143.48, -0.41, 53.36 
TRACE: 179.67, 74.24, -137.87, -161.21, -0.75, 71.09 
TRACE: 179.76, 74.24, -137.80, -18.93, -0.53, -71.16 
```
 */
use crate::annotations::{AnnotatedJoints, PathFlags};
use crate::kinematic_traits::{Joints, J4, J5, J6};

pub struct SuperRotation {
    /// Maximal angle of J5 under that super-rotation is considered
    pub j5_max: f64,

    /// Maximal compensating delta angle. This must occur on both
    /// j4 and J6 in opposite directions.
    pub j4j6delta_min: f64,
}

impl SuperRotation {
    pub fn new(j5_max: f64, j4j6delta_min: f64) -> Self {
        Self {
            j5_max,
            j4j6delta_min,
        }
    }

    pub fn is_super_rotation(&self, prev: &Joints, now: &Joints) -> bool {
        fn sgn(x: f64) -> i32 {
            if x > 0.0 {
                1
            } else if x < 0.0 {
                -1
            } else {
                0
            }
        }

        let r4 = now[J4] - prev[J4];
        let r6 = now[J6] - prev[J6];
        if sgn(r4) == sgn(r6) {
            // Joints rotate same direction
            return false;
        };
        let j4j6 = r4.abs().min(r6.abs());

        // J5 is close to 0 (close to singularity not yet at)
        prev[J5].abs().min(now[J5].abs()) <= self.j5_max
            // J4 and J6 rotate in opposite directions enough            
            && j4j6 >= self.j4j6delta_min
    }

    pub fn fix(&self, trajectory: &Vec<AnnotatedJoints>) -> Vec<AnnotatedJoints> {
        // Handle trajectories with 2 or fewer elements directly.
        if trajectory.len() <= 2 {
            return trajectory.clone();
        }

        let mut filtered = Vec::with_capacity(trajectory.len());

        // Always include the first element
        filtered.push(trajectory[0].clone());

        // Iterate through consecutive pairs using `windows`
        for pair in trajectory.windows(2) {
            //filtered.push(pair[0].clone()); // Always include the first element of the pair

            // Optionally modify the second element
            if self.is_super_rotation(&pair[0].joints, &pair[1].joints) {
                // If it is super-rotation, do not move J4 and J6 at all. 
                let mut modified_second = pair[1].clone();
                modified_second.joints[J4] = pair[0].joints[J4];
                modified_second.joints[J6] = pair[0].joints[J6];
                modified_second.flags.insert(PathFlags::DEBUG);
                filtered.push(modified_second);
            } else {
                // Include the unmodified second element
                filtered.push(pair[1].clone());
            }
        }

        // Always include the last element of the trajectory
        filtered.push(trajectory.last().unwrap().clone());

        filtered
    }
}
