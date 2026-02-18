use particle_filter::{Degrees, FloatPoint, Particle, RobotPose};

use crate::{CREATE3_RADIUS, SensorInfo};

#[derive(Clone, Debug, Default)]
pub struct CircleObstacles {
    obstacles: Vec<CircleObstacle>,
    total_strike_distance: f64,
    num_strikes: usize,
}

#[derive(Clone, Debug)]
pub struct CircleObstacle {
    center: FloatPoint,
    radius: f64,
}

impl CircleObstacle {
    pub fn strike_depth(&self, pos: FloatPoint) -> Option<f64> {
        let dist = pos.euclidean_distance(self.center);
        if dist < self.radius {
            Some(self.radius - dist)
        } else {
            None
        }
    }
}

impl Particle for CircleObstacles {
    type SensorType = SensorInfo;

    fn error(&mut self, pose: &RobotPose) -> f64 {
        for strike in self
            .obstacles
            .iter()
            .filter_map(|ob| ob.strike_depth(pose.pos))
        {
            self.total_strike_distance += strike;
            self.num_strikes += 1;
        }
        self.total_strike_distance
    }

    fn mean_stdev(&self, sensor_info: &Self::SensorType) -> (f64, Degrees) {
        match sensor_info {
            SensorInfo::Pose(_) => (7e-4, Degrees::new(2.0)),
            SensorInfo::Bump(_) => (0.16, Degrees::new(3.1)),
        }
    }

    fn sensor_update(&mut self, estimated_pose: &RobotPose, sensor_info: &Self::SensorType) {
        match sensor_info {
            SensorInfo::Pose(_) => {}
            SensorInfo::Bump(bump) => {
                let heading = estimated_pose.theta + bump.angle_offset();
                self.obstacles.push(CircleObstacle {
                    center: (CREATE3_RADIUS, heading).into(),
                    radius: CREATE3_RADIUS,
                });
            }
        }
    }
}
