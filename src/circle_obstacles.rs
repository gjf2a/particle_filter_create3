use particle_filter::{FloatPoint, Particle, RobotPose};

use crate::{CREATE3_RADIUS, SensorInfo};

#[derive(Clone, Debug, Default)]
pub struct CircleObstacles {
    obstacles: Vec<CircleObstacle>,
    pose: RobotPose,
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

    fn error(&self) -> f64 {
        self.total_strike_distance
    }

    fn pose(&self) -> RobotPose {
        self.pose
    }

    fn set_pose(&mut self, new_pose: particle_filter::nums::RobotPose) {
        self.pose = new_pose;
    }

    fn sensor_update(&mut self, sensor_info: &Self::SensorType) {
        match sensor_info {
            SensorInfo::Pose(robot_pose) => {
                self.pose = *robot_pose; // THIS IS WRONG! ADD THE DIFFERENCE FROM LAST POSE ESTIMATE!
                for strike in self
                    .obstacles
                    .iter()
                    .filter_map(|ob| ob.strike_depth(self.pose.pos))
                {
                    self.total_strike_distance += strike;
                    self.num_strikes += 1;
                }
            }
            SensorInfo::Bump(bump) => {
                let heading = self.pose.theta + bump.angle_offset();
                self.obstacles.push(CircleObstacle {
                    center: (CREATE3_RADIUS, heading).into(),
                    radius: CREATE3_RADIUS,
                });
            }
        }
    }
}
