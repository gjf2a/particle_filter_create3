use particle_filter::{Particle, RobotPose, point::{FloatPoint, Point}};

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

    fn update<N: Fn(RobotPose, &Self::SensorType) -> RobotPose>(
        &mut self,
        sensor_info: &Self::SensorType,
        noise_func: N,
    ) {
        match sensor_info {
            SensorInfo::Pose(robot_pose) => {
                self.pose = noise_func(*robot_pose, sensor_info);
                for strike in self.obstacles.iter().filter_map(|ob| ob.strike_depth(self.pose.pos)) {
                    self.total_strike_distance += strike;
                    self.num_strikes += 1;
                }
            }
            SensorInfo::Bump(bump) => {
                let heading = self.pose.theta + bump.angle_offset();
                let x = CREATE3_RADIUS * heading.cos();
                let y = CREATE3_RADIUS * heading.sin();
                self.obstacles.push(CircleObstacle { center: Point::new([x, y]), radius: CREATE3_RADIUS });
            }
        }
    }
}