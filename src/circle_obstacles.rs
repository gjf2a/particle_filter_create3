use particle_filter::{Degrees, FloatPoint, ObstacleMap, PoseEstimate, RobotPose};

use crate::{Bump, CREATE3_RADIUS};

#[derive(Clone, Debug, Default)]
pub struct CircleObstacles {
    obstacles: Vec<CircleObstacle>,
    total_strike_distance: f64,
    num_strikes: usize,
}

impl CircleObstacles {
    pub fn obstacle_extremes(&self) -> BoundingBox {
        self.obstacles.iter().map(|ob| ob.center).collect()
    }
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

#[derive(Default, Clone, Copy, Debug)]
pub struct BoundingBox {
    min_x: f64,
    max_x: f64,
    min_y: f64, 
    max_y: f64,
}

impl FromIterator<FloatPoint> for BoundingBox {
    fn from_iter<T: IntoIterator<Item = FloatPoint>>(iter: T) -> Self {
        let mut result = Self::default();
        for point in iter {
            if result.min_x > point[0] {
                result.min_x = point[0];
            }
            if result.max_x < point[0] {
                result.max_x = point[0];
            }
            if result.min_y > point[1] {
                result.min_y = point[1];
            }
            if result.max_y < point[1] {
                result.max_y = point[1];
            }
        }
        result
    }
}

impl ObstacleMap for CircleObstacles {
    type SensorType = Bump;

    fn error(&mut self, estimated_pose: &PoseEstimate) -> f64 {
        let pose: RobotPose = (*estimated_pose).into();
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

    fn mean_stdev(&self, sensor_info: Option<&Self::SensorType>) -> (f64, Degrees) {
        match sensor_info {
            None => (7e-4, Degrees::new(2e-4)),
            Some(_) => (0.16, Degrees::new(3.1)),
        }
        /*
        match sensor_info {
            //SensorInfo::Pose(_) => (7e-4, Degrees::new(2.0)),
            SensorInfo::Pose(_) => (7e-5, Degrees::new(2e-4)),
            //SensorInfo::Bump(_) => (0.16, Degrees::new(3.1)),
            SensorInfo::Bump(_) => (0.00016, Degrees::new(0.0031)),
        }*/
    }

    fn sensor_update(
        &mut self,
        estimated_pose: &PoseEstimate,
        sensor_info: Option<&Self::SensorType>,
    ) {
        if let Some(bump) = sensor_info {
            let pose: RobotPose = (*estimated_pose).into();
            let heading = pose.theta + bump.angle_offset();
            let offset: FloatPoint = (CREATE3_RADIUS, heading).into();
            self.obstacles.push(CircleObstacle {
                center: pose.pos + offset,
                radius: CREATE3_RADIUS,
            });
        }
    }
}
