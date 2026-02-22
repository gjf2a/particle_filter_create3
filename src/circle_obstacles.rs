use particle_filter::{BoundingBox, FloatPoint, Noise, ObstacleMap, Radians, RobotPose};

use crate::{Bump, CREATE3_RADIUS, Noises};

#[derive(Clone, Debug)]
pub struct CircleObstacles {
    obstacles: Vec<CircleObstacle>,
    estimate_bounding_box: BoundingBox,
    total_strike_distance: f64,
    num_strikes: usize,
    noises: Noises,
}

impl CircleObstacles {
    pub fn new(noises: Noises) -> Self {
        Self {
            obstacles: vec![],
            estimate_bounding_box: BoundingBox::default(),
            total_strike_distance: 0.0,
            num_strikes: 0,
            noises,
        }
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

impl ObstacleMap for CircleObstacles {
    type SensorType = Bump;

    fn error(&mut self, pose: RobotPose<Radians>) -> f64 {
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

    fn noise(&self, sensor_info: Option<&Self::SensorType>) -> Noise {
        self.noises.noise(sensor_info)
    }

    fn sensor_update(&mut self, pose: RobotPose<Radians>, sensor_info: Option<&Self::SensorType>) {
        if let Some(bump) = sensor_info {
            self.obstacles.push(CircleObstacle {
                center: bump.bump_location(pose),
                radius: CREATE3_RADIUS,
            });
        }
        self.estimate_bounding_box.observe(pose.pos[0], pose.pos[1]);
    }

    fn bounding_box(&self) -> BoundingBox {
        //self.obstacles.iter().map(|ob| ob.center).collect()
        self.estimate_bounding_box
    }
}
