use particle_filter::{BoundingBox, FloatPoint, Noise, ObstacleMap, Radians, RobotPose};

use crate::{Bump, Noises};

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

#[derive(Clone)]
pub struct Landmarks {
    landmarks: Vec<CircleObstacle>,
    total_landmark_error: f64,
    total_landmark_attempts: usize,
    noises: Noises,
}

impl Landmarks {
    pub fn new(noises: Noises) -> Self {
        Self {
            landmarks: vec![],
            total_landmark_attempts: 0,
            total_landmark_error: 0.0,
            noises
        }
    }
}

impl ObstacleMap for Landmarks {
    type SensorType = Bump;

    fn error(&self) -> f64 {
        todo!()
    }

    fn sensor_update(&mut self, pose: RobotPose<Radians>, sensor_info: Option<&Self::SensorType>) {
        todo!()
    }

    fn noise(&self, sensor_info: Option<&Self::SensorType>) -> Noise {
        self.noises.noise(sensor_info)
    }

    fn bounding_box(&self) -> BoundingBox {
        todo!()
    }
}